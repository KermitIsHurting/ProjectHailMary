// @file occlusion_predictor_node.cpp
// @brief ROS 2 node that maintains ghost tracks and publishes their forecasts.
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/param_utils.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/track_state_ids.hpp"
#include "cuas_fusion/prediction/kinematic_predictor.hpp"
#include "cuas_fusion/prediction/occlusion_predictor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cuas_msgs/msg/track.hpp>
#include <cuas_msgs/msg/track_array.hpp>
#include <cuas_msgs/msg/predicted_track.hpp>
#include <cuas_msgs/msg/trajectory_waypoints.hpp>
#include <cstdio>

namespace cuas {

class OcclusionPredictorNode : public rclcpp::Node
{
public:
    OcclusionPredictorNode()
    : Node("occlusion_predictor_node")
    {
        declare_parameter("prediction_horizon_sec", 5.0);
        declare_parameter("prediction_step_dt", 0.1);
        declare_parameter("publish_rate_hz", 20.0);
        declare_parameter("max_occlusion_sec", 4.0);
        declare_parameter("mahalanobis_gate", 3.0);

        horizon_ = get_parameter("prediction_horizon_sec").as_double();
        step_dt_ = get_parameter("prediction_step_dt").as_double();
        float64_t rate    = get_parameter("publish_rate_hz").as_double();
        rate = clamp_rate_hz(get_logger(), "publish_rate_hz", rate, 20.0);
        const float64_t max_occ = get_parameter("max_occlusion_sec").as_double();
        const float64_t gate    = get_parameter("mahalanobis_gate").as_double();

        n_steps_ = clamp_prediction_steps(get_logger(), horizon_, step_dt_,
                                          kMaxTrajectorySteps);
        predictor_.configure(max_occ, gate);

        sub_ = create_subscription<cuas_msgs::msg::TrackArray>(
            "/tracks", 10,
            std::bind(&OcclusionPredictorNode::track_callback, this, std::placeholders::_1));

        pub_pred_ = create_publisher<cuas_msgs::msg::PredictedTrack>(
            "/predicted_tracks/occlusion", 10);
        pub_traj_ = create_publisher<cuas_msgs::msg::TrajectoryWaypoints>(
            "/trajectory_waypoints/occlusion", 10);

        clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

        const int32_t period_ms = static_cast<int32_t>(1000.0 / rate);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&OcclusionPredictorNode::publish, this));

        RCLCPP_INFO(get_logger(), "Occlusion predictor node ready");
    }

private:
    static constexpr float64_t kSigmaASq = 0.25;

    void track_callback(const cuas_msgs::msg::TrackArray::ConstSharedPtr& msg)
    {
        const float64_t now = clock_->now().seconds();

        const uint32_t n = static_cast<uint32_t>(msg->tracks.size());

        // WHY: horizon_cache_ is a FixedMap capped at TRACK_MAX_TRACKS and
        // insert_or_assign never evicts. imm_tracker_node's track_id grows
        // unboundedly, so stale keys would fill the map and block caching
        // for newer IDs. Prune keys not present in the current /tracks
        // snapshot before inserting.
        horizon_cache_.erase_if(
            [&msg](const uint32_t& id, const float32_t&) -> bool {
                const uint32_t m = static_cast<uint32_t>(msg->tracks.size());
                for (uint32_t k = 0U; k < m; ++k) {
                    if (msg->tracks[k].track_id == id) {
                        return false;
                    }
                }
                return true;
            });

        for (uint32_t i = 0U; i < n; ++i) {
            const cuas_msgs::msg::Track & t = msg->tracks[i];
            (void)horizon_cache_.insert_or_assign(t.track_id, t.prediction_horizon_s);
            if (t.track_state_id == cuas::track_state::kOccluded) {
                if (ghost_tracks_.find(t.track_id) == nullptr) {
                    OcclusionPredictor::GhostTrack ghost;
                    ghost.track_id = t.track_id;
                    ghost.state    = KinematicPredictor::build_state_from_position_velocity(
                        t.position_x_m, t.position_y_m, t.position_z_m,
                        static_cast<float64_t>(t.vx_mps),
                        static_cast<float64_t>(t.vy_mps),
                        static_cast<float64_t>(t.vz_mps));
                    ghost.covariance = KinematicPredictor::build_initial_covariance_6d();
                    ghost.occlusion_start_time_sec = now;
                    (void)ghost_tracks_.insert_or_assign(t.track_id, ghost);
                }
            } else if ((t.track_state_id == cuas::track_state::kConfirmed) ||
                       (t.track_state_id == cuas::track_state::kReacquired)) {
                OcclusionPredictor::GhostTrack* ghost = ghost_tracks_.find(t.track_id);
                if (ghost != nullptr) {
                    if (predictor_.reacquire(*ghost,
                                             t.position_x_m,
                                             t.position_y_m,
                                             t.position_z_m)) {
                        (void)ghost_tracks_.erase(t.track_id);
                    }
                }
            } else if (t.track_state_id == cuas::track_state::kLost) {
                (void)ghost_tracks_.erase(t.track_id);
            } else {
                // intentionally empty: other states are no-ops
            }
        }
    }

    void publish()
    {
        const float64_t now = clock_->now().seconds();

        const Eigen::MatrixXd F =
            KinematicPredictor::build_transition_matrix_6d(step_dt_);
        const Eigen::MatrixXd Q =
            KinematicPredictor::build_process_noise_6d(step_dt_, kSigmaASq);

        for (uint32_t s = 0U; s < ghost_tracks_.slot_count(); ++s) {
            auto& slot = ghost_tracks_.slots()[s];
            if (!slot.occupied) {
                continue;
            }
            OcclusionPredictor::GhostTrack& ghost = slot.value;

            auto traj = predictor_.propagateGhost(ghost, now, step_dt_, n_steps_, F, Q);
            if (ghost.expired) {
                slot.occupied = false;
                continue;
            }
            if (traj.positions.empty()) {
                continue;
            }

            cuas_msgs::msg::PredictedTrack pred;
            pred.header.stamp    = this->now();
            pred.header.frame_id = "base_link";
            pred.track_id        = slot.key;

            const auto& last = traj.positions.back();
            pred.pos_x_m = last.x();
            pred.pos_y_m = last.y();
            pred.pos_z_m = last.z();
            pred.vel_x_mps = ghost.state(3);
            pred.vel_y_mps = ghost.state(4);
            pred.vel_z_mps = ghost.state(5);

            for (uint32_t r = 0U; r < 6U; ++r) {
                for (uint32_t c = 0U; c < 6U; ++c) {
                    const int32_t ri = static_cast<int32_t>(r);
                    const int32_t ci = static_cast<int32_t>(c);
                    pred.covariance[(r * 6U) + c] = ghost.covariance(ri, ci);
                }
            }

            pred.bearing_deg            = traj.final_bearing_deg;
            pred.elevation_deg          = traj.final_elevation_deg;
            // CV is the only forward model (see kinematic_predictor.hpp);
            // the fabricated 0.33/0.33/0.34 blend misrepresented the output.
            pred.model_weight_cv        = 1.0;
            pred.model_weight_ca        = 0.0;
            pred.model_weight_ct        = 0.0;
            pred.track_state            = "OCCLUDED";
            pred.track_state_id         = cuas::track_state::kOccluded;
            // WHY: prediction_horizon_s is stamped onto Track by the tracker
            // node which owns the single /threat/reports join — direct read
            // here, no policy ownership in the predictor.
            const float32_t* h_ptr = horizon_cache_.find(slot.key);
            float32_t horizon_value = 5.0F;
            if (h_ptr != nullptr) {
                horizon_value = *h_ptr;
            }
            pred.prediction_horizon_sec = static_cast<float64_t>(horizon_value);
            pub_pred_->publish(pred);

            cuas_msgs::msg::TrajectoryWaypoints wp;
            wp.header   = pred.header;
            wp.track_id = slot.key;
            const uint32_t nwp = static_cast<uint32_t>(traj.positions.size());
            for (uint32_t i = 0U; i < nwp; ++i) {
                wp.waypoints_x_m.push_back(traj.positions[i].x());
                wp.waypoints_y_m.push_back(traj.positions[i].y());
                wp.waypoints_z_m.push_back(traj.positions[i].z());
                wp.timestamps_sec.push_back(traj.timestamps_sec[i]);
                wp.uncertainty_radii_m.push_back(traj.uncertainty_radii_m[i]);
                wp.bearing_deg.push_back(traj.bearing_deg[i]);
                wp.elevation_deg.push_back(traj.elevation_deg[i]);
            }
            pub_traj_->publish(wp);
        }
    }

    OcclusionPredictor predictor_;
    float64_t horizon_ = 0.0;
    float64_t step_dt_ = 0.0;
    int32_t   n_steps_ = 0;

    FixedMap<uint32_t, OcclusionPredictor::GhostTrack, TRACK_MAX_TRACKS> ghost_tracks_{};
    FixedMap<uint32_t, float32_t, TRACK_MAX_TRACKS> horizon_cache_{};

    rclcpp::Subscription<cuas_msgs::msg::TrackArray>::SharedPtr sub_;
    rclcpp::Publisher<cuas_msgs::msg::PredictedTrack>::SharedPtr pub_pred_;
    rclcpp::Publisher<cuas_msgs::msg::TrajectoryWaypoints>::SharedPtr pub_traj_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::Clock> clock_;
};

} // namespace cuas

// Single sanctioned exception boundary (DEV-001): owned code never
// throws, but rclcpp/rmw, parameter access, and bad_alloc can. Without
// this handler a library throw becomes std::terminate with no fault
// record, invisible to the health monitor. Catch by const ref per
// MISRA C++:2023 18.3.2.
int main(int argc, char** argv)
{
    int exit_code = 0;
    try {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<cuas::OcclusionPredictorNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FATAL: unhandled exception in OcclusionPredictorNode: %s\n", e.what());
        exit_code = 1;
    } catch (...) {
        std::fprintf(stderr, "FATAL: unhandled non-std exception in OcclusionPredictorNode\n");
        exit_code = 1;
    }
    return exit_code;
}
