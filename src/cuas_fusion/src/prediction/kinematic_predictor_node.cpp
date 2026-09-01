// @file kinematic_predictor_node.cpp
// @brief ROS 2 node that projects tracks forward and publishes predictions.
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/param_utils.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/track_state_ids.hpp"
#include "cuas_fusion/estimation/imm_filter.hpp"
#include "cuas_fusion/prediction/kinematic_predictor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cuas_msgs/msg/track.hpp>
#include <cuas_msgs/msg/track_array.hpp>
#include <cuas_msgs/msg/predicted_track.hpp>
#include <cuas_msgs/msg/trajectory_waypoints.hpp>
#include <cstdio>

namespace cuas {

struct CovarianceCache {
    Eigen::Matrix<float64_t, 6, 6> P =
        Eigen::Matrix<float64_t, 6, 6>::Identity();
};

class KinematicPredictorNode : public rclcpp::Node
{
public:
    KinematicPredictorNode()
    : Node("kinematic_predictor_node")
    {
        declare_parameter("prediction_horizon_sec", 5.0);
        declare_parameter("prediction_step_dt", 0.1);
        declare_parameter("publish_rate_hz", 20.0);

        horizon_ = get_parameter("prediction_horizon_sec").as_double();
        step_dt_ = get_parameter("prediction_step_dt").as_double();
        float64_t rate = get_parameter("publish_rate_hz").as_double();
        rate = clamp_rate_hz(get_logger(), "publish_rate_hz", rate, 20.0);

        n_steps_ = clamp_prediction_steps(get_logger(), horizon_, step_dt_,
                                          kMaxTrajectorySteps);

        sub_ = create_subscription<cuas_msgs::msg::TrackArray>(
            "/tracks", 10,
            std::bind(&KinematicPredictorNode::track_callback, this, std::placeholders::_1));

        pub_pred_ = create_publisher<cuas_msgs::msg::PredictedTrack>(
            "/predicted_tracks/kinematic", 10);
        pub_traj_ = create_publisher<cuas_msgs::msg::TrajectoryWaypoints>(
            "/trajectory_waypoints/kinematic", 10);

        const int32_t period_ms = static_cast<int32_t>(1000.0 / rate);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&KinematicPredictorNode::publish, this));

        RCLCPP_INFO(get_logger(), "Kinematic predictor node ready (horizon=%.1fs, dt=%.2fs)",
                     horizon_, step_dt_);
    }

private:
    static constexpr float64_t kSigmaASq = 0.25;

    void track_callback(const cuas_msgs::msg::TrackArray::ConstSharedPtr& msg)
    {
        latest_tracks_ = msg;

        const uint32_t n = static_cast<uint32_t>(msg->tracks.size());

        // WHY: imm_tracker_node auto-increments next_track_id_ without bound
        // while cov_cache_ is a FixedMap capped at TRACK_MAX_TRACKS. Old keys
        // are never evicted by insert_or_assign, so after enough track churn
        // the map saturates and insertions for new IDs silently fail, leaving
        // publish() to skip them via the cov_cache_.find == nullptr branch
        // and halting /trajectory_waypoints/kinematic output. Prune keys not
        // present in the current /tracks snapshot before inserting.
        cov_cache_.erase_if(
            [&msg](const uint32_t& id, const CovarianceCache&) -> bool {
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
            // WHY: Track.msg has no covariance field, so each arriving track
            // is treated as a fresh radar measurement that collapses our
            // positional belief back to the initial uncertainty. Without this
            // reset, cache->P grows unboundedly across publish ticks because
            // there is no Kalman update to bound the open-loop propagation.
            CovarianceCache fresh;
            fresh.P = KinematicPredictor::build_initial_covariance_6d();
            (void)cov_cache_.insert_or_assign(t.track_id, fresh);
        }
    }

    void publish()
    {
        if (latest_tracks_ == nullptr) {
            return;
        }
        const uint32_t n = static_cast<uint32_t>(latest_tracks_->tracks.size());
        for (uint32_t ti = 0U; ti < n; ++ti) {
            const cuas_msgs::msg::Track & t = latest_tracks_->tracks[ti];
            if ((t.track_state_id != cuas::track_state::kConfirmed) &&
                (t.track_state_id != cuas::track_state::kReacquired)) {
                continue;
            }

            const Eigen::VectorXd state =
                KinematicPredictor::build_state_from_position_velocity(
                    t.position_x_m, t.position_y_m, t.position_z_m,
                    static_cast<float64_t>(t.vx_mps),
                    static_cast<float64_t>(t.vy_mps),
                    static_cast<float64_t>(t.vz_mps));

            CovarianceCache* cache = cov_cache_.find(t.track_id);
            if (cache == nullptr) {
                continue;
            }
            Eigen::MatrixXd P = cache->P;

            const Eigen::MatrixXd F =
                KinematicPredictor::build_transition_matrix_6d(step_dt_);
            const Eigen::MatrixXd Q =
                KinematicPredictor::build_process_noise_6d(step_dt_, kSigmaASq);

            auto traj = predictor_.propagateForward(state, P, F, Q,
                                                    step_dt_, n_steps_);

            // WHY: cache->P is reset per measurement in track_callback; no
            // inter-tick propagation here — each forecast starts from the
            // bounded measurement-scale covariance, so uncertainty_radii_m
            // reflects only the horizon-local growth inside propagateForward.

            cuas_msgs::msg::PredictedTrack pred;
            pred.header.stamp = this->now();
            pred.header.frame_id = "base_link";
            pred.track_id = t.track_id;

            if (!traj.positions.empty()) {
                const auto& last = traj.positions.back();
                pred.pos_x_m = last.x();
                pred.pos_y_m = last.y();
                pred.pos_z_m = last.z();
            }
            pred.vel_x_mps = state(3);
            pred.vel_y_mps = state(4);
            pred.vel_z_mps = state(5);

            const Eigen::MatrixXd Pcov = cache->P;
            for (uint32_t r = 0U; r < 6U; ++r) {
                for (uint32_t c = 0U; c < 6U; ++c) {
                    const int32_t ri = static_cast<int32_t>(r);
                    const int32_t ci = static_cast<int32_t>(c);
                    pred.covariance[(r * 6U) + c] = Pcov(ri, ci);
                }
            }

            pred.bearing_deg            = traj.final_bearing_deg;
            pred.elevation_deg          = traj.final_elevation_deg;
            // CV is the only forward model (see kinematic_predictor.hpp);
            // the fabricated 0.33/0.33/0.34 blend misrepresented the output.
            pred.model_weight_cv        = 1.0;
            pred.model_weight_ca        = 0.0;
            pred.model_weight_ct        = 0.0;
            pred.track_state            = t.track_state;
            pred.track_state_id         = t.track_state_id;
            // WHY: prediction_horizon_s is stamped onto Track by the tracker
            // node which owns the single /threat/reports join — direct read
            // here, no policy ownership in the predictor.
            pred.prediction_horizon_sec = static_cast<float64_t>(t.prediction_horizon_s);
            pub_pred_->publish(pred);

            cuas_msgs::msg::TrajectoryWaypoints wp;
            wp.header   = pred.header;
            wp.track_id = t.track_id;
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

    KinematicPredictor predictor_;
    float64_t horizon_ = 0.0;
    float64_t step_dt_ = 0.0;
    int32_t   n_steps_ = 0;

    // ConstSharedPtr, not a deep copy: TrackArray copies allocate per tick
    // (A3.6; intent_classifier_node is the reference pattern).
    cuas_msgs::msg::TrackArray::ConstSharedPtr latest_tracks_;
    FixedMap<uint32_t, CovarianceCache, TRACK_MAX_TRACKS> cov_cache_{};

    rclcpp::Subscription<cuas_msgs::msg::TrackArray>::SharedPtr sub_;
    rclcpp::Publisher<cuas_msgs::msg::PredictedTrack>::SharedPtr pub_pred_;
    rclcpp::Publisher<cuas_msgs::msg::TrajectoryWaypoints>::SharedPtr pub_traj_;
    rclcpp::TimerBase::SharedPtr timer_;
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
        auto node = std::make_shared<cuas::KinematicPredictorNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FATAL: unhandled exception in KinematicPredictorNode: %s\n", e.what());
        exit_code = 1;
    } catch (...) {
        std::fprintf(stderr, "FATAL: unhandled non-std exception in KinematicPredictorNode\n");
        exit_code = 1;
    }
    return exit_code;
}
