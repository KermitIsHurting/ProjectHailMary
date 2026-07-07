// @file prediction_mux_node.cpp
// @brief ROS 2 node merging kinematic and occlusion prediction streams.
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/track_state_ids.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cuas_msgs/msg/predicted_track.hpp>
#include <cuas_msgs/msg/trajectory_waypoints.hpp>
#include <cstdio>

namespace cuas {

class PredictionMuxNode : public rclcpp::Node
{
public:
    PredictionMuxNode()
    : Node("prediction_mux_node")
    {
        sub_kin_pred_ = create_subscription<cuas_msgs::msg::PredictedTrack>(
            "/predicted_tracks/kinematic", 10,
            std::bind(&PredictionMuxNode::kin_pred_callback, this, std::placeholders::_1));

        sub_occ_pred_ = create_subscription<cuas_msgs::msg::PredictedTrack>(
            "/predicted_tracks/occlusion", 10,
            std::bind(&PredictionMuxNode::occ_pred_callback, this, std::placeholders::_1));

        sub_kin_traj_ = create_subscription<cuas_msgs::msg::TrajectoryWaypoints>(
            "/trajectory_waypoints/kinematic", 10,
            std::bind(&PredictionMuxNode::kin_traj_callback, this, std::placeholders::_1));

        sub_occ_traj_ = create_subscription<cuas_msgs::msg::TrajectoryWaypoints>(
            "/trajectory_waypoints/occlusion", 10,
            std::bind(&PredictionMuxNode::occ_traj_callback, this, std::placeholders::_1));

        pub_pred_ = create_publisher<cuas_msgs::msg::PredictedTrack>(
            "/predicted_tracks", 10);
        pub_traj_ = create_publisher<cuas_msgs::msg::TrajectoryWaypoints>(
            "/trajectory_waypoints", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PredictionMuxNode::merge_tick, this));

        RCLCPP_INFO(get_logger(), "Prediction mux node ready");
    }

private:
    void kin_pred_callback(const cuas_msgs::msg::PredictedTrack::ConstSharedPtr& msg)
    {
        (void)kinematic_pred_.insert_or_assign(msg->track_id, *msg);
    }

    void occ_pred_callback(const cuas_msgs::msg::PredictedTrack::ConstSharedPtr& msg)
    {
        (void)occlusion_pred_.insert_or_assign(msg->track_id, *msg);
    }

    void kin_traj_callback(const cuas_msgs::msg::TrajectoryWaypoints::ConstSharedPtr& msg)
    {
        (void)kinematic_traj_.insert_or_assign(msg->track_id, *msg);
    }

    void occ_traj_callback(const cuas_msgs::msg::TrajectoryWaypoints::ConstSharedPtr& msg)
    {
        (void)occlusion_traj_.insert_or_assign(msg->track_id, *msg);
    }

    void merge_tick()
    {
        // WHY: evict cached entries whose source predictor has stopped
        // publishing for them (track exited /tracks). Without this the
        // FixedMap accumulates dead IDs and keeps republishing them each
        // tick, so downstream consumers see phantom track_ids that no
        // longer exist in /tracks. Direct ns arithmetic (instead of
        // rclcpp::Time subtraction) sidesteps the clock-type throw path
        // since JSF forbids try/catch.
        const int64_t now_ns = this->now().nanoseconds();
        auto age_ns = [now_ns](const builtin_interfaces::msg::Time& s) -> int64_t {
            const int64_t ts_ns = (static_cast<int64_t>(s.sec) * 1000000000LL) +
                                   static_cast<int64_t>(s.nanosec);
            return now_ns - ts_ns;
        };
        const int64_t stale_ns =
            static_cast<int64_t>(cuas::kPredictionStaleSec * 1.0e9);
        auto stale_pred = [&age_ns, stale_ns](const uint32_t&,
                                              const cuas_msgs::msg::PredictedTrack& m) {
            return age_ns(m.header.stamp) > stale_ns;
        };
        auto stale_traj = [&age_ns, stale_ns](const uint32_t&,
                                              const cuas_msgs::msg::TrajectoryWaypoints& m) {
            return age_ns(m.header.stamp) > stale_ns;
        };
        kinematic_pred_.erase_if(stale_pred);
        occlusion_pred_.erase_if(stale_pred);
        kinematic_traj_.erase_if(stale_traj);
        occlusion_traj_.erase_if(stale_traj);

        FixedVector<uint32_t, TRACK_MAX_TRACKS * 2U> seen;
        for (uint32_t i = 0U; i < kinematic_pred_.slot_count(); ++i) {
            const auto& slot = kinematic_pred_.slots()[i];
            if (slot.occupied) {
                (void)seen.push_back(slot.key);
            }
        }
        for (uint32_t i = 0U; i < occlusion_pred_.slot_count(); ++i) {
            const auto& slot = occlusion_pred_.slots()[i];
            if (!slot.occupied) {
                continue;
            }
            bool already = false;
            for (uint32_t k = 0U; k < seen.size(); ++k) {
                if (seen[k] == slot.key) {
                    already = true;
                    break;
                }
            }
            if (!already) {
                (void)seen.push_back(slot.key);
            }
        }

        for (uint32_t idx = 0U; idx < seen.size(); ++idx) {
            const uint32_t id = seen[idx];

            const cuas_msgs::msg::PredictedTrack* kit = kinematic_pred_.find(id);
            const cuas_msgs::msg::PredictedTrack* oit = occlusion_pred_.find(id);

            const cuas_msgs::msg::PredictedTrack* chosen = nullptr;

            if ((kit != nullptr) && (oit != nullptr)) {
                if (oit->track_state_id == cuas::track_state::kOccluded) {
                    chosen = oit;
                } else {
                    chosen = kit;
                }
            } else if (oit != nullptr) {
                chosen = oit;
            } else if (kit != nullptr) {
                chosen = kit;
            } else {
                chosen = nullptr;
            }

            if (chosen != nullptr) {
                // WHY: horizon arrives pre-stamped from the predictors because the
                // classifier owns threat policy; the mux only arbitrates streams.
                pub_pred_->publish(*chosen);
            }

            const cuas_msgs::msg::TrajectoryWaypoints* kt = kinematic_traj_.find(id);
            const cuas_msgs::msg::TrajectoryWaypoints* ot = occlusion_traj_.find(id);

            const cuas_msgs::msg::TrajectoryWaypoints* chosen_t = nullptr;

            if ((kt != nullptr) && (ot != nullptr)) {
                if ((oit != nullptr) && (oit->track_state_id == cuas::track_state::kOccluded)) {
                    chosen_t = ot;
                } else {
                    chosen_t = kt;
                }
            } else if (ot != nullptr) {
                chosen_t = ot;
            } else if (kt != nullptr) {
                chosen_t = kt;
            } else {
                chosen_t = nullptr;
            }

            if (chosen_t != nullptr) {
                pub_traj_->publish(*chosen_t);
            }
        }
    }

    FixedMap<uint32_t, cuas_msgs::msg::PredictedTrack,       TRACK_MAX_TRACKS> kinematic_pred_{};
    FixedMap<uint32_t, cuas_msgs::msg::PredictedTrack,       TRACK_MAX_TRACKS> occlusion_pred_{};
    FixedMap<uint32_t, cuas_msgs::msg::TrajectoryWaypoints,  TRACK_MAX_TRACKS> kinematic_traj_{};
    FixedMap<uint32_t, cuas_msgs::msg::TrajectoryWaypoints,  TRACK_MAX_TRACKS> occlusion_traj_{};

    rclcpp::Subscription<cuas_msgs::msg::PredictedTrack>::SharedPtr sub_kin_pred_;
    rclcpp::Subscription<cuas_msgs::msg::PredictedTrack>::SharedPtr sub_occ_pred_;
    rclcpp::Subscription<cuas_msgs::msg::TrajectoryWaypoints>::SharedPtr sub_kin_traj_;
    rclcpp::Subscription<cuas_msgs::msg::TrajectoryWaypoints>::SharedPtr sub_occ_traj_;

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
        auto node = std::make_shared<cuas::PredictionMuxNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FATAL: unhandled exception in PredictionMuxNode: %s\n", e.what());
        exit_code = 1;
    } catch (...) {
        std::fprintf(stderr, "FATAL: unhandled non-std exception in PredictionMuxNode\n");
        exit_code = 1;
    }
    return exit_code;
}
