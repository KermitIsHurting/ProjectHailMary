// @file imm_tracker_node.cpp
// @brief ROS 2 node wrapping IMMTracker for radar point-cloud input.
#include "cuas_fusion/tracking/imm_tracker.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/types.hpp"
#include "cuas_fusion/common/track_state_ids.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cuas_msgs/msg/track.hpp>
#include <cuas_msgs/msg/track_array.hpp>
#include <cuas_msgs/msg/threat_report_array.hpp>

#include <limits>
#include <string>
#include <utility>
#include <cstdio>

namespace cuas {

static float32_t horizon_for_track(
    uint32_t track_id,
    const cuas_msgs::msg::ThreatReportArray::ConstSharedPtr & reports)
{
    if (reports != nullptr) {
        for (uint32_t i = 0U; i < reports->reports.size(); ++i) {
            if (reports->reports[i].track_id == track_id) {
                return reports->reports[i].prediction_horizon_s;
            }
        }
    }
    return 5.0F;
}

// WHY: single chokepoint string->id translation at the ROS publish boundary
// (DEV-005). All other code compares track_state_id, never the string.
static uint8_t track_state_to_id(const std::string & s)
{
    if (s == "TENTATIVE")  { return cuas::track_state::kTentative; }
    if (s == "CONFIRMED")  { return cuas::track_state::kConfirmed; }
    if (s == "OCCLUDED")   { return cuas::track_state::kOccluded; }
    if (s == "REACQUIRED") { return cuas::track_state::kReacquired; }
    if (s == "LOST")       { return cuas::track_state::kLost; }
    return cuas::track_state::kUnknown;
}

class IMMTrackerNode : public rclcpp::Node
{
public:
    IMMTrackerNode()
    : Node("imm_tracker_node")
    , next_track_id_(1U)
    {
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/radar/detections", 10,
            std::bind(&IMMTrackerNode::radar_callback, this, std::placeholders::_1));

        sub_threats_ = create_subscription<cuas_msgs::msg::ThreatReportArray>(
            "/threat/reports", 10,
            std::bind(&IMMTrackerNode::threats_callback, this, std::placeholders::_1));

        pub_ = create_publisher<cuas_msgs::msg::TrackArray>("/tracks", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&IMMTrackerNode::publish_tracks, this));

        clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
        last_predict_time_ = clock_->now().seconds();

        RCLCPP_INFO(get_logger(), "IMM tracker node ready");
    }

private:
    static constexpr float64_t kAssociationGate = 0.8;

    void threats_callback(const cuas_msgs::msg::ThreatReportArray::ConstSharedPtr& msg)
    {
        latest_threats_ = msg;
    }

    void radar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
    {
        const float64_t now = clock_->now().seconds();

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            const float64_t px = static_cast<float64_t>(*iter_x);
            const float64_t py = static_cast<float64_t>(*iter_y);
            const float64_t pz = static_cast<float64_t>(*iter_z);

            uint32_t  best_id   = 0U;
            float64_t best_dist = kAssociationGate;

            for (uint32_t i = 0U; i < active_tracks_.slot_count(); ++i) {
                auto& slot = active_tracks_.slots()[i];
                if (!slot.occupied) {
                    continue;
                }
                const float64_t d = slot.value.distance_to(px, py, pz);
                if (d < best_dist) {
                    best_dist = d;
                    best_id   = slot.key;
                }
            }

            if (best_id != 0U) {
                IMMTracker* tracker = active_tracks_.find(best_id);
                if (tracker != nullptr) {
                    tracker->update(px, py, pz, now);
                }
            } else {
                const uint32_t new_id = next_track_id_;
                ++next_track_id_;
                IMMTracker fresh(new_id, px, py, pz, now);
                if (!active_tracks_.insert_or_assign(new_id, fresh)) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 1000,
                        "IMM tracker capacity reached (%u), dropping detection",
                        TRACK_MAX_TRACKS);
                }
            }
        }
    }

    void publish_tracks()
    {
        const float64_t now = clock_->now().seconds();
        float64_t dt = now - last_predict_time_;
        last_predict_time_ = now;
        if (dt <= 0.0) {
            dt = 0.05;
        }

        active_tracks_.erase_if(
            [&](uint32_t id, const IMMTracker& tracker) {
                (void)id;
                return (now - tracker.lastUpdateTime()) > 5.0;
            });

        for (uint32_t i = 0U; i < active_tracks_.slot_count(); ++i) {
            auto& slot = active_tracks_.slots()[i];
            if (!slot.occupied) {
                continue;
            }
            slot.value.predict(dt);
        }

        cuas_msgs::msg::TrackArray out;
        out.header.stamp    = this->now();
        out.header.frame_id = "base_link";
        out.tracks.reserve(active_tracks_.size());

        for (uint32_t i = 0U; i < active_tracks_.slot_count(); ++i) {
            const auto& slot = active_tracks_.slots()[i];
            if (!slot.occupied) {
                continue;
            }
            const IMMTracker& tracker = slot.value;
            cuas_msgs::msg::Track t;
            t.track_id = tracker.getTrackId();
            const Eigen::VectorXd pos = tracker.getPosition();
            const Eigen::VectorXd vel = tracker.getVelocity();
            t.position_x_m = static_cast<float32_t>(pos(0));
            t.position_y_m = static_cast<float32_t>(pos(1));
            t.position_z_m = static_cast<float32_t>(pos(2));
            t.velocity_mps = static_cast<float32_t>(tracker.speed());
            t.vx_mps       = static_cast<float32_t>(vel(0));
            t.vy_mps       = static_cast<float32_t>(vel(1));
            t.doppler_mps  = 0.0F;
            t.class_label  = "unknown";
            t.confidence   = tracker.getConfidence();
            t.track_state  = trackStateToString(tracker.getState());
            t.track_state_id = track_state_to_id(t.track_state);
            t.timestamp_ns = static_cast<int64_t>(tracker.lastUpdateTime() * 1.0e9);
            t.is_maneuvering     = tracker.isManeuvering();
            t.imm_ct_probability = tracker.getCtProbability();
            // WHY: tracker owns Track enrichment and is the single join point
            // between threat policy and the Track message — predictors read
            // prediction_horizon_s directly with no additional subscriptions.
            t.prediction_horizon_s = horizon_for_track(t.track_id, latest_threats_);
            out.tracks.push_back(t);
        }

        pub_->publish(out);
    }

    FixedMap<uint32_t, IMMTracker, TRACK_MAX_TRACKS> active_tracks_;
    uint32_t next_track_id_;
    // ConstSharedPtr, not a deep copy: ThreatReportArray copies allocate per
    // scan (A3.6; intent_classifier_node is the reference pattern).
    cuas_msgs::msg::ThreatReportArray::ConstSharedPtr latest_threats_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Subscription<cuas_msgs::msg::ThreatReportArray>::SharedPtr sub_threats_;
    rclcpp::Publisher<cuas_msgs::msg::TrackArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::Clock> clock_;
    float64_t last_predict_time_ = 0.0;
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
        auto node = std::make_shared<cuas::IMMTrackerNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FATAL: unhandled exception in IMMTrackerNode: %s\n", e.what());
        exit_code = 1;
    } catch (...) {
        std::fprintf(stderr, "FATAL: unhandled non-std exception in IMMTrackerNode\n");
        exit_code = 1;
    }
    return exit_code;
}
