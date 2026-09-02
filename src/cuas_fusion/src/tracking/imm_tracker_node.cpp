// @file imm_tracker_node.cpp
// @brief ROS 2 node wrapping IMMTracker for radar point-cloud input.
#include "cuas_fusion/tracking/imm_tracker.hpp"
#include "cuas_fusion/common/clock.hpp"
#include "cuas_fusion/common/eigen_types.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/ros_pointcloud_adapter.hpp"
#include "cuas_fusion/common/types.hpp"
#include "cuas_fusion/common/track_state_ids.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cuas_msgs/msg/track.hpp>
#include <cuas_msgs/msg/track_array.hpp>
#include <cuas_msgs/msg/threat_report_array.hpp>

#include <array>
#include <cmath>
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
    if (s == "COASTED")    { return cuas::track_state::kCoasted; }
    if (s == "DELETED")    { return cuas::track_state::kDeleted; }
    return cuas::track_state::kUnknown;
}

static builtin_interfaces::msg::Time stamp_from_ns(int64_t ns)
{
    builtin_interfaces::msg::Time t;
    t.sec     = static_cast<int32_t>(ns / 1'000'000'000LL);
    t.nanosec = static_cast<uint32_t>(ns % 1'000'000'000LL);
    return t;
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

        // Log throttling only. Predict/evict/stamp time comes from
        // cuas::now_ns() (CLOCK_MONOTONIC), the clock the drivers stamp
        // with; RCL_STEADY_TIME is CLOCK_MONOTONIC_RAW, which drifted
        // 415 ms from it under NTP slew and broke fusion's 150 ms gate
        // after ~75 min of uptime (RC-6, audit F-9).
        clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
        last_predict_time_ = static_cast<float64_t>(cuas::now_ns()) * 1.0e-9;

        RCLCPP_INFO(get_logger(), "IMM tracker node ready");
    }

private:
    void threats_callback(const cuas_msgs::msg::ThreatReportArray::ConstSharedPtr& msg)
    {
        latest_threats_ = msg;
    }

    void radar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
    {
        // A cloud with a foreign layout used to throw out of the iterator
        // constructor into the process exception boundary (RC-8); an empty
        // frame is a valid "nothing seen" from the parser (RC-12) and has
        // no data for the iterators to point at.
        if (!cloudHasFloat32Xyz(*msg)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 5000,
                "Dropping PointCloud2 without float32 x/y/z fields");
            return;
        }
        if (cloudIsEmpty(*msg)) {
            return;
        }

        // Measurement time is the SENSOR stamp (CLOCK_MONOTONIC from the
        // parser), not arrival time: update() velocity gating and fusion's
        // camera alignment both need the instant the return was observed,
        // free of transport jitter (P2.1).
        const float64_t now =
            static_cast<float64_t>(msg->header.stamp.sec) +
            (static_cast<float64_t>(msg->header.stamp.nanosec) * 1.0e-9);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        // One update per track per frame: the nearest gated return wins,
        // the rest spawn. Without this, every return of a dense cloud
        // updated the same track and two close targets merged (RC-3).
        std::array<bool, TRACK_MAX_TRACKS> used{};
        uint32_t dropped_non_finite = 0U;

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            const float64_t px = static_cast<float64_t>(*iter_x);
            const float64_t py = static_cast<float64_t>(*iter_y);
            const float64_t pz = static_cast<float64_t>(*iter_z);
            if (!(std::isfinite(px) && std::isfinite(py) && std::isfinite(pz))) {
                ++dropped_non_finite;   // RC-9: NaN in, NaN track out
                continue;
            }

            uint32_t  best_slot = TRACK_MAX_TRACKS;
            float64_t best_dist = std::numeric_limits<float64_t>::infinity();

            for (uint32_t i = 0U; i < active_tracks_.slot_count(); ++i) {
                const auto& slot = active_tracks_.slots()[i];
                if (!slot.occupied || used[i]) {
                    continue;
                }
                // Gate against the position extrapolated to the return's
                // stamp, widened by this track's uncertainty (RC-3, D-9).
                const float64_t d = slot.value.distanceAt(px, py, pz, now);
                if ((d <= slot.value.associationGateM(now)) && (d < best_dist)) {
                    best_dist = d;
                    best_slot = i;
                }
            }

            if (best_slot < TRACK_MAX_TRACKS) {
                active_tracks_.slots()[best_slot].value.update(px, py, pz, now);
                used[best_slot] = true;
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

        if (dropped_non_finite > 0U) {
            RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 5000,
                "Dropped %u non-finite radar returns", dropped_non_finite);
        }
    }

    void publish_tracks()
    {
        const int64_t   t_now_ns = cuas::now_ns();
        const float64_t now      = static_cast<float64_t>(t_now_ns) * 1.0e-9;
        float64_t dt = now - last_predict_time_;
        last_predict_time_ = now;
        if (dt <= 0.0) {
            dt = 0.05;
        }

        active_tracks_.erase_if(
            [&](uint32_t id, const IMMTracker& tracker) {
                (void)id;
                return (now - tracker.lastUpdateTime()) > kTrackReapAfterS;
            });

        for (uint32_t i = 0U; i < active_tracks_.slot_count(); ++i) {
            auto& slot = active_tracks_.slots()[i];
            if (!slot.occupied) {
                continue;
            }
            slot.value.predict(dt);
        }

        cuas_msgs::msg::TrackArray out;
        // CLOCK_MONOTONIC stamp, the drivers' clock: /tracks is on the
        // measurement path and fusion aligns it against camera stamps in
        // the same domain. this->now() (system time) is NOT comparable (P2.1).
        out.header.stamp    = stamp_from_ns(t_now_ns);
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
            const Eigen::Vector3d pos = tracker.getPosition();
            const Eigen::Vector3d vel = tracker.getVelocity();
            t.position_x_m = static_cast<float32_t>(pos(0));
            t.position_y_m = static_cast<float32_t>(pos(1));
            t.position_z_m = static_cast<float32_t>(pos(2));
            t.velocity_mps = static_cast<float32_t>(tracker.speed());
            t.vx_mps       = static_cast<float32_t>(vel(0));
            t.vy_mps       = static_cast<float32_t>(vel(1));
            // Line-of-sight speed from the estimate, negative = closing.
            // Was hard-wired 0, which made THREATENING unreachable (RC-1).
            t.doppler_mps  = static_cast<float32_t>(tracker.radialSpeed());
            t.class_label  = "unknown";
            t.confidence   = tracker.getConfidence();
            t.track_state  = trackStateToString(tracker.getState());
            t.track_state_id = track_state_to_id(t.track_state);
            t.timestamp_ns = static_cast<int64_t>(tracker.lastUpdateTime() * 1.0e9);
            t.vz_mps = static_cast<float32_t>(vel(2));
            // P3.1: real covariance on the wire (mixed IMM 6x6 triangle);
            // acceleration stays 0 — the legacy cascade has no CA output.
            packUpperTriangle6(tracker.getCovariance(), t.covariance);
            t.source_mask = track_source::kRadar;
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
