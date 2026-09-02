// @file reachability_node.cpp
// @brief ROS 2 node that computes per-track intercept estimates at a fixed cadence.
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/param_utils.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/track_state_ids.hpp"
#include "cuas_fusion/reachability_engine.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cuas_msgs/msg/intercept_report.hpp>
#include <cuas_msgs/msg/intercept_report_array.hpp>
#include <cuas_msgs/msg/threat_report_array.hpp>
#include <cuas_msgs/msg/track_array.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <cstdio>

namespace cuas {

// WHY: default per-axis position variance used when Track.msg carries no
// covariance; downstream ellipse sizing stays bounded and non-degenerate.
static constexpr float32_t kDefaultPositionVarianceM2 = 0.25F;

// WHY: builder kept at file scope (not in node class) so all projection and
// covariance scaffolding lives in one named helper outside the callback body
// — node only delegates to it and to ReachabilityEngine.
static ReachabilityTrackState build_track_state(
    const cuas_msgs::msg::Track & t,
    float32_t cv_weight)
{
    ReachabilityTrackState rstate;
    rstate.x_m = t.position_x_m;
    rstate.y_m = t.position_y_m;

    // Track.msg carries the estimated velocity vector; use it directly.
    // Reconstructing velocity as |v| along the position bearing pointed
    // every target radially outward from the sensor, which made
    // speed_toward reduce to -|v| and the published intercept geometry
    // fictitious (never intercept-possible for magnitude velocities).
    rstate.vx_mps = t.vx_mps;
    rstate.vy_mps = t.vy_mps;

    for (uint32_t r = 0U; r < 4U; ++r) {
        for (uint32_t c = 0U; c < 4U; ++c) {
            rstate.P[r][c] = 0.0F;
        }
    }
    rstate.P[0][0] = kDefaultPositionVarianceM2;
    rstate.P[1][1] = kDefaultPositionVarianceM2;

    float32_t clamped_cv = cv_weight;
    if (clamped_cv < 0.0F) {
        clamped_cv = 0.0F;
    }
    if (clamped_cv > 1.0F) {
        clamped_cv = 1.0F;
    }
    rstate.imm_cv_weight = clamped_cv;

    return rstate;
}

class ReachabilityNode : public rclcpp::Node
{
public:
    ReachabilityNode()
    : Node("reachability_node")
    , engine_()
    , threat_priorities_()
    , latest_tracks_()
    , min_threat_level_(1)
    {
        (void)declare_parameter<int32_t>("min_threat_level", 1);
        (void)declare_parameter<float64_t>("publish_rate_hz", 20.0);

        const int64_t   mtl  = get_parameter("min_threat_level").as_int();
        float64_t rate = get_parameter("publish_rate_hz").as_double();
        rate = clamp_rate_hz(get_logger(), "publish_rate_hz", rate, 20.0);
        min_threat_level_ = static_cast<int32_t>(mtl);

        pub_ = create_publisher<cuas_msgs::msg::InterceptReportArray>(
            "/reachability/warnings", 10);

        sub_tracks_ = create_subscription<cuas_msgs::msg::TrackArray>(
            "/tracks", 10,
            std::bind(&ReachabilityNode::tracks_callback,
                      this, std::placeholders::_1));

        sub_threats_ = create_subscription<cuas_msgs::msg::ThreatReportArray>(
            "/threat/reports", 10,
            std::bind(&ReachabilityNode::threats_callback,
                      this, std::placeholders::_1));

        const float64_t period_ms_d = 1000.0 / rate;
        const int32_t   period_ms   = static_cast<int32_t>(period_ms_d);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&ReachabilityNode::publish_tick, this));

        RCLCPP_INFO(get_logger(),
                    "Reachability node ready (min_threat=%d, rate=%.1fHz)",
                    min_threat_level_, rate);
    }

private:
    static int32_t threat_priority_from_id(uint8_t level_id)
    {
        if (level_id == cuas::threat_level::kThreatening) {
            return 2;
        }
        if (level_id == cuas::threat_level::kSuspect) {
            return 1;
        }
        return 0;
    }

    void tracks_callback(const cuas_msgs::msg::TrackArray::ConstSharedPtr& msg)
    {
        latest_tracks_ = msg;
        // Priorities follow the tracker's id set (RC-4): the 32-slot map
        // filled with dead ids and every later track fell below
        // min_threat_level for the life of the process.
        threat_priorities_.erase_if(
            [&msg](const uint32_t& id, const int32_t&) -> bool {
                for (std::size_t k = 0U; k < msg->tracks.size(); ++k) {
                    if (msg->tracks[k].track_id == id) {
                        return false;
                    }
                }
                return true;
            });
    }

    void threats_callback(
        const cuas_msgs::msg::ThreatReportArray::ConstSharedPtr& msg)
    {
        const uint32_t n = static_cast<uint32_t>(msg->reports.size());
        for (uint32_t i = 0U; i < n; ++i) {
            const cuas_msgs::msg::ThreatReport& r = msg->reports[i];
            const int32_t pri = threat_priority_from_id(r.threat_level_id);
            (void)threat_priorities_.insert_or_assign(r.track_id, pri);
        }
    }

    void publish_tick()
    {
        cuas_msgs::msg::InterceptReportArray out;
        out.stamp = this->now();

        const uint32_t n_tracks = (latest_tracks_ == nullptr)
            ? 0U : static_cast<uint32_t>(latest_tracks_->tracks.size());
        for (uint32_t ti = 0U; ti < n_tracks; ++ti) {
            const cuas_msgs::msg::Track& tr = latest_tracks_->tracks[ti];
            const int32_t* pri_ptr = threat_priorities_.find(tr.track_id);
            int32_t pri = 0;
            if (pri_ptr != nullptr) {
                pri = *pri_ptr;
            }
            if (pri < min_threat_level_) {
                continue;
            }

            const float32_t ct_prob = tr.imm_ct_probability;
            const float32_t cv_weight = 1.0F - ct_prob;

            const ReachabilityTrackState rstate = build_track_state(tr, cv_weight);
            const InterceptResult res = engine_.compute(rstate);

            cuas_msgs::msg::InterceptReport rep;
            rep.track_id                        = tr.track_id;
            rep.time_to_intercept_s             = res.time_to_intercept_s;
            rep.intercept_confidence            = res.intercept_confidence;
            rep.covariance_ellipse_major_m      = res.ellipse_major_m;
            rep.covariance_ellipse_minor_m      = res.ellipse_minor_m;
            rep.covariance_ellipse_heading_rad  = res.ellipse_heading_rad;
            rep.intercept_possible              = res.intercept_possible;
            rep.stamp                           = out.stamp;
            out.reports.push_back(rep);
        }

        pub_->publish(out);
    }

    ReachabilityEngine engine_;
    FixedMap<uint32_t, int32_t, TRACK_MAX_TRACKS> threat_priorities_;
    // ConstSharedPtr, not a deep copy: TrackArray copies allocate per tick
    // (A3.6; intent_classifier_node is the reference pattern).
    cuas_msgs::msg::TrackArray::ConstSharedPtr latest_tracks_;
    int32_t min_threat_level_;

    rclcpp::Publisher<cuas_msgs::msg::InterceptReportArray>::SharedPtr pub_;
    rclcpp::Subscription<cuas_msgs::msg::TrackArray>::SharedPtr sub_tracks_;
    rclcpp::Subscription<cuas_msgs::msg::ThreatReportArray>::SharedPtr sub_threats_;
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
        auto node = std::make_shared<cuas::ReachabilityNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FATAL: unhandled exception in ReachabilityNode: %s\n", e.what());
        exit_code = 1;
    } catch (...) {
        std::fprintf(stderr, "FATAL: unhandled non-std exception in ReachabilityNode\n");
        exit_code = 1;
    }
    return exit_code;
}
