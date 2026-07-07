// @file geofence_node.cpp
// @brief ROS 2 node wrapping GeofenceEngine with per-track zone transition tracking.
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/param_utils.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/track_state_ids.hpp"
#include "cuas_fusion/geofence_engine.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cuas_msgs/msg/geofence_event.hpp>
#include <cuas_msgs/msg/geofence_event_array.hpp>
#include <cuas_msgs/msg/threat_report_array.hpp>
#include <cuas_msgs/msg/track_array.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>

namespace cuas {

// WHY: yaml-cpp may raise exceptions on malformed input; config load happens
// only at node init and a bad file is treated as an unrecoverable startup error.

static constexpr uint8_t kNoZoneIndex = 0xFFU;

class GeofenceNode : public rclcpp::Node
{
public:
    GeofenceNode()
    : Node("geofence_node")
    , engine_()
    , zone_membership_()
    , threat_priorities_()
    , latest_tracks_()
    {
        (void)declare_parameter<std::string>("geofence_config_path", std::string{});
        (void)declare_parameter<float64_t>("publish_rate_hz", 10.0);

        const std::string path = get_parameter("geofence_config_path").as_string();
        float64_t   rate = get_parameter("publish_rate_hz").as_double();
        rate = clamp_rate_hz(get_logger(), "publish_rate_hz", rate, 10.0);

        FixedVector<ZoneConfig, GEOFENCE_MAX_ZONES> configs;
        if (!parse_zones_yaml(path, configs)) {
            RCLCPP_FATAL(get_logger(),
                         "Failed to load geofence zones from '%s'", path.c_str());
            rclcpp::shutdown();
            return;
        }
        engine_.load_zones(configs);

        pub_events_ = create_publisher<cuas_msgs::msg::GeofenceEventArray>(
            "/geofence/violations", 10);

        sub_tracks_ = create_subscription<cuas_msgs::msg::TrackArray>(
            "/tracks", 10,
            std::bind(&GeofenceNode::tracks_callback, this, std::placeholders::_1));

        sub_threats_ = create_subscription<cuas_msgs::msg::ThreatReportArray>(
            "/threat/reports", 10,
            std::bind(&GeofenceNode::threats_callback, this, std::placeholders::_1));

        const float64_t period_ms_d = 1000.0 / rate;
        const int32_t   period_ms   = static_cast<int32_t>(period_ms_d);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&GeofenceNode::publish_tick, this));

        RCLCPP_INFO(get_logger(),
                    "Geofence node ready (zones=%u, rate=%.1fHz)",
                    engine_.zone_count(), rate);
    }

private:
    bool parse_zones_yaml(const std::string& path,
                          FixedVector<ZoneConfig, GEOFENCE_MAX_ZONES>& out) const
    {
        if (path.empty()) {
            return false;
        }
        const YAML::Node root = YAML::LoadFile(path);
        if (!root["zones"]) {
            return false;
        }
        const YAML::Node zones_node = root["zones"];
        if (!zones_node.IsSequence()) {
            return false;
        }

        const uint32_t n_zones = static_cast<uint32_t>(zones_node.size());
        for (uint32_t zi = 0U; zi < n_zones; ++zi) {
            const YAML::Node zn = zones_node[zi];
            ZoneConfig cfg;

            if (zn["id"]) {
                const std::string id_str = zn["id"].as<std::string>();
                const uint32_t id_len_raw = static_cast<uint32_t>(id_str.size());
                const uint32_t max_copy = GEOFENCE_ZONE_ID_LEN - 1U;
                uint32_t n_copy = max_copy;
                if (id_len_raw < max_copy) {
                    n_copy = id_len_raw;
                }
                for (uint32_t i = 0U; i < n_copy; ++i) {
                    cfg.id[i] = id_str[i];
                }
            }

            if (zn["type"]) {
                const std::string type_str = zn["type"].as<std::string>();
                if (type_str == "polygon") {
                    cfg.type = static_cast<uint8_t>(ZoneShape::POLYGON);
                } else {
                    cfg.type = static_cast<uint8_t>(ZoneShape::CIRCLE);
                }
            }

            if (zn["radius_m"]) {
                cfg.radius_m = zn["radius_m"].as<float32_t>();
            }
            if (zn["center_x_m"]) {
                cfg.center_x_m = zn["center_x_m"].as<float32_t>();
            }
            if (zn["center_y_m"]) {
                cfg.center_y_m = zn["center_y_m"].as<float32_t>();
            }
            if (zn["alert_on_entry"]) {
                cfg.alert_on_entry = zn["alert_on_entry"].as<bool>();
            }

            if (zn["vertices"] && zn["vertices"].IsSequence()) {
                const YAML::Node verts = zn["vertices"];
                const uint32_t nv = static_cast<uint32_t>(verts.size());
                for (uint32_t vi = 0U; vi < nv; ++vi) {
                    const YAML::Node v = verts[vi];
                    if (v.IsSequence() && (v.size() == 2U)) {
                        (void)cfg.vertices_x.push_back(v[0].as<float32_t>());
                        (void)cfg.vertices_y.push_back(v[1].as<float32_t>());
                    }
                }
            }

            if (!out.push_back(cfg)) {
                break;
            }
        }
        return out.size() > 0U;
    }

    void tracks_callback(const cuas_msgs::msg::TrackArray::ConstSharedPtr& msg)
    {
        latest_tracks_ = msg;
    }

    void threats_callback(
        const cuas_msgs::msg::ThreatReportArray::ConstSharedPtr& msg)
    {
        const uint32_t n = static_cast<uint32_t>(msg->reports.size());
        for (uint32_t i = 0U; i < n; ++i) {
            const cuas_msgs::msg::ThreatReport& r = msg->reports[i];
            uint8_t pri = 0U;
            if (r.threat_level_id == cuas::threat_level::kThreatening) {
                pri = 2U;
            } else if (r.threat_level_id == cuas::threat_level::kSuspect) {
                pri = 1U;
            } else {
                pri = 0U;
            }
            (void)threat_priorities_.insert_or_assign(r.track_id, pri);
        }
    }

    uint8_t find_zone_index(const char* zone_id) const
    {
        const FixedVector<ZoneConfig, GEOFENCE_MAX_ZONES>& zs = engine_.zones();
        for (uint32_t i = 0U; i < zs.size(); ++i) {
            bool match = true;
            for (uint32_t k = 0U; k < GEOFENCE_ZONE_ID_LEN; ++k) {
                const char a = zs[i].id[k];
                const char b = zone_id[k];
                if (a != b) {
                    match = false;
                    break;
                }
                if (a == '\0') {
                    break;
                }
            }
            if (match) {
                return static_cast<uint8_t>(i);
            }
        }
        return kNoZoneIndex;
    }

    void append_event(cuas_msgs::msg::GeofenceEventArray& arr,
                      uint32_t track_id,
                      uint8_t zone_idx,
                      GeofenceEventType type,
                      float32_t distance_m) const
    {
        if (zone_idx == kNoZoneIndex) {
            return;
        }
        const ZoneConfig& z = engine_.zones()[zone_idx];
        cuas_msgs::msg::GeofenceEvent ev;
        ev.zone_id   = std::string(&z.id[0]);
        ev.track_id  = track_id;
        ev.event_type = static_cast<uint8_t>(type);
        ev.distance_m = distance_m;
        ev.stamp      = arr.stamp;
        arr.events.push_back(ev);
    }

    void publish_tick()
    {
        cuas_msgs::msg::GeofenceEventArray out;
        out.stamp = this->now();

        const uint32_t n_tracks = (latest_tracks_ == nullptr)
            ? 0U : static_cast<uint32_t>(latest_tracks_->tracks.size());
        for (uint32_t ti = 0U; ti < n_tracks; ++ti) {
            const cuas_msgs::msg::Track& track = latest_tracks_->tracks[ti];
            if (track.track_state_id != cuas::track_state::kConfirmed) {
                continue;
            }

            GeofenceResult result;
            const bool triggered = engine_.evaluate(
                track.position_x_m, track.position_y_m, track.track_id, result);

            const uint8_t* prev_ptr = zone_membership_.find(track.track_id);
            uint8_t prev_zone_idx = kNoZoneIndex;
            if (prev_ptr != nullptr) {
                prev_zone_idx = *prev_ptr;
            }
            uint8_t now_zone_idx = kNoZoneIndex;
            if (triggered) {
                now_zone_idx = find_zone_index(result.zone_id);
            }

            if (prev_zone_idx != now_zone_idx) {
                if (prev_zone_idx != kNoZoneIndex) {
                    append_event(out, track.track_id, prev_zone_idx,
                                 GeofenceEventType::EXITED, 0.0F);
                }
                if (now_zone_idx != kNoZoneIndex) {
                    const ZoneConfig& z = engine_.zones()[now_zone_idx];
                    if (z.alert_on_entry) {
                        append_event(out, track.track_id, now_zone_idx,
                                     GeofenceEventType::ENTERED,
                                     result.signed_distance_m);
                    }
                }
            } else {
                if (now_zone_idx != kNoZoneIndex) {
                    const uint8_t* pri_ptr = threat_priorities_.find(track.track_id);
                    uint8_t pri = 0U;
                    if (pri_ptr != nullptr) {
                        pri = *pri_ptr;
                    }
                    if (pri >= 2U) {
                        append_event(out, track.track_id, now_zone_idx,
                                     GeofenceEventType::INSIDE_THREAT,
                                     result.signed_distance_m);
                    }
                }
            }

            (void)zone_membership_.insert_or_assign(track.track_id, now_zone_idx);
        }

        pub_events_->publish(out);
    }

    GeofenceEngine engine_;
    FixedMap<uint32_t, uint8_t, TRACK_MAX_TRACKS> zone_membership_;
    FixedMap<uint32_t, uint8_t, TRACK_MAX_TRACKS> threat_priorities_;
    // ConstSharedPtr, not a deep copy: TrackArray copies allocate per tick
    // (A3.6; intent_classifier_node is the reference pattern).
    cuas_msgs::msg::TrackArray::ConstSharedPtr latest_tracks_;

    rclcpp::Publisher<cuas_msgs::msg::GeofenceEventArray>::SharedPtr pub_events_;
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
        auto node = std::make_shared<cuas::GeofenceNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FATAL: unhandled exception in GeofenceNode: %s\n", e.what());
        exit_code = 1;
    } catch (...) {
        std::fprintf(stderr, "FATAL: unhandled non-std exception in GeofenceNode\n");
        exit_code = 1;
    }
    return exit_code;
}
