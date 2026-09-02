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

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>

namespace cuas {

// Config load happens only at node init; anything malformed is an
// unrecoverable startup error, logged with the zone index (RC-13). The old
// loader silently turned an unknown type into an r=0 circle and skipped
// bad vertices, so the fence enforced was not the fence configured.

static constexpr uint8_t kNoZoneIndex = 0xFFU;
// GeofenceEventArray is bounded (DEV-011); beyond this the rest of the
// tick's events are dropped with one throttled WARN.
static constexpr std::size_t kMaxEventsPerTick = 64U;

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
            RCLCPP_FATAL(get_logger(), "geofence_config_path is empty");
            return false;
        }
        YAML::Node root;
        try {
            root = YAML::LoadFile(path);
        } catch (const YAML::Exception& e) {
            RCLCPP_FATAL(get_logger(), "Geofence YAML '%s': %s", path.c_str(), e.what());
            return false;
        }
        if (!root["zones"] || !root["zones"].IsSequence()) {
            RCLCPP_FATAL(get_logger(), "Geofence YAML '%s': no 'zones' sequence", path.c_str());
            return false;
        }
        const YAML::Node zones_node = root["zones"];

        const uint32_t n_zones = static_cast<uint32_t>(zones_node.size());
        // Fail loud on over-capacity config (A2.6): silently dropping zones
        // or vertices would enforce a *different* boundary than configured —
        // a fence that quietly shrank is worse than a node that refuses to
        // start.
        if (n_zones > GEOFENCE_MAX_ZONES) {
            RCLCPP_FATAL(get_logger(),
                "Geofence config has %u zones; the engine supports %u — "
                "refusing to enforce a truncated boundary",
                n_zones, GEOFENCE_MAX_ZONES);
            return false;
        }
        for (uint32_t zi = 0U; zi < n_zones; ++zi) {
            const YAML::Node zn = zones_node[zi];
            ZoneConfig cfg;

            try {
                if (!zn["id"] || !zn["type"]) {
                    RCLCPP_FATAL(get_logger(), "Geofence zone %u: 'id' and 'type' are required", zi);
                    return false;
                }
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

                const std::string type_str = zn["type"].as<std::string>();
                if (type_str == "polygon") {
                    cfg.type = static_cast<uint8_t>(ZoneShape::POLYGON);
                } else if (type_str == "circle") {
                    cfg.type = static_cast<uint8_t>(ZoneShape::CIRCLE);
                } else {
                    RCLCPP_FATAL(get_logger(), "Geofence zone %u ('%s'): unknown type '%s'",
                                 zi, id_str.c_str(), type_str.c_str());
                    return false;
                }

                if (zn["alert_on_entry"]) {
                    cfg.alert_on_entry = zn["alert_on_entry"].as<bool>();
                }

                if (cfg.type == static_cast<uint8_t>(ZoneShape::CIRCLE)) {
                    if (!zn["radius_m"]) {
                        RCLCPP_FATAL(get_logger(), "Geofence zone %u ('%s'): circle needs radius_m",
                                     zi, id_str.c_str());
                        return false;
                    }
                    cfg.radius_m = zn["radius_m"].as<float32_t>();
                    if (zn["center_x_m"]) {
                        cfg.center_x_m = zn["center_x_m"].as<float32_t>();
                    }
                    if (zn["center_y_m"]) {
                        cfg.center_y_m = zn["center_y_m"].as<float32_t>();
                    }
                    if (!(cfg.radius_m > 0.0F) || !std::isfinite(cfg.radius_m) ||
                        !std::isfinite(cfg.center_x_m) || !std::isfinite(cfg.center_y_m)) {
                        RCLCPP_FATAL(get_logger(),
                            "Geofence zone %u ('%s'): circle radius/centre must be finite, radius > 0",
                            zi, id_str.c_str());
                        return false;
                    }
                } else {
                    if (!zn["vertices"] || !zn["vertices"].IsSequence()) {
                        RCLCPP_FATAL(get_logger(), "Geofence zone %u ('%s'): polygon needs a vertices list",
                                     zi, id_str.c_str());
                        return false;
                    }
                    const YAML::Node verts = zn["vertices"];
                    const uint32_t nv = static_cast<uint32_t>(verts.size());
                    if (nv > cfg.vertices_x.capacity()) {
                        RCLCPP_FATAL(get_logger(),
                            "Geofence zone %u has %u vertices; the engine "
                            "supports %u — refusing to enforce a truncated "
                            "polygon",
                            zi, nv, cfg.vertices_x.capacity());
                        return false;
                    }
                    for (uint32_t vi = 0U; vi < nv; ++vi) {
                        const YAML::Node v = verts[vi];
                        if (!v.IsSequence() || (v.size() != 2U)) {
                            RCLCPP_FATAL(get_logger(),
                                "Geofence zone %u ('%s'): vertex %u is not [x, y]",
                                zi, id_str.c_str(), vi);
                            return false;
                        }
                        const float32_t vx = v[0].as<float32_t>();
                        const float32_t vy = v[1].as<float32_t>();
                        if (!std::isfinite(vx) || !std::isfinite(vy)) {
                            RCLCPP_FATAL(get_logger(),
                                "Geofence zone %u ('%s'): vertex %u is not finite",
                                zi, id_str.c_str(), vi);
                            return false;
                        }
                        (void)cfg.vertices_x.push_back(vx);
                        (void)cfg.vertices_y.push_back(vy);
                    }
                    if (cfg.vertices_x.size() < 3U) {
                        RCLCPP_FATAL(get_logger(),
                            "Geofence zone %u ('%s'): polygon needs >= 3 vertices, has %u",
                            zi, id_str.c_str(), cfg.vertices_x.size());
                        return false;
                    }
                }
            } catch (const YAML::Exception& e) {
                RCLCPP_FATAL(get_logger(), "Geofence zone %u: %s", zi, e.what());
                return false;
            }

            if (!out.push_back(cfg)) {
                break;
            }
        }
        if (out.empty()) {
            RCLCPP_FATAL(get_logger(), "Geofence YAML '%s': zero zones", path.c_str());
            return false;
        }
        return true;
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

    static bool id_in_tracks(uint32_t id, const cuas_msgs::msg::TrackArray& tracks)
    {
        for (std::size_t k = 0U; k < tracks.tracks.size(); ++k) {
            if (tracks.tracks[k].track_id == id) {
                return true;
            }
        }
        return false;
    }

    void publish_tick()
    {
        cuas_msgs::msg::GeofenceEventArray out;
        out.stamp = this->now();

        if (latest_tracks_ == nullptr) {
            pub_events_->publish(out);
            return;
        }
        const cuas_msgs::msg::TrackArray& tracks = *latest_tracks_;

        // Per-track state follows the tracker's id set (RC-4): the 32-slot
        // maps used to fill with dead ids, after which new tracks got no
        // membership (ENTERED spam / never EXITED) and no priority.
        zone_membership_.erase_if(
            [&tracks](const uint32_t& id, const uint16_t&) -> bool {
                return !id_in_tracks(id, tracks);
            });
        threat_priorities_.erase_if(
            [&tracks](const uint32_t& id, const uint8_t&) -> bool {
                return !id_in_tracks(id, tracks);
            });

        const uint32_t n_tracks = static_cast<uint32_t>(tracks.tracks.size());
        for (uint32_t ti = 0U; ti < n_tracks; ++ti) {
            const cuas_msgs::msg::Track& track = tracks.tracks[ti];
            if (track.track_state_id != cuas::track_state::kConfirmed) {
                continue;
            }

            // Every containing zone is reported (RC-16): membership is a
            // bit per zone, and each zone gets its own ENTERED / EXITED /
            // INSIDE_THREAT, so a no-fly polygon inside a perimeter circle
            // is no longer masked by the circle.
            FixedVector<GeofenceResult, GEOFENCE_MAX_ZONES> hits;
            (void)engine_.evaluateAll(track.position_x_m, track.position_y_m, hits);
            uint16_t now_mask = 0U;
            std::array<float32_t, GEOFENCE_MAX_ZONES> dist{};
            for (uint32_t h = 0U; h < hits.size(); ++h) {
                const uint8_t zi = find_zone_index(hits[h].zone_id);
                if (zi != kNoZoneIndex) {
                    now_mask = static_cast<uint16_t>(now_mask | static_cast<uint16_t>(1U << zi));
                    dist[zi] = hits[h].signed_distance_m;
                }
            }

            const uint16_t* prev_ptr = zone_membership_.find(track.track_id);
            const uint16_t prev_mask = (prev_ptr != nullptr) ? *prev_ptr : static_cast<uint16_t>(0U);

            const uint8_t* pri_ptr = threat_priorities_.find(track.track_id);
            const uint8_t pri = (pri_ptr != nullptr) ? *pri_ptr : static_cast<uint8_t>(0U);

            const uint32_t n_zones = engine_.zone_count();
            bool dropped = false;
            uint16_t processed = 0U;
            for (uint32_t zi = 0U; zi < n_zones; ++zi) {
                const uint16_t bit = static_cast<uint16_t>(1U << zi);
                const bool was = (prev_mask & bit) != 0U;
                const bool now = (now_mask & bit) != 0U;
                if (out.events.size() >= kMaxEventsPerTick) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                        "Geofence events exceed the %zu-per-tick bound; dropping the rest",
                        kMaxEventsPerTick);
                    dropped = true;
                    break;
                }
                processed = static_cast<uint16_t>(processed | bit);
                if (was && !now) {
                    append_event(out, track.track_id, static_cast<uint8_t>(zi),
                                 GeofenceEventType::EXITED, 0.0F);
                } else if (!was && now) {
                    if (engine_.zones()[zi].alert_on_entry) {
                        append_event(out, track.track_id, static_cast<uint8_t>(zi),
                                     GeofenceEventType::ENTERED, dist[zi]);
                    }
                } else if (now && (pri >= 2U)) {
                    append_event(out, track.track_id, static_cast<uint8_t>(zi),
                                 GeofenceEventType::INSIDE_THREAT, dist[zi]);
                } else {
                    // intentionally empty: outside, unchanged
                }
            }

            // Zones handled this tick take their new membership; zones cut
            // off by the cap keep the old bit so their transition is
            // reported next tick, once (R6 F6, R6b-3).
            const uint16_t stored = dropped
                ? static_cast<uint16_t>((now_mask & processed) | (prev_mask & static_cast<uint16_t>(~processed)))
                : now_mask;
            (void)zone_membership_.insert_or_assign(track.track_id, stored);
        }

        pub_events_->publish(out);
    }

    GeofenceEngine engine_;
    FixedMap<uint32_t, uint16_t, TRACK_MAX_TRACKS> zone_membership_;
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
