// @file geofence_engine.hpp
// @brief Pure-math geofence zone evaluator with fixed-capacity storage.
#pragma once

#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

namespace cuas {

static constexpr uint32_t GEOFENCE_ZONE_ID_LEN       = 32U;
static constexpr uint32_t GEOFENCE_MAX_POLYGON_VERTS = 32U;
static constexpr uint32_t GEOFENCE_MAX_ZONES         = 16U;
// GeofenceEngine::membershipMask packs one bit per zone into a uint16_t.
static_assert(GEOFENCE_MAX_ZONES <= 16U, "membership mask is 16 bits");

enum class ZoneShape : uint8_t {
    CIRCLE  = 0U,
    POLYGON = 1U
};

enum class GeofenceEventType : uint8_t {
    ENTERED       = 0U,
    EXITED        = 1U,
    INSIDE_THREAT = 2U
};

struct ZoneConfig {
    char      id[GEOFENCE_ZONE_ID_LEN];
    uint8_t   type;
    float32_t center_x_m;
    float32_t center_y_m;
    float32_t radius_m;
    FixedVector<float32_t, GEOFENCE_MAX_POLYGON_VERTS> vertices_x;
    FixedVector<float32_t, GEOFENCE_MAX_POLYGON_VERTS> vertices_y;
    bool      alert_on_entry;

    ZoneConfig()
    : id{}
    , type(static_cast<uint8_t>(ZoneShape::CIRCLE))
    , center_x_m(0.0F)
    , center_y_m(0.0F)
    , radius_m(0.0F)
    , vertices_x()
    , vertices_y()
    , alert_on_entry(false)
    {
    }
};

struct GeofenceResult {
    char      zone_id[GEOFENCE_ZONE_ID_LEN];
    uint8_t   event_type;
    float32_t signed_distance_m;
    bool      triggered;

    GeofenceResult()
    : zone_id{}
    , event_type(static_cast<uint8_t>(GeofenceEventType::INSIDE_THREAT))
    , signed_distance_m(0.0F)
    , triggered(false)
    {
    }
};

class GeofenceEngine {
public:
    GeofenceEngine();

    void load_zones(const FixedVector<ZoneConfig, GEOFENCE_MAX_ZONES>& configs);

    // First containing zone in config order (kept for callers that want
    // one answer); evaluateAll reports EVERY containing zone (RC-16): a
    // no-fly polygon inside a perimeter circle was masked by the circle
    // because only the first hit was ever returned.
    bool evaluate(float32_t x_m,
                  float32_t y_m,
                  uint32_t track_id,
                  GeofenceResult& result) const;

    uint32_t evaluateAll(float32_t x_m,
                         float32_t y_m,
                         FixedVector<GeofenceResult, GEOFENCE_MAX_ZONES>& results) const;

    // Bit i set = inside zone i (config order); GEOFENCE_MAX_ZONES <= 16.
    uint16_t membershipMask(float32_t x_m, float32_t y_m) const;

    uint32_t zone_count() const;

    const FixedVector<ZoneConfig, GEOFENCE_MAX_ZONES>& zones() const;

private:
    bool point_in_circle(const ZoneConfig& zone,
                         float32_t x_m,
                         float32_t y_m,
                         float32_t& signed_distance_m) const;

    bool point_in_polygon(const ZoneConfig& zone,
                          float32_t x_m,
                          float32_t y_m,
                          float32_t& signed_distance_m) const;

    float32_t min_distance_to_polygon_edges(const ZoneConfig& zone,
                                            float32_t x_m,
                                            float32_t y_m) const;

    FixedVector<ZoneConfig, GEOFENCE_MAX_ZONES> zones_;
};

} // namespace cuas
