// @file geofence_engine.cpp
// @brief Implements circle containment and ray-casting polygon containment.
#include "cuas_fusion/geofence_engine.hpp"

#include <cmath>

namespace cuas {

GeofenceEngine::GeofenceEngine()
: zones_()
{
}

void GeofenceEngine::load_zones(
    const FixedVector<ZoneConfig, GEOFENCE_MAX_ZONES>& configs)
{
    zones_.clear();
    const uint32_t count = configs.size();
    for (uint32_t i = 0U; i < count; ++i) {
        (void)zones_.push_back(configs[i]);
    }
}

uint32_t GeofenceEngine::zone_count() const
{
    return zones_.size();
}

const FixedVector<ZoneConfig, GEOFENCE_MAX_ZONES>& GeofenceEngine::zones() const
{
    return zones_;
}

bool GeofenceEngine::evaluate(
    float32_t x_m,
    float32_t y_m,
    uint32_t track_id,
    GeofenceResult& result) const
{
    // track_id is retained in the API so nodes can correlate results without
    // re-deriving it; the engine itself is stateless and does not use it.
    (void)track_id;

    for (uint32_t i = 0U; i < GEOFENCE_ZONE_ID_LEN; ++i) {
        result.zone_id[i] = '\0';
    }
    result.event_type = static_cast<uint8_t>(GeofenceEventType::INSIDE_THREAT);
    result.signed_distance_m = 0.0F;
    result.triggered = false;

    const uint32_t n_zones = zones_.size();
    for (uint32_t zi = 0U; zi < n_zones; ++zi) {
        const ZoneConfig& zone = zones_[zi];
        float32_t signed_dist = 0.0F;
        bool inside = false;

        switch (static_cast<ZoneShape>(zone.type)) {
            case ZoneShape::CIRCLE: {
                inside = point_in_circle(zone, x_m, y_m, signed_dist);
                break;
            }
            case ZoneShape::POLYGON: {
                inside = point_in_polygon(zone, x_m, y_m, signed_dist);
                break;
            }
            default: {
                inside = false;
                break;
            }
        }

        if (inside) {
            for (uint32_t k = 0U; k < GEOFENCE_ZONE_ID_LEN; ++k) {
                result.zone_id[k] = zone.id[k];
            }
            result.signed_distance_m = signed_dist;
            result.triggered = true;
            return true;
        }
    }

    return false;
}

bool GeofenceEngine::point_in_circle(
    const ZoneConfig& zone,
    float32_t x_m,
    float32_t y_m,
    float32_t& signed_distance_m) const
{
    const float32_t dx = x_m - zone.center_x_m;
    const float32_t dy = y_m - zone.center_y_m;
    const float32_t dist = std::sqrt((dx * dx) + (dy * dy));
    signed_distance_m = dist - zone.radius_m;
    return signed_distance_m <= 0.0F;
}

bool GeofenceEngine::point_in_polygon(
    const ZoneConfig& zone,
    float32_t x_m,
    float32_t y_m,
    float32_t& signed_distance_m) const
{
    const uint32_t n = zone.vertices_x.size();
    if (n < 3U) {
        signed_distance_m = 0.0F;
        return false;
    }

    bool inside = false;
    uint32_t j = n - 1U;
    for (uint32_t i = 0U; i < n; ++i) {
        const float32_t xi = zone.vertices_x[i];
        const float32_t yi = zone.vertices_y[i];
        const float32_t xj = zone.vertices_x[j];
        const float32_t yj = zone.vertices_y[j];

        const bool straddles_y = ((yi > y_m) != (yj > y_m));
        bool crosses = false;
        if (straddles_y) {
            const float32_t dy_edge = yj - yi;
            if (dy_edge != 0.0F) {
                const float32_t x_intersect =
                    (((xj - xi) * (y_m - yi)) / dy_edge) + xi;
                crosses = (x_m < x_intersect);
            }
        }
        if (crosses) {
            inside = !inside;
        }
        j = i;
    }

    const float32_t edge_dist = min_distance_to_polygon_edges(zone, x_m, y_m);
    if (inside) {
        signed_distance_m = -edge_dist;
    } else {
        signed_distance_m = edge_dist;
    }
    return inside;
}

float32_t GeofenceEngine::min_distance_to_polygon_edges(
    const ZoneConfig& zone,
    float32_t x_m,
    float32_t y_m) const
{
    const uint32_t n = zone.vertices_x.size();
    if (n < 2U) {
        return 0.0F;
    }

    float32_t min_dist = 0.0F;
    bool initialized = false;
    uint32_t j = n - 1U;
    for (uint32_t i = 0U; i < n; ++i) {
        const float32_t x1 = zone.vertices_x[j];
        const float32_t y1 = zone.vertices_y[j];
        const float32_t x2 = zone.vertices_x[i];
        const float32_t y2 = zone.vertices_y[i];
        const float32_t dx = x2 - x1;
        const float32_t dy = y2 - y1;
        const float32_t len_sq = (dx * dx) + (dy * dy);

        float32_t t = 0.0F;
        if (len_sq > 0.0F) {
            t = (((x_m - x1) * dx) + ((y_m - y1) * dy)) / len_sq;
            if (t < 0.0F) {
                t = 0.0F;
            }
            if (t > 1.0F) {
                t = 1.0F;
            }
        }
        const float32_t px = x1 + (t * dx);
        const float32_t py = y1 + (t * dy);
        const float32_t ex = x_m - px;
        const float32_t ey = y_m - py;
        const float32_t d = std::sqrt((ex * ex) + (ey * ey));
        if (!initialized || (d < min_dist)) {
            min_dist = d;
            initialized = true;
        }
        j = i;
    }
    return min_dist;
}

} // namespace cuas
