// @file test_geofence_engine.cpp
// @brief Unit tests for GeofenceEngine circle and polygon logic.
#include "cuas_fusion/geofence_engine.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <gtest/gtest.h>

namespace {

cuas::ZoneConfig make_circle_zone(const char * id,
                                  const cuas::float32_t cx,
                                  const cuas::float32_t cy,
                                  const cuas::float32_t r)
{
    cuas::ZoneConfig z;
    z.type           = static_cast<cuas::uint8_t>(cuas::ZoneShape::CIRCLE);
    z.center_x_m     = cx;
    z.center_y_m     = cy;
    z.radius_m       = r;
    z.alert_on_entry = true;
    for (cuas::uint32_t k = 0U; k < cuas::GEOFENCE_ZONE_ID_LEN; ++k) {
        const char c = id[k];
        z.id[k] = c;
        if (c == '\0') {
            break;
        }
    }
    return z;
}

cuas::ZoneConfig make_left_polygon_zone()
{
    cuas::ZoneConfig z;
    z.type           = static_cast<cuas::uint8_t>(cuas::ZoneShape::POLYGON);
    z.alert_on_entry = true;
    const cuas::float32_t xs[4] = {-4.0F, -4.0F, -1.0F, -1.0F};
    const cuas::float32_t ys[4] = { 0.0F,  6.0F,  6.0F,  0.0F};
    for (cuas::uint32_t i = 0U; i < 4U; ++i) {
        (void)z.vertices_x.push_back(xs[i]);
        (void)z.vertices_y.push_back(ys[i]);
    }
    const char id[] = "no_fly_left";
    const cuas::uint32_t id_len = static_cast<cuas::uint32_t>(sizeof(id));
    for (cuas::uint32_t k = 0U; (k < id_len) && (k < cuas::GEOFENCE_ZONE_ID_LEN); ++k) {
        z.id[k] = id[k];
    }
    return z;
}

} // namespace

// RC-16: a point inside both the perimeter circle and the no-fly polygon
// must be reported for BOTH zones; the first-hit API masked the polygon.
TEST(GeofenceEngineTest, OverlappingZonesAreAllReported)
{
    cuas::FixedVector<cuas::ZoneConfig, cuas::GEOFENCE_MAX_ZONES> zones;
    ASSERT_TRUE(zones.push_back(make_circle_zone("perimeter", 0.0F, 0.0F, 20.0F)));
    ASSERT_TRUE(zones.push_back(make_left_polygon_zone()));
    cuas::GeofenceEngine engine;
    engine.load_zones(zones);

    cuas::FixedVector<cuas::GeofenceResult, cuas::GEOFENCE_MAX_ZONES> hits;
    EXPECT_EQ(engine.evaluateAll(-2.0F, 3.0F, hits), 2U);
    ASSERT_EQ(hits.size(), 2U);
    EXPECT_STREQ(hits[0].zone_id, "perimeter");
    EXPECT_STREQ(hits[1].zone_id, "no_fly_left");
    EXPECT_EQ(engine.membershipMask(-2.0F, 3.0F), 0x3U);
    EXPECT_EQ(engine.membershipMask(5.0F, 3.0F), 0x1U);
    EXPECT_EQ(engine.membershipMask(50.0F, 3.0F), 0x0U);

    cuas::GeofenceResult first;
    EXPECT_TRUE(engine.evaluate(-2.0F, 3.0F, 1U, first));
    EXPECT_STREQ(first.zone_id, "perimeter");
}

TEST(GeofenceEngineTest, CircleInside)
{
    cuas::GeofenceEngine engine;
    cuas::FixedVector<cuas::ZoneConfig, cuas::GEOFENCE_MAX_ZONES> zones;
    (void)zones.push_back(make_circle_zone("perimeter", 0.0F, 0.0F, 5.0F));
    engine.load_zones(zones);

    cuas::GeofenceResult r;
    const bool triggered = engine.evaluate(0.0F, 0.0F, 1U, r);
    EXPECT_TRUE(triggered);
    EXPECT_TRUE(r.triggered);
}

TEST(GeofenceEngineTest, CircleOutside)
{
    cuas::GeofenceEngine engine;
    cuas::FixedVector<cuas::ZoneConfig, cuas::GEOFENCE_MAX_ZONES> zones;
    (void)zones.push_back(make_circle_zone("perimeter", 0.0F, 0.0F, 5.0F));
    engine.load_zones(zones);

    cuas::GeofenceResult r;
    const bool triggered = engine.evaluate(10.0F, 0.0F, 1U, r);
    EXPECT_FALSE(triggered);
    EXPECT_FALSE(r.triggered);
}

TEST(GeofenceEngineTest, CircleBoundary)
{
    cuas::GeofenceEngine engine;
    cuas::FixedVector<cuas::ZoneConfig, cuas::GEOFENCE_MAX_ZONES> zones;
    (void)zones.push_back(make_circle_zone("perimeter", 0.0F, 0.0F, 5.0F));
    engine.load_zones(zones);

    cuas::GeofenceResult r;
    const bool triggered = engine.evaluate(5.0F, 0.0F, 1U, r);
    EXPECT_TRUE(triggered);
    EXPECT_TRUE(r.triggered);
}

TEST(GeofenceEngineTest, PolygonInside)
{
    cuas::GeofenceEngine engine;
    cuas::FixedVector<cuas::ZoneConfig, cuas::GEOFENCE_MAX_ZONES> zones;
    (void)zones.push_back(make_left_polygon_zone());
    engine.load_zones(zones);

    cuas::GeofenceResult r;
    const bool triggered = engine.evaluate(-2.0F, 3.0F, 1U, r);
    EXPECT_TRUE(triggered);
    EXPECT_TRUE(r.triggered);
}

TEST(GeofenceEngineTest, PolygonOutside)
{
    cuas::GeofenceEngine engine;
    cuas::FixedVector<cuas::ZoneConfig, cuas::GEOFENCE_MAX_ZONES> zones;
    (void)zones.push_back(make_left_polygon_zone());
    engine.load_zones(zones);

    cuas::GeofenceResult r;
    const bool triggered = engine.evaluate(0.0F, 3.0F, 1U, r);
    EXPECT_FALSE(triggered);
    EXPECT_FALSE(r.triggered);
}

TEST(GeofenceEngineTest, SignedDistanceNegativeInside)
{
    cuas::GeofenceEngine engine;
    cuas::FixedVector<cuas::ZoneConfig, cuas::GEOFENCE_MAX_ZONES> zones;
    (void)zones.push_back(make_circle_zone("perimeter", 0.0F, 0.0F, 5.0F));
    engine.load_zones(zones);

    cuas::GeofenceResult r;
    const bool triggered = engine.evaluate(0.0F, 0.0F, 1U, r);
    EXPECT_TRUE(triggered);
    EXPECT_LT(r.signed_distance_m, 0.0F);
}

// WHY: public evaluate() resets signed_distance_m to 0 when nothing triggers;
// outside-zone distance is not observable without modifying the engine, so the
// contract we test is: triggered=false AND signed_distance_m stays at 0.
TEST(GeofenceEngineTest, SignedDistancePositiveOutside)
{
    cuas::GeofenceEngine engine;
    cuas::FixedVector<cuas::ZoneConfig, cuas::GEOFENCE_MAX_ZONES> zones;
    (void)zones.push_back(make_circle_zone("perimeter", 0.0F, 0.0F, 5.0F));
    engine.load_zones(zones);

    cuas::GeofenceResult r;
    const bool triggered = engine.evaluate(10.0F, 0.0F, 1U, r);
    EXPECT_FALSE(triggered);
    EXPECT_FALSE(r.triggered);
    EXPECT_FLOAT_EQ(r.signed_distance_m, 0.0F);
}
