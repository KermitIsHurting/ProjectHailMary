// @file sim_radar_node.cpp
// @brief ROS 2 node publishing simulated radar detections as PointCloud2.
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/param_utils.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/drivers/sim_radar.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <ctime>
#include <string>
#include <cstdio>

namespace cuas {

namespace {

// Same time base as the hardware radar_parser_node (CLOCK_MONOTONIC):
// the sim must be indistinguishable from the real sensor downstream,
// and the pipeline measurement path runs on one monotonic clock (P2.1).
builtin_interfaces::msg::Time monotonic_stamp()
{
    struct timespec ts{};
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    builtin_interfaces::msg::Time t;
    t.sec     = static_cast<int32_t>(ts.tv_sec);
    t.nanosec = static_cast<uint32_t>(ts.tv_nsec);
    return t;
}

} // namespace

class SimRadarNode : public rclcpp::Node
{
public:
    SimRadarNode()
    : Node("sim_radar_node")
    , sim_radar_(42U)
    , is_circle_(false)
    , publish_rate_hz_(20.0F)
    {
        (void)declare_parameter<float64_t>("publish_rate_hz", 20.0);
        (void)declare_parameter<std::string>("scenario", "approach");
        (void)declare_parameter<int64_t>("noise_seed", 42);

        float64_t rate = get_parameter("publish_rate_hz").as_double();
        rate = clamp_rate_hz(get_logger(), "publish_rate_hz", rate, 20.0);
        const std::string scenario = get_parameter("scenario").as_string();
        const int64_t seed = get_parameter("noise_seed").as_int();

        publish_rate_hz_ = static_cast<float32_t>(rate);
        sim_radar_ = SimRadar(static_cast<uint32_t>(seed));

        load_scenario(scenario);

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/radar/detections", 10);

        const int32_t period_ms = static_cast<int32_t>(1000.0 / rate);
        // Integrate with the timer's actual period, not 1/rate: the ms
        // truncation made simulated speed 1-4 % high at 30/60 Hz (RC-37).
        dt_s_ = static_cast<float32_t>(period_ms) / 1000.0F;
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&SimRadarNode::tick, this));

        RCLCPP_INFO(get_logger(),
                    "Sim radar node ready (rate=%.1fHz scenario=%s seed=%ld)",
                    rate, scenario.c_str(), seed);
    }

private:
    static ScenarioTarget make_target(uint32_t id, float32_t x, float32_t y, float32_t z,
                                      float32_t vx, float32_t vy, float32_t vz)
    {
        ScenarioTarget t;
        t.x_m = x;      t.y_m = y;      t.z_m = z;
        t.vx_mps = vx;  t.vy_mps = vy;  t.vz_mps = vz;
        t.rcs_dbsm  = 10.0F;
        t.target_id = id;
        return t;
    }

    // Scenes are in radar_frame per the ICD: X = azimuth (right), Y = range
    // (forward), Z = up. The original approach/two_targets scenes moved
    // along X, i.e. across the beam at 90 deg, which the camera never sees
    // and fusion never projects (RC-38). Names map to the A6 shape list.
    void load_scenario(const std::string& name)
    {
        std::string scenario = name;
        if ((scenario != "approach") && (scenario != "lateral") &&
            (scenario != "circle") && (scenario != "two_targets") &&
            (scenario != "crossing") && (scenario != "clutter") &&
            (scenario != "max_range") && (scenario != "hover"))
        {
            RCLCPP_WARN(get_logger(),
                        "Unknown scenario '%s', defaulting to approach",
                        name.c_str());
            scenario = "approach";
        }

        if (scenario == "approach") {
            // S-1: closes along boresight at 0.5 m/s from 14 m: passes the
            // r=5 fence at 18 s, the 4 m threat range at 20 s, the sensor
            // at 28 s, and leaves the 15 m clip at 58 s.
            sim_radar_.addTarget(make_target(1U, 0.0F, 14.0F, 1.5F, 0.0F, -0.5F, 0.0F));
        } else if (scenario == "lateral") {
            // S-5: crosses the field at 4 m range at 0.5 m/s: enters the
            // no-fly polygon (x in [-4, -1]) at 16 s, leaves it at 22 s,
            // leaves the 15 m clip at 53 s.
            sim_radar_.addTarget(make_target(1U, -12.0F, 4.0F, 1.5F, 0.5F, 0.0F, 0.0F));
        } else if (scenario == "circle") {
            // S-11 soak: orbits the sensor at 6 m, never leaves range.
            sim_radar_.addTarget(make_target(1U, 0.0F, 6.0F, 1.5F, 1.0F, 0.0F, 0.0F));
            is_circle_ = true;
        } else if (scenario == "two_targets") {
            // Parallel approach, 2 m apart, from 14 m at 0.4 m/s (35 s to
            // the sensor).
            sim_radar_.addTarget(make_target(1U,  1.0F, 14.0F, 1.5F, 0.0F, -0.4F, 0.0F));
            sim_radar_.addTarget(make_target(2U, -1.0F, 14.0F, 1.5F, 0.0F, -0.4F, 0.0F));
        } else if (scenario == "crossing") {
            // S-2: paths cross at (0, 6) after 24 s at 0.5 m/s each — does
            // association swap? Both start inside the 15 m clip (13.4 m).
            sim_radar_.addTarget(make_target(1U, -12.0F, 6.0F, 1.5F,  0.5F, 0.0F, 0.0F));
            sim_radar_.addTarget(make_target(2U,  12.0F, 6.0F, 1.5F, -0.5F, 0.0F, 0.0F));
        } else if (scenario == "clutter") {
            // S-3: three static reflectors INSIDE the clutter map (x, y in
            // [-5, 5), 0.25 m cells), at cell centres. With 0.1 m return
            // noise a centred reflector hits its cell in ~62 % of frames,
            // just above the 60 % learning threshold: the run measures how
            // many returns leak past the learned map (audit D-4 / RC-27).
            sim_radar_.addTarget(make_target(1U,  2.125F, 3.125F, 0.5F, 0.0F, 0.0F, 0.0F));
            sim_radar_.addTarget(make_target(2U, -2.875F, 4.125F, 0.5F, 0.0F, 0.0F, 0.0F));
            sim_radar_.addTarget(make_target(3U,  0.125F, 2.375F, 0.5F, 0.0F, 0.0F, 0.0F));
        } else if (scenario == "max_range") {
            // S-6: first return at the 15 m clip, closing at 0.05 m/s so the
            // readout window still sees it beyond 14 m.
            sim_radar_.addTarget(make_target(1U, 0.0F, 14.9F, 1.5F, 0.0F, -0.05F, 0.0F));
        } else {
            // hover: a stationary target at 6 m (RC-27: the one-shot clutter
            // map learns it as clutter — Needs John's relearn policy).
            sim_radar_.addTarget(make_target(1U, 0.0F, 6.0F, 1.5F, 0.0F, 0.0F, 0.0F));
        }
    }

    void tick()
    {
        const float32_t dt = dt_s_;

        if (is_circle_) {
            ScenarioTarget& t = sim_radar_.getTarget(0U);
            const float32_t v_sq = t.vx_mps * t.vx_mps + t.vy_mps * t.vy_mps;
            const float32_t r_sq = t.x_m * t.x_m + t.y_m * t.y_m;
            if (r_sq > 0.01F) {
                // WHY: centripetal acceleration maintains circular orbit at current radius
                const float32_t inv_r_sq = 1.0F / r_sq;
                t.vx_mps += -v_sq * t.x_m * inv_r_sq * dt;
                t.vy_mps += -v_sq * t.y_m * inv_r_sq * dt;
            }
        }

        sim_radar_.step(dt);

        // Publish every frame, even empty (RC-12): an empty scene must stay
        // distinguishable from a dead radar on the bus.
        publish_cloud(sim_radar_.getPoints());
    }

    void publish_cloud(const FixedVector<SimRadarPoint, kSimRadarMaxPoints>& pts)
    {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp    = monotonic_stamp();
        msg.header.frame_id = "radar_frame";
        msg.height          = 1U;
        msg.width           = pts.size();
        msg.is_dense        = false;
        msg.is_bigendian    = false;

        sensor_msgs::PointCloud2Modifier mod(msg);
        mod.setPointCloud2Fields(
            4,
            "x",        1, sensor_msgs::msg::PointField::FLOAT32,
            "y",        1, sensor_msgs::msg::PointField::FLOAT32,
            "z",        1, sensor_msgs::msg::PointField::FLOAT32,
            "velocity", 1, sensor_msgs::msg::PointField::FLOAT32);
        mod.resize(pts.size());

        sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");
        sensor_msgs::PointCloud2Iterator<float> it_v(msg, "velocity");

        for (uint32_t i = 0U; i < pts.size(); ++i) {
            *it_x = pts[i].x_m;         ++it_x;
            *it_y = pts[i].y_m;         ++it_y;
            *it_z = pts[i].z_m;         ++it_z;
            *it_v = pts[i].doppler_mps; ++it_v;
        }

        pub_->publish(msg);
    }

    SimRadar    sim_radar_;
    bool        is_circle_;
    float32_t   publish_rate_hz_;
    float32_t   dt_s_ = 0.05F;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr                                timer_;
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
        auto node = std::make_shared<cuas::SimRadarNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FATAL: unhandled exception in SimRadarNode: %s\n", e.what());
        exit_code = 1;
    } catch (...) {
        std::fprintf(stderr, "FATAL: unhandled non-std exception in SimRadarNode\n");
        exit_code = 1;
    }
    return exit_code;
}
