// @file sim_radar_node.cpp
// @brief ROS 2 node publishing simulated radar detections as /radar/detections.
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/radar_sim.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <chrono>
#include <ctime>
#include <functional>
#include <memory>

namespace cuas {

class SimRadarNode : public rclcpp::Node
{
public:
    SimRadarNode()
    : Node("sim_radar_node")
    , sim_()
    , last_tick_sec_(0.0F)
    , clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME))
    {
        (void)declare_parameter<float64_t>("publish_rate_hz", 16.0);
        (void)declare_parameter<float64_t>("noise_sigma_m",   0.05);
        (void)declare_parameter<float64_t>("target_0_x",      3.0);
        (void)declare_parameter<float64_t>("target_0_y",      0.0);
        (void)declare_parameter<float64_t>("target_0_vx",    -0.5);
        (void)declare_parameter<float64_t>("target_0_vy",     0.0);
        (void)declare_parameter<float64_t>("target_1_x",     -2.0);
        (void)declare_parameter<float64_t>("target_1_y",      1.0);
        (void)declare_parameter<float64_t>("target_1_vx",     0.3);
        (void)declare_parameter<float64_t>("target_1_vy",     0.2);

        const float64_t rate  = get_parameter("publish_rate_hz").as_double();
        const float64_t sigma = get_parameter("noise_sigma_m").as_double();

        sim_.init(static_cast<float32_t>(sigma));

        SimTarget t0;
        t0.x_m    = static_cast<float32_t>(get_parameter("target_0_x").as_double());
        t0.y_m    = static_cast<float32_t>(get_parameter("target_0_y").as_double());
        t0.vx_mps = static_cast<float32_t>(get_parameter("target_0_vx").as_double());
        t0.vy_mps = static_cast<float32_t>(get_parameter("target_0_vy").as_double());
        t0.active = true;
        sim_.set_target(0U, t0);

        SimTarget t1;
        t1.x_m    = static_cast<float32_t>(get_parameter("target_1_x").as_double());
        t1.y_m    = static_cast<float32_t>(get_parameter("target_1_y").as_double());
        t1.vx_mps = static_cast<float32_t>(get_parameter("target_1_vx").as_double());
        t1.vy_mps = static_cast<float32_t>(get_parameter("target_1_vy").as_double());
        t1.active = true;
        sim_.set_target(1U, t1);

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/radar/detections", 10);

        const float64_t period_ms_d = 1000.0 / rate;
        const int32_t   period_ms   = static_cast<int32_t>(period_ms_d);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&SimRadarNode::tick, this));

        last_tick_sec_ = static_cast<float32_t>(clock_->now().seconds());

        RCLCPP_INFO(get_logger(),
                    "Sim radar node ready (rate=%.1fHz sigma=%.3fm)", rate, sigma);
    }

private:
    static builtin_interfaces::msg::Time monotonic_stamp()
    {
        struct timespec ts{};
        (void)clock_gettime(CLOCK_MONOTONIC, &ts);
        builtin_interfaces::msg::Time t;
        t.sec     = static_cast<int32_t>(ts.tv_sec);
        t.nanosec = static_cast<uint32_t>(ts.tv_nsec);
        return t;
    }

    void tick()
    {
        const float32_t now = static_cast<float32_t>(clock_->now().seconds());
        float32_t       dt  = now - last_tick_sec_;
        if (dt < 0.0F) {
            dt = 0.0F;
        }
        last_tick_sec_ = now;

        sim_.step(dt);

        FixedVector<SimPoint, kRadarSimMaxPoints> pts;
        const uint32_t n = sim_.generate(dt, pts);

        if (n == 0U) {
            return;
        }

        publish_cloud(pts);
    }

    void publish_cloud(const FixedVector<SimPoint, kRadarSimMaxPoints> & pts)
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

    RadarSim                       sim_;
    float32_t                      last_tick_sec_;
    std::shared_ptr<rclcpp::Clock> clock_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr                                timer_;
};

} // namespace cuas

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cuas::SimRadarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
