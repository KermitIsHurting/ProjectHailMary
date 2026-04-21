// @file sim_radar_node.cpp
// @brief ROS 2 node publishing simulated radar detections as PointCloud2.
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/drivers/sim_radar.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <string>

namespace cuas {

class SimRadarNode : public rclcpp::Node
{
public:
    SimRadarNode()
    : Node("sim_radar_node")
    , sim_radar_(42U)
    , is_circle_(false)
    , publish_rate_hz_(16.0F)
    {
        (void)declare_parameter<float64_t>("publish_rate_hz", 16.0);
        (void)declare_parameter<std::string>("scenario", "approach");
        (void)declare_parameter<int64_t>("noise_seed", 42);

        const float64_t rate = get_parameter("publish_rate_hz").as_double();
        const std::string scenario = get_parameter("scenario").as_string();
        const int64_t seed = get_parameter("noise_seed").as_int();

        publish_rate_hz_ = static_cast<float32_t>(rate);
        sim_radar_ = SimRadar(static_cast<uint32_t>(seed));

        load_scenario(scenario);

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/radar/detections", 10);

        const int32_t period_ms = static_cast<int32_t>(1000.0 / rate);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&SimRadarNode::tick, this));

        RCLCPP_INFO(get_logger(),
                    "Sim radar node ready (rate=%.1fHz scenario=%s seed=%ld)",
                    rate, scenario.c_str(), seed);
    }

private:
    void load_scenario(const std::string& name)
    {
        if (name == "approach") {
            ScenarioTarget t;
            t.x_m      = 8.0F;
            t.y_m      = 0.0F;
            t.z_m      = 1.5F;
            t.vx_mps   = -1.0F;
            t.vy_mps   = 0.0F;
            t.vz_mps   = 0.0F;
            t.rcs_dbsm = 10.0F;
            t.target_id = 1U;
            sim_radar_.addTarget(t);
        } else if (name == "lateral") {
            ScenarioTarget t;
            t.x_m      = -5.0F;
            t.y_m      = 4.0F;
            t.z_m      = 1.5F;
            t.vx_mps   = 1.0F;
            t.vy_mps   = 0.0F;
            t.vz_mps   = 0.0F;
            t.rcs_dbsm = 10.0F;
            t.target_id = 1U;
            sim_radar_.addTarget(t);
        } else if (name == "circle") {
            ScenarioTarget t;
            t.x_m      = 3.0F;
            t.y_m      = 0.0F;
            t.z_m      = 1.5F;
            t.vx_mps   = 0.0F;
            t.vy_mps   = 1.0F;
            t.vz_mps   = 0.0F;
            t.rcs_dbsm = 10.0F;
            t.target_id = 1U;
            sim_radar_.addTarget(t);
            is_circle_ = true;
        } else if (name == "two_targets") {
            ScenarioTarget t1;
            t1.x_m      = 6.0F;
            t1.y_m      = 1.0F;
            t1.z_m      = 1.5F;
            t1.vx_mps   = -0.8F;
            t1.vy_mps   = 0.0F;
            t1.vz_mps   = 0.0F;
            t1.rcs_dbsm = 10.0F;
            t1.target_id = 1U;
            sim_radar_.addTarget(t1);

            ScenarioTarget t2;
            t2.x_m      = 6.0F;
            t2.y_m      = -1.0F;
            t2.z_m      = 1.5F;
            t2.vx_mps   = -0.8F;
            t2.vy_mps   = 0.0F;
            t2.vz_mps   = 0.0F;
            t2.rcs_dbsm = 10.0F;
            t2.target_id = 2U;
            sim_radar_.addTarget(t2);
        } else {
            RCLCPP_WARN(get_logger(), "Unknown scenario '%s', defaulting to approach",
                        name.c_str());
            load_scenario("approach");
        }
    }

    void tick()
    {
        const float32_t dt = 1.0F / publish_rate_hz_;

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

        const auto pts = sim_radar_.getPoints();
        if (pts.empty()) {
            return;
        }

        publish_cloud(pts);
    }

    void publish_cloud(const FixedVector<SimRadarPoint, kSimRadarMaxPoints>& pts)
    {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp    = this->now();
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

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr                                timer_;
};

} // namespace cuas

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cuas::SimRadarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
