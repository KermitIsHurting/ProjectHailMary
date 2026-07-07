// @file clutter_map_node.cpp
// @brief ROS 2 node wrapping ClutterMap to filter persistent static returns.
#include "cuas_fusion/clutter_map.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cuas_msgs/msg/clutter_status.hpp>

#include <chrono>
#include <functional>
#include <memory>

namespace cuas {

class ClutterMapNode : public rclcpp::Node
{
public:
    ClutterMapNode()
    : Node("clutter_map_node")
    , map_()
    , passthrough_during_learning_(true)
    {
        (void)declare_parameter<int32_t>("learn_frames", 200);
        (void)declare_parameter<float64_t>("threshold", 0.6);
        (void)declare_parameter<bool>("passthrough_during_learning", true);

        const int64_t   lf   = get_parameter("learn_frames").as_int();
        const float64_t th   = get_parameter("threshold").as_double();
        const bool      pass = get_parameter("passthrough_during_learning").as_bool();

        if (lf > 0) {
            map_.set_learn_frames(static_cast<uint32_t>(lf));
        }
        map_.set_threshold(static_cast<float32_t>(th));
        passthrough_during_learning_ = pass;

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/radar/detections", 10,
            std::bind(&ClutterMapNode::cloud_cb, this, std::placeholders::_1));

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/radar/filtered", 10);

        pub_status_ = create_publisher<cuas_msgs::msg::ClutterStatus>(
            "/clutter/status", 10);

        status_timer_ = create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&ClutterMapNode::publish_status, this));

        RCLCPP_INFO(get_logger(),
                    "Clutter map node ready (learn_frames=%ld threshold=%.2f)",
                    static_cast<long>(lf), th);
    }

private:
    void cloud_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
    {
        FixedVector<float32_t, kClutterMapMaxPoints> xs;
        FixedVector<float32_t, kClutterMapMaxPoints> ys;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
            if (!xs.push_back(*iter_x)) {
                break;
            }
            if (!ys.push_back(*iter_y)) {
                break;
            }
        }

        if (!map_.is_learned()) {
            map_.add_frame(xs, ys);
            if (passthrough_during_learning_) {
                pub_->publish(*msg);
            }
            return;
        }

        publish_filtered(*msg);
    }

    void publish_filtered(const sensor_msgs::msg::PointCloud2 & in)
    {
        sensor_msgs::PointCloud2ConstIterator<float> it_x(in, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(in, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(in, "z");
        sensor_msgs::PointCloud2ConstIterator<float> it_v(in, "velocity");

        FixedVector<float32_t, kClutterMapMaxPoints> kx;
        FixedVector<float32_t, kClutterMapMaxPoints> ky;
        FixedVector<float32_t, kClutterMapMaxPoints> kz;
        FixedVector<float32_t, kClutterMapMaxPoints> kv;

        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_v) {
            const float32_t x = *it_x;
            const float32_t y = *it_y;
            if (map_.is_clutter(x, y)) {
                continue;
            }
            if (!kx.push_back(x))   { break; }
            if (!ky.push_back(y))   { break; }
            if (!kz.push_back(*it_z)) { break; }
            if (!kv.push_back(*it_v)) { break; }
        }

        sensor_msgs::msg::PointCloud2 out;
        out.header       = in.header;
        out.height       = 1U;
        out.width        = kx.size();
        out.is_dense     = false;
        out.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier mod(out);
        mod.setPointCloud2Fields(
            4,
            "x",        1, sensor_msgs::msg::PointField::FLOAT32,
            "y",        1, sensor_msgs::msg::PointField::FLOAT32,
            "z",        1, sensor_msgs::msg::PointField::FLOAT32,
            "velocity", 1, sensor_msgs::msg::PointField::FLOAT32);
        mod.resize(kx.size());

        sensor_msgs::PointCloud2Iterator<float> ox(out, "x");
        sensor_msgs::PointCloud2Iterator<float> oy(out, "y");
        sensor_msgs::PointCloud2Iterator<float> oz(out, "z");
        sensor_msgs::PointCloud2Iterator<float> ov(out, "velocity");

        for (uint32_t i = 0U; i < kx.size(); ++i) {
            *ox = kx[i]; ++ox;
            *oy = ky[i]; ++oy;
            *oz = kz[i]; ++oz;
            *ov = kv[i]; ++ov;
        }

        pub_->publish(out);
    }

    void publish_status()
    {
        cuas_msgs::msg::ClutterStatus msg;
        uint8_t learned_state = 0U;
        if (map_.is_learned()) {
            learned_state = 1U;
        }
        msg.state           = learned_state;
        msg.frames_learned  = map_.frame_count();
        msg.frames_required = map_.learn_frames();
        msg.occupancy_ratio = map_.occupancy_ratio();
        msg.stamp           = clock_.now();
        pub_status_->publish(msg);
    }

    ClutterMap    map_;
    bool          passthrough_during_learning_;
    rclcpp::Clock clock_{RCL_STEADY_TIME};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
    // WHY: typed fields replace runtime string formatting,
    // eliminating heap allocation and giving downstream consumers
    // directly usable values without parsing.
    rclcpp::Publisher<cuas_msgs::msg::ClutterStatus>::SharedPtr    pub_status_;
    rclcpp::TimerBase::SharedPtr                                   status_timer_;
};

} // namespace cuas

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cuas::ClutterMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
