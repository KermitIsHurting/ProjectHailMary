// @file inference_node.cpp
// @brief ROS 2 node wrapping TrtDetector for camera image input.
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/ros_image_adapter.hpp"
#include "cuas_fusion/inference/trt_detector.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <cstdlib>
#include <string>
#include <vector>

namespace cuas {

class InferenceNode : public rclcpp::Node
{
public:
    InferenceNode()
    : Node("inference_node")
    {
        const char* home = std::getenv("HOME");
        const std::string default_path =
            std::string(home != nullptr ? home : "/root")
            + "/ProjectHailMarry/models/yolov8s_int8.engine";
        declare_parameter<std::string>("engine_path", default_path);

        const std::string engine_path = get_parameter("engine_path").as_string();

        if (!detector_.init(engine_path)) {
            RCLCPP_FATAL(get_logger(), "TrtDetector init failed: %s", engine_path.c_str());
            rclcpp::shutdown();
            return;
        }

        pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
            "/inference/detections", 5);

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 1,
            std::bind(&InferenceNode::imageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Inference node ready — engine: %s", engine_path.c_str());
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        cv::Mat frame;
        if (!rosImageToBgr(*msg, frame)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Unsupported image encoding '%s' or empty frame",
                msg->encoding.c_str());
            return;
        }

        std::vector<BoundingBox> detections;
        if (!detector_.infer(frame, detections)) {
            return;
        }

        vision_msgs::msg::Detection2DArray out;
        out.header = msg->header;

        for (std::size_t i = 0U; i < detections.size(); ++i) {
            const BoundingBox& bb = detections[i];
            vision_msgs::msg::Detection2D det;
            det.header = msg->header;

            det.bbox.center.position.x = static_cast<float64_t>(bb.x + bb.w * 0.5F);
            det.bbox.center.position.y = static_cast<float64_t>(bb.y + bb.h * 0.5F);
            det.bbox.size_x = static_cast<float64_t>(bb.w);
            det.bbox.size_y = static_cast<float64_t>(bb.h);

            vision_msgs::msg::ObjectHypothesisWithPose hyp;
            hyp.hypothesis.class_id = std::to_string(bb.class_id);
            hyp.hypothesis.score    = static_cast<float64_t>(bb.confidence);
            det.results.push_back(hyp);

            out.detections.push_back(det);
        }

        pub_->publish(out);
    }

    TrtDetector detector_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_;
};

} // namespace cuas

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cuas::InferenceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
