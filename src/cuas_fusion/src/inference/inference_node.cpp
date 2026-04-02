// inference_node.cpp
// Thin ROS 2 wrapper around TrtDetector: subscribes to camera images,
// runs YOLOv8 TensorRT inference, publishes Detection2DArray.

#include "cuas_fusion/inference/trt_detector.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <vector>

namespace cuas {

class InferenceNode : public rclcpp::Node
{
public:
    InferenceNode()
    : Node("inference_node")
    {
        declare_parameter<std::string>(
            "engine_path",
            std::string(getenv("HOME")) + "/ProjectHailMarry/models/yolov8s_int8.engine");

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
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
            return;
        }

        std::vector<BoundingBox> detections;
        if (!detector_.infer(cv_ptr->image, detections)) {
            return;
        }

        vision_msgs::msg::Detection2DArray out;
        out.header = msg->header;

        for (const auto& bb : detections) {
            vision_msgs::msg::Detection2D det;
            det.header = msg->header;

            det.bbox.center.position.x = static_cast<double>(bb.x + bb.w * 0.5f);
            det.bbox.center.position.y = static_cast<double>(bb.y + bb.h * 0.5f);
            det.bbox.size_x = static_cast<double>(bb.w);
            det.bbox.size_y = static_cast<double>(bb.h);

            vision_msgs::msg::ObjectHypothesisWithPose hyp;
            hyp.hypothesis.class_id = std::to_string(bb.class_id);
            hyp.hypothesis.score    = static_cast<double>(bb.confidence);
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
