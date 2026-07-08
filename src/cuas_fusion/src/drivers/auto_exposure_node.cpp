// @file auto_exposure_node.cpp
// @brief Closed-loop exposure/gain control via V4L2 from image brightness.
//
// The camera driver deliberately never touches exposure (protected file,
// ROLLBACK.md); this node owns brightness on a SEPARATE control-only fd.
// V4L2 controls are device-global and may be set while another process
// streams — this automates the manual `v4l2-ctl -c exposure=` workflow.
#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_types.hpp"
#include "cuas_fusion/common/ros_image_adapter.hpp"
#include "cuas_fusion/drivers/auto_exposure.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core.hpp>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

namespace cuas {

class AutoExposureNode final : public rclcpp::Node {
public:
    AutoExposureNode() : Node("auto_exposure_node")
    {
        declare_parameter("device", std::string("/dev/video0"));
        declare_parameter("target_mean", 110.0);
        declare_parameter("deadband", 10.0);
        declare_parameter("max_step_frac", 0.25);
        declare_parameter("exposure_min", 2);
        declare_parameter("exposure_max", 8000);
        declare_parameter("gain_min", 100);
        declare_parameter("gain_max", 1200);
        declare_parameter("update_period_s", 0.3);

        device_ = get_parameter("device").as_string();
        params_.target_mean =
            static_cast<float32_t>(get_parameter("target_mean").as_double());
        params_.deadband =
            static_cast<float32_t>(get_parameter("deadband").as_double());
        params_.max_step_frac =
            static_cast<float32_t>(get_parameter("max_step_frac").as_double());
        params_.limits.exposure_min =
            static_cast<int32_t>(get_parameter("exposure_min").as_int());
        params_.limits.exposure_max =
            static_cast<int32_t>(get_parameter("exposure_max").as_int());
        params_.limits.gain_min =
            static_cast<int32_t>(get_parameter("gain_min").as_int());
        params_.limits.gain_max =
            static_cast<int32_t>(get_parameter("gain_max").as_int());

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            CAMERA_TOPIC, rclcpp::QoS(1),
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                latest_ = std::move(msg);
            });

        const auto period_s = get_parameter("update_period_s").as_double();
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<float64_t>(period_s)),
            [this]() { tick(); });

        RCLCPP_INFO(get_logger(),
                    "Auto exposure ready — device %s, target mean %.1f",
                    device_.c_str(),
                    static_cast<float64_t>(params_.target_mean));
    }

    ~AutoExposureNode() override
    {
        if (fd_ >= 0) {
            (void)::close(fd_);
        }
    }

private:
    // Control-only open; adopts the sensor's current settings on attach so
    // a manually-tuned starting point is respected, not stomped.
    bool ensureDevice()
    {
        if (fd_ >= 0) {
            return true;
        }
        fd_ = ::open(device_.c_str(), O_RDWR);
        if (fd_ < 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "AE: cannot open %s; retrying",
                                 device_.c_str());
            return false;
        }
        if (!getCtrl(V4L2_CID_EXPOSURE, exposure_) ||
            !getCtrl(V4L2_CID_ANALOGUE_GAIN, gain_)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "AE: control read failed on %s; retrying",
                                 device_.c_str());
            (void)::close(fd_);
            fd_ = -1;
            return false;
        }
        RCLCPP_INFO(get_logger(), "AE attached: exposure=%d gain=%d",
                    exposure_, gain_);
        return true;
    }

    bool getCtrl(const uint32_t id, int32_t& value)
    {
        struct v4l2_control ctrl {};
        ctrl.id = id;
        if (::ioctl(fd_, VIDIOC_G_CTRL, &ctrl) != 0) {
            return false;
        }
        value = ctrl.value;
        return true;
    }

    bool setCtrl(const uint32_t id, const int32_t value)
    {
        struct v4l2_control ctrl {};
        ctrl.id = id;
        ctrl.value = value;
        return ::ioctl(fd_, VIDIOC_S_CTRL, &ctrl) == 0;
    }

    void tick()
    {
        if (!ensureDevice()) {
            return;
        }
        // DEV-010: single-threaded executor; latest_ is written only in the
        // subscription callback on this same thread.
        const sensor_msgs::msg::Image::ConstSharedPtr img = latest_;
        if (img == nullptr) {
            return;
        }
        if (!rosImageToBgr(*img, frame_)) {
            return;
        }
        const cv::Scalar ch = cv::mean(frame_);
        const auto mean_luma = static_cast<float32_t>(
            (0.114 * ch[0]) + (0.587 * ch[1]) + (0.299 * ch[2]));

        const AeCommand cmd = nextAeCommand(exposure_, gain_, mean_luma,
                                            params_);
        if (!cmd.changed) {
            return;
        }
        bool ok = true;
        if (cmd.exposure != exposure_) {
            ok = setCtrl(V4L2_CID_EXPOSURE, cmd.exposure) && ok;
        }
        if (cmd.gain != gain_) {
            ok = setCtrl(V4L2_CID_ANALOGUE_GAIN, cmd.gain) && ok;
        }
        if (!ok) {
            // Device may have re-enumerated (unplug); reattach next tick.
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "AE: control write failed; reattaching");
            (void)::close(fd_);
            fd_ = -1;
            return;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "AE: mean=%.1f exposure %d->%d gain %d->%d",
                             static_cast<float64_t>(mean_luma),
                             exposure_, cmd.exposure, gain_, cmd.gain);
        exposure_ = cmd.exposure;
        gain_ = cmd.gain;
    }

    std::string device_;
    AeParams params_{};
    int fd_ = -1;
    int32_t exposure_ = 0;
    int32_t gain_ = 0;
    cv::Mat frame_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace cuas

// Single sanctioned exception boundary (DEV-001).
int main(int argc, char** argv)
{
    int exit_code = 0;
    try {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<cuas::AutoExposureNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::fprintf(stderr,
                     "FATAL: unhandled exception in AutoExposureNode: %s\n",
                     e.what());
        exit_code = 1;
    } catch (...) {
        std::fprintf(stderr,
                     "FATAL: unhandled non-std exception in AutoExposureNode\n");
        exit_code = 1;
    }
    return exit_code;
}
