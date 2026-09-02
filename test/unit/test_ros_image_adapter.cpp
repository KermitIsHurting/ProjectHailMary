// test_ros_image_adapter.cpp
// Unit tests for the non-throwing Image -> cv::Mat adapter: a short or
// empty data buffer must be rejected, not read past its end (audit B4 / RC-8a).

#include "cuas_fusion/common/ros_image_adapter.hpp"

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

sensor_msgs::msg::Image make_image(const char* encoding, uint32_t w, uint32_t h)
{
    sensor_msgs::msg::Image img;
    img.encoding = encoding;
    img.width    = w;
    img.height   = h;
    img.step     = w * 3U;
    img.data.assign(static_cast<std::size_t>(img.step) * h, 0U);
    return img;
}

} // namespace

TEST(RosImageAdapter, AcceptsWellFormedBgr8)
{
    const auto img = make_image("bgr8", 64U, 48U);
    cv::Mat out;
    ASSERT_TRUE(cuas::rosImageToBgr(img, out));
    EXPECT_EQ(out.rows, 48);
    EXPECT_EQ(out.cols, 64);
}

TEST(RosImageAdapter, RejectsEmptyShortAndNarrowStepBuffers)
{
    cv::Mat out;

    auto empty = make_image("bgr8", 64U, 48U);
    empty.data.clear();
    EXPECT_FALSE(cuas::rosImageToBgr(empty, out));

    auto truncated = make_image("bgr8", 64U, 48U);
    truncated.data.resize(truncated.data.size() - 1U);
    EXPECT_FALSE(cuas::rosImageToBgr(truncated, out));

    auto narrow = make_image("rgb8", 64U, 48U);
    narrow.step = 64U * 2U;   // claims 2 bytes per pixel for a 3-channel encoding
    EXPECT_FALSE(cuas::rosImageToBgr(narrow, out));

    auto zero = make_image("bgr8", 0U, 48U);
    EXPECT_FALSE(cuas::rosImageToBgr(zero, out));
}

TEST(RosImageAdapter, Rgb8IsConvertedToBgr)
{
    auto img = make_image("rgb8", 2U, 1U);
    img.data = {255U, 0U, 0U, 0U, 0U, 255U};   // red, blue in RGB
    cv::Mat out;
    ASSERT_TRUE(cuas::rosImageToBgr(img, out));
    const cv::Vec3b p0 = out.at<cv::Vec3b>(0, 0);
    const cv::Vec3b p1 = out.at<cv::Vec3b>(0, 1);
    EXPECT_EQ(p0[2], 255U);   // red lands in the R (index 2) channel of BGR
    EXPECT_EQ(p1[0], 255U);   // blue lands in B
}

TEST(RosImageAdapter, UnknownEncodingIsRejected)
{
    const auto img = make_image("mono8", 8U, 8U);
    cv::Mat out;
    EXPECT_FALSE(cuas::rosImageToBgr(img, out));
}
