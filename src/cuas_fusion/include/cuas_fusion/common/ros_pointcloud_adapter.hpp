// @file ros_pointcloud_adapter.hpp
// @brief Non-throwing layout guards for sensor_msgs::msg::PointCloud2 input.
#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>

namespace cuas {

inline bool cloudHasFloat32Field(const sensor_msgs::msg::PointCloud2& msg,
                                 const char* name)
{
    const auto it = std::find_if(msg.fields.begin(), msg.fields.end(),
        [name](const sensor_msgs::msg::PointField& f) { return f.name == name; });
    return (it != msg.fields.end()) &&
           (it->datatype == sensor_msgs::msg::PointField::FLOAT32);
}

// PointCloud2Iterator's constructor throws on a missing field and takes
// data.front() of an empty cloud (RC-8, R6a-12); callers check both first.
inline bool cloudHasFloat32Xyz(const sensor_msgs::msg::PointCloud2& msg)
{
    return cloudHasFloat32Field(msg, "x") && cloudHasFloat32Field(msg, "y") &&
           cloudHasFloat32Field(msg, "z");
}

inline bool cloudIsEmpty(const sensor_msgs::msg::PointCloud2& msg)
{
    return (msg.width == 0U) || (msg.height == 0U) || msg.data.empty();
}

} // namespace cuas
