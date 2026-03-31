// radar_driver.hpp
// Low-level interface to the radar sensor: handles UDP socket I/O,
// raw frame parsing, and publishing RadarFrame messages to ROS 2.

#pragma once

namespace cuas {

class RadarDriver {
public:
    RadarDriver() = default;
};

} // namespace cuas
