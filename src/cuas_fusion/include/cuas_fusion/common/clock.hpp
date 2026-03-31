// clock.hpp
// Centralized clock abstraction providing monotonic and ROS sim-time access,
// used to stamp detections and synchronize multi-sensor data streams.

#pragma once

namespace cuas {

class Clock {
public:
    Clock() = default;
};

} // namespace cuas
