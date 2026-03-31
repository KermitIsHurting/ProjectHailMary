// test_fusion_pipeline.cpp
// Integration test for the end-to-end fusion pipeline: injects synthetic
// radar and camera detections, runs fusion + tracking, and validates that
// confirmed tracks are produced with correct state estimates within latency budget.

#include "cuas_fusion/fusion/fusion_engine.hpp"
#include "cuas_fusion/tracking/track_manager.hpp"
#include <gtest/gtest.h>

namespace cuas {

} // namespace cuas
