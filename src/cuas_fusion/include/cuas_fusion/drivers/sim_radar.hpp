// @file sim_radar.hpp
// @brief Pure-math radar simulator generating synthetic point cloud detections.
#pragma once

#include "cuas_fusion/common/constants.hpp"
#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

namespace cuas {

struct ScenarioTarget {
    float32_t x_m       = 0.0F;
    float32_t y_m       = 0.0F;
    float32_t z_m       = 0.0F;
    float32_t vx_mps    = 0.0F;
    float32_t vy_mps    = 0.0F;
    float32_t vz_mps    = 0.0F;
    float32_t rcs_dbsm  = 0.0F;
    uint32_t  target_id = 0U;
};

struct SimRadarPoint {
    float32_t x_m         = 0.0F;
    float32_t y_m         = 0.0F;
    float32_t z_m         = 0.0F;
    float32_t doppler_mps = 0.0F;
};

class SimRadar {
public:
    explicit SimRadar(uint32_t seed);

    void addTarget(const ScenarioTarget& target);
    void step(float32_t dt_s);
    FixedVector<SimRadarPoint, kSimRadarMaxPoints> getPoints() const;
    ScenarioTarget& getTarget(uint32_t index);

private:
    float32_t gaussian_sample() const;

    FixedVector<ScenarioTarget, kSimRadarMaxTargets> targets_;
    mutable uint32_t rng_state_;
};

} // namespace cuas
