// @file radar_sim.hpp
// @brief Pure-math 4-target radar simulator emitting noisy doppler returns.
#pragma once

#include "cuas_fusion/common/fixed_containers.hpp"
#include "cuas_fusion/common/fixed_types.hpp"

namespace cuas {

static constexpr uint32_t kRadarSimMaxTargets = 4U;
static constexpr uint32_t kRadarSimMaxPoints  = 32U;

struct SimTarget {
    float32_t x_m;
    float32_t y_m;
    float32_t vx_mps;
    float32_t vy_mps;
    bool      active;
};

struct SimPoint {
    float32_t x_m;
    float32_t y_m;
    float32_t z_m;
    float32_t doppler_mps;
    float32_t snr_db;
};

class RadarSim {
public:
    RadarSim();

    void init(const float32_t noise_sigma_m);
    void set_target(const uint32_t idx, const SimTarget & t);
    void step(const float32_t dt_sec);
    uint32_t generate(const float32_t dt_sec,
                      FixedVector<SimPoint, kRadarSimMaxPoints> & out) const;

private:
    float32_t                                     noise_sigma_m_;
    FixedVector<SimTarget, kRadarSimMaxTargets>   targets_;
};

} // namespace cuas
