// @file sim_radar.cpp
// @brief SimRadar implementation with seeded LCG noise and doppler projection.
#include "cuas_fusion/drivers/sim_radar.hpp"

#include <cmath>

namespace cuas {

static constexpr float32_t kTwoPi         = 6.2831853F;
static constexpr uint32_t  kLcgMultiplier = 1664525U;
static constexpr uint32_t  kLcgIncrement  = 1013904223U;
static constexpr float32_t kLcgScale      = 2.3283064e-10F;
static constexpr float32_t kMinRange      = 0.01F;

SimRadar::SimRadar(uint32_t seed)
    : targets_()
    , rng_state_(seed)
{
}

void SimRadar::addTarget(const ScenarioTarget& target)
{
    (void)targets_.push_back(target);
}

void SimRadar::step(float32_t dt_s)
{
    for (uint32_t i = 0U; i < targets_.size(); ++i) {
        ScenarioTarget& t = targets_[i];
        t.x_m += t.vx_mps * dt_s;
        t.y_m += t.vy_mps * dt_s;
        t.z_m += t.vz_mps * dt_s;
    }
}

float32_t SimRadar::gaussian_sample() const
{
    rng_state_ = (rng_state_ * kLcgMultiplier) + kLcgIncrement;
    float32_t u1 = static_cast<float32_t>(rng_state_) * kLcgScale;
    if (u1 < 1.0e-7F) {
        u1 = 1.0e-7F;
    }
    rng_state_ = (rng_state_ * kLcgMultiplier) + kLcgIncrement;
    const float32_t u2 = static_cast<float32_t>(rng_state_) * kLcgScale;
    const float32_t mag = std::sqrt(-2.0F * std::log(u1));
    return mag * std::cos(kTwoPi * u2);
}

FixedVector<SimRadarPoint, kSimRadarMaxPoints> SimRadar::getPoints() const
{
    FixedVector<SimRadarPoint, kSimRadarMaxPoints> out;

    for (uint32_t i = 0U; i < targets_.size(); ++i) {
        const ScenarioTarget& t = targets_[i];

        const float32_t range_sq = t.x_m * t.x_m + t.y_m * t.y_m + t.z_m * t.z_m;
        if (range_sq > kSimRadarMaxRangeM * kSimRadarMaxRangeM) {
            continue;
        }

        // One centroid per target: /radar/detections carries cluster
        // centroids (ICD §3), and the parser's DBSCAN would have merged the
        // raw returns of one body. Publishing 2-8 raw returns per target
        // made the tracker spawn twin tracks (A6 approach run).
        (void)t.rcs_dbsm;
        for (int32_t p = 0; p < 1; ++p) {
            const float32_t nx = gaussian_sample() * kSimRadarNoiseSigmaM;
            const float32_t ny = gaussian_sample() * kSimRadarNoiseSigmaM;
            const float32_t nz = gaussian_sample() * kSimRadarNoiseSigmaM;

            const float32_t x_obs = t.x_m + nx;
            const float32_t y_obs = t.y_m + ny;
            const float32_t z_obs = t.z_m + nz;

            float32_t obs_range = std::sqrt(x_obs * x_obs + y_obs * y_obs + z_obs * z_obs);
            if (obs_range > kSimRadarMaxRangeM) {
                continue;
            }
            if (obs_range < kMinRange) {
                obs_range = kMinRange;
            }

            // Radial velocity in the TI convention: positive = receding,
            // negative = closing (verified against the April hardware bags,
            // audit D-7). The old leading minus inverted the sign (RC-36).
            const float32_t doppler =
                (t.vx_mps * x_obs + t.vy_mps * y_obs + t.vz_mps * z_obs) / obs_range;

            SimRadarPoint pt;
            pt.x_m         = x_obs;
            pt.y_m         = y_obs;
            pt.z_m         = z_obs;
            pt.doppler_mps = doppler;

            if (!out.push_back(pt)) {
                return out;
            }
        }
    }

    return out;
}

ScenarioTarget& SimRadar::getTarget(uint32_t index)
{
    // Out-of-range access into the FixedVector is UB by its contract; hand
    // back an inert dummy instead (A2.9). The node is single-threaded.
    if (index >= targets_.size()) {
        static ScenarioTarget dummy{};
        dummy = ScenarioTarget{};
        return dummy;
    }
    return targets_[index];
}

} // namespace cuas
