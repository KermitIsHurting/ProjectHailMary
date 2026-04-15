// @file radar_sim.cpp
// @brief RadarSim implementation: Box-Muller Gaussian noise, doppler projection.
#include "cuas_fusion/radar_sim.hpp"

#include <cmath>

namespace cuas {

static constexpr float32_t kTwoPi           = 6.2831853F;
static constexpr float32_t kMinRangeM       = 0.1F;
static constexpr float32_t kSnrReferenceM   = 10.0F;
static constexpr uint32_t  kLcgMultiplier   = 1664525U;
static constexpr uint32_t  kLcgIncrement    = 1013904223U;
static constexpr float32_t kLcgScale        = 2.3283064e-10F;

// WHY: Box-Muller consumes pairs; the spare sample is held in function-local
// static state to avoid heap, <random>, and member mutation under a const API.
static float32_t gaussian_sample(const float32_t sigma)
{
    static uint32_t  rng_state    = 0x12345U;
    static float32_t spare        = 0.0F;
    static bool      have_spare   = false;

    if (have_spare) {
        have_spare = false;
        return spare * sigma;
    }

    rng_state = (rng_state * kLcgMultiplier) + kLcgIncrement;
    float32_t u1 = static_cast<float32_t>(rng_state) * kLcgScale;
    if (u1 < 1.0e-7F) {
        u1 = 1.0e-7F;
    }

    rng_state = (rng_state * kLcgMultiplier) + kLcgIncrement;
    const float32_t u2 = static_cast<float32_t>(rng_state) * kLcgScale;

    const float32_t mag   = std::sqrt(-2.0F * std::log(u1));
    const float32_t theta = kTwoPi * u2;
    const float32_t z0    = mag * std::cos(theta);
    const float32_t z1    = mag * std::sin(theta);

    spare      = z1;
    have_spare = true;

    return z0 * sigma;
}

RadarSim::RadarSim()
: noise_sigma_m_(0.0F)
, targets_()
{
    for (uint32_t i = 0U; i < kRadarSimMaxTargets; ++i) {
        const SimTarget t{0.0F, 0.0F, 0.0F, 0.0F, false};
        (void)targets_.push_back(t);
    }
}

void RadarSim::init(const float32_t noise_sigma_m)
{
    noise_sigma_m_ = noise_sigma_m;
}

void RadarSim::set_target(const uint32_t idx, const SimTarget & t)
{
    if (idx >= kRadarSimMaxTargets) {
        return;
    }
    targets_[idx] = t;
}

uint32_t RadarSim::generate(const float32_t dt_sec,
                            FixedVector<SimPoint, kRadarSimMaxPoints> & out) const
{
    (void)dt_sec;
    out.clear();

    uint32_t emitted = 0U;
    for (uint32_t i = 0U; i < kRadarSimMaxTargets; ++i) {
        const SimTarget t = targets_[i];
        if (!t.active) {
            continue;
        }

        const float32_t nx = gaussian_sample(noise_sigma_m_);
        const float32_t ny = gaussian_sample(noise_sigma_m_);

        const float32_t x_obs = t.x_m + nx;
        const float32_t y_obs = t.y_m + ny;

        float32_t range = std::sqrt((x_obs * x_obs) + (y_obs * y_obs));
        if (range < kMinRangeM) {
            range = kMinRangeM;
        }

        const float32_t bx = -x_obs / range;
        const float32_t by = -y_obs / range;
        const float32_t doppler = (t.vx_mps * bx) + (t.vy_mps * by);

        const float32_t snr = 20.0F * std::log10(kSnrReferenceM / range);

        SimPoint p;
        p.x_m         = x_obs;
        p.y_m         = y_obs;
        p.z_m         = 0.0F;
        p.doppler_mps = doppler;
        p.snr_db      = snr;

        if (!out.push_back(p)) {
            break;
        }
        ++emitted;
    }

    return emitted;
}

} // namespace cuas
