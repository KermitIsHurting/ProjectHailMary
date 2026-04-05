#include "cuas_fusion/tracking/track_manager.hpp"
#include "cuas_fusion/common/constants.hpp"

#include <cmath>
#include <cstdio>

namespace cuas {

bool TrackManager::init()
{
    for (auto& e : entries_) {
        e.active     = false;
        e.hit_count  = 0;
        e.miss_count = 0;
    }
    next_id_ = 1;
    return true;
}

size_t TrackManager::findNearest(float x, float y, float z) const
{
    size_t best_idx  = TRACK_MAX_TRACKS;
    float  best_dist = TRACK_ASSOCIATION_DIST_M;

    for (size_t i = 0; i < TRACK_MAX_TRACKS; ++i) {
        if (!entries_[i].active) {
            continue;
        }
        const Track& t = entries_[i].track;
        float dx = t.position_x_m_ - x;
        float dy = t.position_y_m_ - y;
        float dz = t.position_z_m_ - z;
        float d  = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (d < best_dist) {
            best_dist = d;
            best_idx  = i;
        }
    }
    return best_idx;
}

size_t TrackManager::allocateSlot()
{
    for (size_t i = 0; i < TRACK_MAX_TRACKS; ++i) {
        if (!entries_[i].active) {
            return i;
        }
    }
    return TRACK_MAX_TRACKS;
}

bool TrackManager::update(const std::vector<FusedDetection>& detections,
                          std::vector<Track>& confirmed_out)
{
    confirmed_out.clear();

    bool matched[TRACK_MAX_TRACKS] = {};

    for (const auto& det : detections) {
        size_t idx = findNearest(det.position_x_m, det.position_y_m, det.position_z_m);

        if (idx < TRACK_MAX_TRACKS) {
            Track& t = entries_[idx].track;
            t.position_x_m_ = det.position_x_m;
            t.position_y_m_ = det.position_y_m;
            t.position_z_m_ = det.position_z_m;
            t.velocity_mps_ = std::abs(det.velocity_mps);
            t.doppler_mps_  = det.velocity_mps;
            t.class_label_  = det.class_label;
            t.confidence_   = det.confidence;
            t.timestamp_ns_ = det.timestamp_ns;

            entries_[idx].hit_count++;
            entries_[idx].miss_count = 0;
            matched[idx] = true;

            if (entries_[idx].hit_count >= TRACK_CONFIRM_HITS) {
                t.state_ = TrackState::CONFIRMED;
            }
        } else {
            size_t slot = allocateSlot();
            if (slot >= TRACK_MAX_TRACKS) {
                fprintf(stderr, "[TrackManager] at capacity (%zu tracks), dropping detection\n",
                        TRACK_MAX_TRACKS);
                continue;
            }
            TrackEntry& e = entries_[slot];
            e.active     = true;
            e.hit_count  = 1;
            e.miss_count = 0;
            matched[slot] = true;

            Track& t = e.track;
            t.track_id_      = next_id_++;
            t.position_x_m_  = det.position_x_m;
            t.position_y_m_  = det.position_y_m;
            t.position_z_m_  = det.position_z_m;
            t.velocity_mps_  = std::abs(det.velocity_mps);
            t.doppler_mps_   = det.velocity_mps;
            t.class_label_   = det.class_label;
            t.confidence_    = det.confidence;
            t.state_         = TrackState::TENTATIVE;
            t.timestamp_ns_  = det.timestamp_ns;
        }
    }

    for (size_t i = 0; i < TRACK_MAX_TRACKS; ++i) {
        if (!entries_[i].active || matched[i]) {
            continue;
        }
        entries_[i].miss_count++;
        entries_[i].hit_count = 0;
        if (entries_[i].miss_count >= TRACK_MAX_MISSES) {
            entries_[i].active = false;
        } else {
            entries_[i].track.state_ = TrackState::COASTED;
        }
    }

    for (size_t i = 0; i < TRACK_MAX_TRACKS; ++i) {
        if (entries_[i].active &&
            entries_[i].track.state_ == TrackState::CONFIRMED)
        {
            confirmed_out.push_back(entries_[i].track);
        }
    }

    return true;
}

} // namespace cuas
