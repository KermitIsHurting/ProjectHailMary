float32 position_x_m
float32 position_y_m
float32 position_z_m
float32 velocity_mps
string  class_label
float32 confidence
float32 pixel_u
float32 pixel_v
int64   timestamp_ns
float32 range_m
float32 azimuth_deg
float32 bbox_width_px
float32 bbox_height_px

# --- OVERHAUL P3.1 additive field (old bags replay with zero here) ---
# Bit 0 = radar, bit 1 = camera contributed to this detection
# (track_source constants in common/track_state_ids.hpp).
uint8 source_mask
