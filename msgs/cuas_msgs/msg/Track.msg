uint32  track_id
float32 position_x_m
float32 position_y_m
float32 position_z_m
float32 velocity_mps
float32 doppler_mps
string  class_label
float32 confidence           # 0.0 to 1.0, decays on miss
string  track_state
int64   timestamp_ns
bool    is_maneuvering       # true when CT model dominant 3+ frames
float32 imm_ct_probability   # current CT model probability 0.0 to 1.0
