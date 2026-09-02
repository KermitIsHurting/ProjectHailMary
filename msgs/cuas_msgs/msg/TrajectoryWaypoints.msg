std_msgs/Header header
uint32 track_id
# Bound = kMaxTrajectorySteps in common/param_utils.hpp (P3.1 / DEV-011).
float64[<=64] waypoints_x_m
float64[<=64] waypoints_y_m
float64[<=64] waypoints_z_m
float64[<=64] timestamps_sec
float64[<=64] uncertainty_radii_m
float64[<=64] bearing_deg
float64[<=64] elevation_deg
