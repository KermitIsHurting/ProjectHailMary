std_msgs/Header header
# Bound = planned RADAR_MAX_POINTS_PER_FRAME (HANDOFF §6.1c); message is
# defined but not currently published (parser emits PointCloud2 directly).
RadarDetection[<=256] detections
