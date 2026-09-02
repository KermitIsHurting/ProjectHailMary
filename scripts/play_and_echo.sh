#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(dirname "$SCRIPT_DIR")"
BAG="${1:-$HOME/demo_take3}"
source "$WORKSPACE/install/setup.bash"
export AMENT_PREFIX_PATH="$WORKSPACE/install/cuas_fusion:$WORKSPACE/install/cuas_msgs:/opt/ros/humble"

ros2 bag play "$BAG" --clock --topics /radar/detections /camera/image_raw &
BAG_PID=$!
sleep 4

python3 -c "
import rclpy
from rclpy.node import Node
from cuas_msgs.msg import FusedDetectionArray, ThreatReportArray

rclpy.init()
node = Node('bag_listener')

got_fusion = [False]
got_threat = [False]

def fusion_cb(msg):
    if got_fusion[0]: return
    got_fusion[0] = True
    print('=== FUSION DETECTIONS ===')
    print(f'  Num detections: {len(msg.detections)}')
    for i, d in enumerate(msg.detections[:5]):
        print(f'  [{i}] range_m={d.range_m:.2f} az={d.azimuth_deg:.1f} vel={d.velocity_mps:.2f} class={d.class_label} conf={d.confidence:.2f} u={d.pixel_u:.0f} v={d.pixel_v:.0f} bw={d.bbox_width_px:.0f} bh={d.bbox_height_px:.0f}')

def threat_cb(msg):
    if got_threat[0]: return
    got_threat[0] = True
    print('=== THREAT REPORTS ===')
    print(f'  Num reports: {len(msg.reports)}')
    for i, r in enumerate(msg.reports[:5]):
        print(f'  [{i}] track_id={r.track_id} threat_level={r.threat_level} pos=({r.position_x_m:.2f},{r.position_y_m:.2f})')

node.create_subscription(FusedDetectionArray, '/fusion/detections', fusion_cb, 10)
node.create_subscription(ThreatReportArray, '/threat/reports', threat_cb, 10)

import time
start = time.time()
while time.time() - start < 20 and not (got_fusion[0] and got_threat[0]):
    rclpy.spin_once(node, timeout_sec=0.1)

if not got_fusion[0]: print('WARNING: No fusion detections received')
if not got_threat[0]: print('WARNING: No threat reports received')

node.destroy_node()
rclpy.shutdown()
"

kill $BAG_PID 2>/dev/null || true
