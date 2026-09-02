#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$(dirname "$SCRIPT_DIR")/install/setup.bash"

echo "=== THREAT REPORT FIELDS ==="
python3 -c "
import rclpy
from rclpy.node import Node
from cuas_msgs.msg import ThreatReportArray
rclpy.init()
node = Node('checker')
got = [False]
def cb(msg):
    if got[0]: return
    got[0] = True
    for r in msg.reports[:3]:
        print(f'  id={r.track_id} threat={r.threat_level} esc={r.escalation_state} q={r.quality_score:.2f} dwell={r.dwell_time_s:.1f}s pred=({r.predicted_impact_x_m:.1f},{r.predicted_impact_y_m:.1f}) zones={len(r.exclusion_zones_violated)}')
node.create_subscription(ThreatReportArray, '/threat/reports', cb, 10)
import time
start = time.time()
while time.time() - start < 15 and not got[0]:
    rclpy.spin_once(node, timeout_sec=0.1)
if not got[0]: print('  WARNING: No threat reports received')
node.destroy_node()
rclpy.shutdown()
"

echo ""
echo "=== COT UDP CHECK ==="
timeout 3 python3 -c "
import socket, struct
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', 6969))
mreq = struct.pack('4sl', socket.inet_aton('239.2.3.1'), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
sock.settimeout(3.0)
try:
    data, addr = sock.recvfrom(4096)
    print(f'  Received {len(data)} bytes from {addr}')
    print(f'  {data[:200].decode(\"utf-8\", errors=\"replace\")}')
except socket.timeout:
    print('  No CoT packets received (expected if no THREATENING tracks)')
sock.close()
" 2>&1 || echo "  (timeout - no THREATENING/ENGAGED tracks active)"
