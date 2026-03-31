#!/usr/bin/env bash
# run_radar.sh
# One-shot script for ProjectHailMary radar pipeline:
#   1. Sends radar config to /dev/ttyUSB0 (starts sensor)
#   2. Launches radar_parser_node reading from /dev/ttyUSB2
#
# Usage:
#   ./scripts/run_radar.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(dirname "$SCRIPT_DIR")"
CONFIG="$WORKSPACE/src/cuas_fusion/config/radar_profile.cfg"

# ---- ROS 2 environment ----
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH="$WORKSPACE/install/cuas_fusion:$WORKSPACE/install/cuas_msgs:$AMENT_PREFIX_PATH"

echo "============================================================"
echo "  ProjectHailMary — Radar Startup"
echo "  Config : $CONFIG"
echo "  Ctrl-C to stop"
echo "============================================================"

# ---- Send radar config ----
echo "[1/2] Configuring radar..."
python3 "$SCRIPT_DIR/test_radar.py" "$CONFIG" /dev/ttyUSB0 /dev/ttyUSB2
echo

# ---- Launch parser node ----
echo "[2/2] Launching radar_parser_node on /dev/ttyUSB2..."
ros2 run cuas_fusion radar_parser_node /dev/ttyUSB2
