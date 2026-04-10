#!/usr/bin/env bash
# run_radar.sh
# One-shot script for ProjectHailMary radar pipeline:
#   1. Sends radar config to the config port (starts sensor)
#   2. Launches radar_parser_node reading from the data port
#
# Port detection priority:
#   1. udev symlinks (/dev/radar_data, /dev/radar_config)
#   2. Auto-detect CP210x ports by VID:PID 10c4:ea60
#   3. ROS 2 parameter override at launch time
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

# ---- Detect ports ----
if [[ -e /dev/radar_data && -e /dev/radar_config ]]; then
  DATA_PORT=/dev/radar_data
  CONFIG_PORT=/dev/radar_config
  echo "Using udev symlinks: data=$DATA_PORT  config=$CONFIG_PORT"
else
  echo "No udev symlinks found, using auto-detection..."
  source "$SCRIPT_DIR/send_radar_config.sh" --detect-only 2>/dev/null || true
  : "${DATA_PORT:=/dev/radar_data}"
  : "${CONFIG_PORT:=/dev/radar_config}"
fi

echo "============================================================"
echo "  ProjectHailMary — Radar Startup"
echo "  Config : $CONFIG"
echo "  Data   : $DATA_PORT"
echo "  Config : $CONFIG_PORT"
echo "  Ctrl-C to stop"
echo "============================================================"

# ---- Send radar config ----
echo "[1/2] Configuring radar..."
python3 "$SCRIPT_DIR/test_radar.py" "$CONFIG" "$CONFIG_PORT" "$DATA_PORT"
echo

# ---- Launch parser node (uses ROS param for port) ----
echo "[2/2] Launching radar_parser_node on $DATA_PORT..."
ros2 run cuas_fusion radar_parser_node --ros-args \
  -p data_port:="$DATA_PORT" \
  -p config_port:="$CONFIG_PORT"
