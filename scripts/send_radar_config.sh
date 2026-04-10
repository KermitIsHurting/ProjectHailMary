#!/usr/bin/env bash
# send_radar_config.sh
# Sends a TI IWR6843ISK .cfg file to the radar config port line by line.
# Skips comment lines starting with %, adds 50 ms delay between each command.
#
# Port detection priority:
#   1. udev symlinks (/dev/radar_data, /dev/radar_config)
#   2. Auto-detect CP210x ports by VID:PID 10c4:ea70
#   3. Manual override via CLI argument
#
# Usage:
#   ./send_radar_config.sh [config_file] [config_port]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${1:-${SCRIPT_DIR}/../src/cuas_fusion/config/radar_profile.cfg}"
BAUD=115200
DELAY=0.05   # 50 ms between commands

# --- auto-detection ---

detect_radar_ports() {
  local ports=($(ls /dev/ttyUSB* 2>/dev/null | sort))
  local radar_ports=()
  for port in "${ports[@]}"; do
    local vid=$(udevadm info -q property -n "$port" 2>/dev/null \
                | grep ID_VENDOR_ID | cut -d= -f2)
    local pid=$(udevadm info -q property -n "$port" 2>/dev/null \
                | grep ID_MODEL_ID | cut -d= -f2)
    if [[ "$vid" == "10c4" && "$pid" == "ea70" ]]; then
      radar_ports+=("$port")
    fi
  done
  if [[ ${#radar_ports[@]} -lt 2 ]]; then
    echo "ERROR: Expected 2 radar ports, found ${#radar_ports[@]}" >&2
    echo "Plugged in ports: ${ports[*]}" >&2
    exit 1
  fi
  # Sort — lower number = data port, higher number = config port
  radar_ports=($(printf '%s\n' "${radar_ports[@]}" | sort))
  DATA_PORT="${radar_ports[0]}"
  CONFIG_PORT="${radar_ports[1]}"
  echo "Radar detected: data=$DATA_PORT  config=$CONFIG_PORT"
}

# If CLI override provided, use it directly
if [[ -n "${2:-}" ]]; then
  CONFIG_PORT="$2"
  DATA_PORT="${DATA_PORT:-unknown}"
  echo "Using CLI override: config=$CONFIG_PORT"
# Use symlinks if udev rules have been applied
elif [[ -e /dev/radar_data && -e /dev/radar_config ]]; then
  DATA_PORT=/dev/radar_data
  CONFIG_PORT=/dev/radar_config
  echo "Using udev symlinks: data=$DATA_PORT  config=$CONFIG_PORT"
else
  detect_radar_ports
fi

# --- sanity checks ---
if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "ERROR: Config file not found: $CONFIG_FILE" >&2
    exit 1
fi

if [[ ! -c "$CONFIG_PORT" ]]; then
    echo "ERROR: Port not found or not a character device: $CONFIG_PORT" >&2
    echo "       Check USB connection and try:  ls /dev/ttyUSB*" >&2
    exit 1
fi

if ! stty -F "$CONFIG_PORT" "$BAUD" cs8 -cstopb -parenb raw -echo 2>/dev/null; then
    echo "ERROR: Failed to configure $CONFIG_PORT — do you have permission?" >&2
    echo "       Try:  sudo usermod -aG dialout \$USER  (then log out and back in)" >&2
    exit 1
fi

echo "============================================================"
echo "  Config : $CONFIG_FILE"
echo "  Port   : $CONFIG_PORT @ ${BAUD} baud"
echo "  Delay  : ${DELAY}s between commands"
echo "============================================================"

sent=0
skipped=0
line_num=0

while IFS= read -r line || [[ -n "$line" ]]; do
    (( ++line_num ))

    # strip trailing carriage return (handles Windows-style line endings)
    line="${line%$'\r'}"

    # skip empty lines
    [[ -z "$line" ]] && continue

    # skip comment lines starting with %
    if [[ "$line" == %* ]]; then
        (( ++skipped ))
        continue
    fi

    printf "  [cmd %02d] %s\n" "$sent" "$line"

    # write the command followed by a newline to the serial port
    printf '%s\r\n' "$line" > "$CONFIG_PORT"

    (( ++sent ))
    sleep "$DELAY"

done < "$CONFIG_FILE"

echo "============================================================"
echo "  Done.  Sent: ${sent} commands   Skipped: ${skipped} comments"
echo "============================================================"
