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
  # The CP2105 exposes two USB interfaces: 00 = CLI/config, 01 = data — the
  # same rule scripts/99-iwr6843.rules encodes. The old "lower ttyUSB index
  # = data" guess was inverted on this box (RC-24).
  local ports=($(ls /dev/ttyUSB* 2>/dev/null | sort))
  DATA_PORT=""
  CONFIG_PORT=""
  for port in "${ports[@]}"; do
    local props
    props=$(udevadm info -q property -n "$port" 2>/dev/null)
    local vid=$(grep '^ID_VENDOR_ID=' <<<"$props" | cut -d= -f2)
    local pid=$(grep '^ID_MODEL_ID=' <<<"$props" | cut -d= -f2)
    local iface=$(grep '^ID_USB_INTERFACE_NUM=' <<<"$props" | cut -d= -f2)
    if [[ "$vid" == "10c4" && "$pid" == "ea70" ]]; then
      case "$iface" in
        01) DATA_PORT="$port" ;;
        00) CONFIG_PORT="$port" ;;
      esac
    fi
  done
  if [[ -z "$DATA_PORT" || -z "$CONFIG_PORT" ]]; then
    echo "ERROR: need CP210x interfaces 00 (config) and 01 (data); found data='$DATA_PORT' config='$CONFIG_PORT'" >&2
    echo "Plugged in ports: ${ports[*]}" >&2
    exit 1
  fi
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
