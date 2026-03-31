#!/usr/bin/env bash
# send_radar_config.sh
# Sends a TI IWR6843ISK .cfg file to the radar config port (/dev/ttyUSB0) line by line.
# Skips comment lines starting with %, adds 50 ms delay between each command.
#
# Usage:
#   ./send_radar_config.sh [config_file] [port]
#
# Defaults:
#   config_file : ../src/cuas_fusion/config/radar_profile.cfg (relative to this script)
#   port        : /dev/ttyUSB0

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${1:-${SCRIPT_DIR}/../src/cuas_fusion/config/radar_profile.cfg}"
CONFIG_PORT="${2:-/dev/ttyUSB0}"
BAUD=115200
DELAY=0.05   # 50 ms between commands

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
