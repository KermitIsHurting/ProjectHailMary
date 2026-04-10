#!/usr/bin/env python3
"""
test_radar.py — IWR6843ISK diagnostic script
Sends full radar config to the config port, captures all responses,
then listens on the data port for binary data frames.

Port detection priority:
    1. udev symlinks (/dev/radar_data, /dev/radar_config)
    2. CLI arguments
    3. Auto-detect CP210x ports by VID:PID 10c4:ea60

Usage:
    python3 test_radar.py [config_file] [config_port] [data_port]
Defaults:
    config_file : ../src/cuas_fusion/config/radar_profile.cfg
    config_port : /dev/radar_config  (115200 baud, UART CLI)
    data_port   : /dev/radar_data    (921600 baud, binary output)
"""

import os
import sys
import time
import threading
import serial

# ---------------------------------------------------------------------------
# Paths / ports
# ---------------------------------------------------------------------------
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
WORKSPACE   = os.path.dirname(SCRIPT_DIR)

CONFIG_FILE = (sys.argv[1] if len(sys.argv) > 1
               else os.path.join(WORKSPACE,
                                 "src/cuas_fusion/config/radar_profile.cfg"))
CONFIG_PORT = sys.argv[2] if len(sys.argv) > 2 else "/dev/radar_config"
DATA_PORT   = sys.argv[3] if len(sys.argv) > 3 else "/dev/radar_data"

CONFIG_BAUD  = 115200
DATA_BAUD    = 921600
CMD_DELAY_S  = 0.200   # 200 ms between commands
LISTEN_S     = 5       # seconds to listen on data port after sensorStart
BOOT_WAIT_S  = 3.0    # seconds to wait for DSS to boot after reset


# ---------------------------------------------------------------------------
# Step 1: Verify serial ports exist
# ---------------------------------------------------------------------------
def check_ports():
    print("=" * 60)
    print("STEP 1 — Verify serial ports")
    print("=" * 60)
    ok = True
    for port in (CONFIG_PORT, DATA_PORT):
        exists = os.path.exists(port)
        readable = os.access(port, os.R_OK | os.W_OK) if exists else False
        status = "OK" if (exists and readable) else ("EXISTS but NO PERMISSION" if exists else "MISSING")
        print(f"  {port}: {status}")
        if not exists or not readable:
            ok = False
    if not ok:
        print()
        print("  Fix: sudo usermod -aG dialout $USER  (then log out/in)")
        print("  Or:  sudo chmod a+rw", CONFIG_PORT, DATA_PORT)
        sys.exit(1)
    print()


# ---------------------------------------------------------------------------
# Step 2: Read config file, stripping comments and blanks
# ---------------------------------------------------------------------------
def load_config():
    print("=" * 60)
    print(f"STEP 2 — Load config: {CONFIG_FILE}")
    print("=" * 60)
    if not os.path.exists(CONFIG_FILE):
        print(f"  ERROR: config file not found: {CONFIG_FILE}")
        sys.exit(1)

    commands = []
    with open(CONFIG_FILE, "r") as f:
        for raw in f:
            line = raw.rstrip("\r\n").strip()
            if not line or line.startswith("%"):
                continue
            commands.append(line)

    print(f"  Loaded {len(commands)} commands")
    for i, cmd in enumerate(commands):
        print(f"    [{i:02d}] {cmd}")
    print()
    return commands


# ---------------------------------------------------------------------------
# Helper: read all waiting bytes from serial port (non-blocking drain)
# ---------------------------------------------------------------------------
def drain_response(ser, timeout=0.15):
    """Read everything the radar sends back within `timeout` seconds."""
    ser.timeout = timeout
    chunks = []
    while True:
        chunk = ser.read(4096)
        if not chunk:
            break
        chunks.append(chunk)
    return b"".join(chunks).decode("ascii", errors="replace")


# ---------------------------------------------------------------------------
# Step 3: Send config and capture responses
# ---------------------------------------------------------------------------
def reset_and_boot(ser):
    """
    Attempt a hardware reset via DTR toggle, then wait for the boot banner.
    On IWR6843ISK EVMs the XDS110 FTDI chip typically wires nDTR → nRST.
    If this doesn't reset the board (no banner printed), that's OK — we'll
    still try to configure the already-running demo.
    """
    print("=" * 60)
    print("STEP 3a — Attempt hardware reset via DTR toggle")
    print("=" * 60)

    try:
        ser.dtr = True          # assert DTR (nRST = low = RESET)
        time.sleep(0.05)
        ser.dtr = False         # release DTR (nRST = high = running)
        ser.reset_input_buffer()

        print(f"  DTR toggled — waiting {BOOT_WAIT_S}s for MSS+DSS to boot...")
        time.sleep(BOOT_WAIT_S)

        banner = drain_response(ser, timeout=0.5)
        if banner.strip():
            print("  Boot banner captured:")
            for line in banner.splitlines():
                if line.strip():
                    print(f"    {line}")
            if "mmwDemo" in banner:
                print("  [OK] Demo firmware prompt seen — MSS is up")
            if "DSS" in banner or "Sensor" in banner or "version" in banner.lower():
                print("  [OK] DSS-related boot message seen — DSS appears to be running")
            else:
                print("  [WARN] No DSS boot message seen — DSS may not have started")
                print("         -> If sensorStart still fails, do a full power cycle:")
                print("            Unplug the EVM power jack, wait 5s, plug back in,")
                print("            then re-run this script.")
        else:
            print("  No boot banner received — device was already running (no reset).")
            print("  DTR is likely not wired to nRST on this EVM.")
            print()
            print("  ACTION REQUIRED if sensorStart keeps failing:")
            print("    1. Physically unplug the EVM power (barrel jack or USB power)")
            print("    2. Confirm SOP2 switch is ON (functional mode)")
            print("    3. Plug power back in and wait 3 seconds")
            print("    4. Re-run this script")
    except Exception as e:
        print(f"  DTR toggle failed: {e} (skipping reset step)")
    print()


def probe_firmware(ser):
    """
    Send 'help' and a few version probes to learn what the firmware supports.
    This helps identify any required CLI commands we might be missing.
    """
    print("=" * 60)
    print("STEP 3b — Probe firmware (help / version)")
    print("=" * 60)

    ser.reset_input_buffer()

    for probe_cmd in ("help", "version", "mmwaveVersion"):
        ser.write((probe_cmd + "\r\n").encode("ascii"))
        ser.flush()
        time.sleep(0.3)
        resp = drain_response(ser, timeout=0.3)
        lines = [l.strip() for l in resp.splitlines() if l.strip()]
        if lines and not all("not recognized" in l or l == probe_cmd
                             or "mmwDemo" in l for l in lines):
            print(f"  [{probe_cmd}] →")
            for l in lines:
                print(f"    {l}")
        else:
            print(f"  [{probe_cmd}] → not recognized or empty")

    print()


def send_config(commands):
    print("=" * 60)
    print(f"STEP 3 — Send config to {CONFIG_PORT} @ {CONFIG_BAUD} baud")
    print("=" * 60)

    try:
        ser = serial.Serial(
            port     = CONFIG_PORT,
            baudrate = CONFIG_BAUD,
            bytesize = serial.EIGHTBITS,
            parity   = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            timeout  = 0.5,
            xonxoff  = False,
            rtscts   = False,
        )
    except serial.SerialException as e:
        print(f"  ERROR opening {CONFIG_PORT}: {e}")
        sys.exit(1)

    # Attempt hardware reset so DSS gets a clean boot
    reset_and_boot(ser)

    # Probe: discover all commands the firmware knows
    probe_firmware(ser)

    # Flush anything stale
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    sensor_started = False
    last_error     = None

    for i, cmd in enumerate(commands):
        print(f"\n  [{i:02d}] >>> {cmd}")

        # Extra hold before sensorStart: let DSS finish processing all
        # forwarded configs and send CONFIG_DONE to MSS.
        if cmd.strip().lower().startswith("sensorstart"):
            print("  [INFO] Holding 2s before sensorStart for DSS CONFIG_DONE...")
            time.sleep(2.0)

        ser.write((cmd + "\r\n").encode("ascii"))
        ser.flush()

        time.sleep(CMD_DELAY_S)
        response = drain_response(ser, timeout=0.15)

        lines = [l.strip() for l in response.splitlines() if l.strip()]
        if lines:
            for l in lines:
                print(f"       <<< {l}")
        else:
            print(f"       <<< (no response)")

        # Check sensorStart outcome
        if cmd.strip().lower().startswith("sensorstart"):
            full_resp = response.strip()
            if "done" in full_resp.lower():
                sensor_started = True
                print("\n  [OK] sensorStart succeeded!")
            else:
                last_error = full_resp
                print(f"\n  [FAIL] sensorStart failed: {full_resp!r}")
                # Query demo status to get internal state
                time.sleep(0.1)
                ser.write(b"queryDemoStatus\r\n")
                ser.flush()
                time.sleep(0.3)
                status_resp = drain_response(ser, timeout=0.3)
                if status_resp.strip():
                    print("\n  [queryDemoStatus] →")
                    for l in status_resp.splitlines():
                        if l.strip():
                            print(f"    {l}")

    ser.close()
    return sensor_started, last_error


# ---------------------------------------------------------------------------
# Step 4a: Listen on data port and print raw hex
# ---------------------------------------------------------------------------
def listen_data_port():
    print()
    print("=" * 60)
    print(f"STEP 4 — Listen on {DATA_PORT} @ {DATA_BAUD} baud for {LISTEN_S}s")
    print("=" * 60)

    try:
        data_ser = serial.Serial(
            port     = DATA_PORT,
            baudrate = DATA_BAUD,
            bytesize = serial.EIGHTBITS,
            parity   = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            timeout  = 0.5,
            xonxoff  = False,
            rtscts   = False,
        )
    except serial.SerialException as e:
        print(f"  ERROR opening {DATA_PORT}: {e}")
        return

    data_ser.reset_input_buffer()

    total_bytes = 0
    magic       = bytes([0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07])
    magic_found = False
    deadline    = time.time() + LISTEN_S

    print(f"  Listening... (magic word = {magic.hex()})")
    raw_buf = bytearray()

    while time.time() < deadline:
        chunk = data_ser.read(512)
        if not chunk:
            continue
        raw_buf.extend(chunk)
        total_bytes += len(chunk)

        if not magic_found and magic in raw_buf:
            magic_found = True
            idx = raw_buf.index(magic)
            print(f"  [MAGIC FOUND] at offset {idx} into stream")

    data_ser.close()

    print(f"\n  Total bytes received: {total_bytes}")
    if total_bytes == 0:
        print("  WARNING: zero bytes on data port — sensor may not have started")
        print("           or data port baud rate mismatch")
    else:
        # Print first 256 bytes as hex dump
        display = bytes(raw_buf[:256])
        print(f"\n  First {len(display)} bytes (hex dump):")
        for row in range(0, len(display), 16):
            chunk = display[row:row+16]
            hex_part  = " ".join(f"{b:02x}" for b in chunk)
            asc_part  = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
            print(f"    {row:04x}  {hex_part:<47}  {asc_part}")

        if magic_found:
            print("\n  [PASS] Magic word detected — binary frame data is flowing!")
        else:
            print("\n  [WARN] No magic word seen yet in this window (but bytes received)")


# ---------------------------------------------------------------------------
# Step 4b: Diagnose sensorStart failure
# ---------------------------------------------------------------------------
def diagnose_failure(error_msg):
    print()
    print("=" * 60)
    print("DIAGNOSIS — sensorStart failed")
    print("=" * 60)
    print(f"  Error: {error_msg!r}")
    print()
    print("  Common causes of 'Full configuration must be provided Error -1':")
    print()
    print("  1. A required command was missing or rejected before sensorStart.")
    print("     -> Check every response above for anything other than 'Done'.")
    print()
    print("  2. CQRxSatMonitor txSelMask doesn't match enabled TX antennas.")
    print("     -> channelCfg has txChannelEn=7 (TX1+TX2+TX3).")
    print("     -> CQRxSatMonitor txSelMask=3 only covers TX1+TX2.")
    print("     FIX: Change 'CQRxSatMonitor 0 3 5 121 0'")
    print("          to   'CQRxSatMonitor 0 7 5 121 0'")
    print()
    print("  3. sensorStop / flushCfg failed silently (radar was already stopped).")
    print("     -> Harmless for sensorStop but verify flushCfg got 'Done'.")
    print()
    print("  4. cfarCfg threshold as float (15.0/12.0) — some SDK builds want int.")
    print("     FIX: Use '15' and '12' instead of '15.0' and '12.0'.")
    print()
    print("  5. Config sent to wrong port (data port instead of config port).")
    print("     -> Confirmed config is going to", CONFIG_PORT)
    print()
    print("  RECOMMENDED FIX:")
    print("    Edit radar_profile.cfg:")
    print("      CQRxSatMonitor 0 7 5 121 0   (was: 0 3 5 121 0)")
    print("      cfarCfg -1 0 2 8 4 3 0 15 0  (was: 15.0)")
    print("      cfarCfg -1 1 0 4 2 3 1 12 0  (was: 12.0)")
    print()
    print("  Also: radar_parser_node uses /dev/radar_data by default.")
    print("        Install udev rules or pass --ros-args -p data_port:=/dev/ttyUSBx")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    print()
    print("╔══════════════════════════════════════════════════════════╗")
    print("║         IWR6843ISK Radar Diagnostic Script               ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print()

    check_ports()
    commands = load_config()
    sensor_started, last_error = send_config(commands)

    if sensor_started:
        listen_data_port()
    else:
        diagnose_failure(last_error or "(no response captured)")

    print()
    print("=" * 60)
    print("Diagnostic complete.")
    print("=" * 60)


if __name__ == "__main__":
    main()
