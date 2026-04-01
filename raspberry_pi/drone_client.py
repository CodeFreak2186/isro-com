"""
================================================================================
  Drone Client — Raspberry Pi + Pixhawk (pymavlink)
================================================================================
  Reads REAL telemetry from a Pixhawk via MAVLink and POSTs it to the server.

  CONNECTION OPTIONS (set CONNECT_STRING below):
  ─────────────────────────────────────────────
  A) Direct serial (Pixhawk → Pi UART):
       CONNECT_STRING = "serial:/dev/ttyAMA0:57600"
       (use /dev/ttyUSB0 if connected via USB)

  B) Via MAVProxy (recommended — MAVProxy runs as middleware):
       First run on Pi terminal:
         mavproxy.py --master=/dev/ttyAMA0 --baudrate=57600 --out=udp:127.0.0.1:14550
       Then set:
         CONNECT_STRING = "udp:127.0.0.1:14550"

  C) TCP via MAVProxy:
       CONNECT_STRING = "tcp:127.0.0.1:5760"

  Install dependencies:
       pip install -r requirements.txt

  MAVLink messages consumed:
       HEARTBEAT           → armed status, flight mode
       SYS_STATUS          → battery voltage & current
       ATTITUDE            → roll, pitch, yaw
       LOCAL_POSITION_NED  → pos x/y/z, vel x/y/z
       VFR_HUD             → altitude, climb rate, throttle, heading
       RC_CHANNELS         → ch1-ch5, RSSI
       EKF_STATUS_REPORT   → EKF flags and variances
       OPTICAL_FLOW        → flow x/y, compensated flow, quality
       DISTANCE_SENSOR     → rangefinder distance
================================================================================
"""

import os
import time
import math
import logging
import requests
from pymavlink import mavutil

# ---------------------------------------------------------------------------
# ★  CONFIGURE THESE  ★
# ---------------------------------------------------------------------------
SERVER_URL         = "https://isro-com.onrender.com"   # your Render URL or local IP
CONNECT_STRING     = "serial:/dev/ttyACM0:115200"     # Pixhawk via USB-serial (ACM)
BAUD_RATE          = 115200                            # ttyACM0 typically runs at 115200

TELEMETRY_INTERVAL = 0.2    # seconds between POSTs to server (5 Hz)
POLL_INTERVAL      = 4.0    # seconds between image polls
REQUEST_TIMEOUT    = 5      # HTTP timeout in seconds
STREAM_RATE_HZ     = 10     # MAVLink message request rate

DOWNLOAD_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "downloads")
os.makedirs(DOWNLOAD_DIR, exist_ok=True)

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# ArduCopter flight mode map  (custom_mode field from HEARTBEAT)
# ---------------------------------------------------------------------------
COPTER_MODES = {
    0:  "STABILIZE",  1:  "ACRO",       2:  "ALT_HOLD",
    3:  "AUTO",       4:  "GUIDED",     5:  "LOITER",
    6:  "RTL",        7:  "CIRCLE",     9:  "LAND",
    11: "DRIFT",      13: "SPORT",      14: "FLIP",
    15: "AUTOTUNE",   16: "POSHOLD",    17: "BRAKE",
    18: "THROW",      20: "GUIDED_NOGPS", 21: "SMART_RTL",
}

# ---------------------------------------------------------------------------
# Shared telemetry state dict — updated by message handlers, read by sender
# ---------------------------------------------------------------------------
state = {
    "status":   "DISARMED",
    "mode":     "STABILIZE",

    "ekf_flags":       "0x0000",
    "ekf_healthy":     "false",
    "ekf_active":      "",
    "ekf_vel_var":     0.0,
    "ekf_pos_var":     0.0,
    "ekf_compass_var": 0.0,

    "flow_x":       0.0,
    "flow_y":       0.0,
    "flow_comp_x":  0.0,
    "flow_comp_y":  0.0,
    "flow_quality": 0,

    "range_distance": 0.0,
    "range_voltage":  0.0,

    "pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0,
    "vel_x": 0.0, "vel_y": 0.0, "vel_z": 0.0,

    "alt":      0.0,
    "climb":    0.0,
    "throttle": 0,
    "heading":  0,

    "roll":  0.0,
    "pitch": 0.0,
    "yaw":   0.0,

    "rc_ch1": 0, "rc_ch2": 0, "rc_ch3": 0,
    "rc_ch4": 0, "rc_ch5": 0, "rc_rssi": 0,

    "battery_voltage": 0.0,
    "battery_current": 0.0,
}


# ===========================================================================
# MAVLink connection + stream setup
# ===========================================================================

def connect() -> mavutil.mavfile:
    """
    Connect to the Pixhawk and wait for a heartbeat.
    Returns the mavutil connection object.
    """
    log.info("Connecting to Pixhawk via: %s", CONNECT_STRING)

    # Parse serial vs UDP/TCP
    if CONNECT_STRING.startswith("serial:"):
        # Format: serial:/dev/ttyAMA0:57600
        _, path, baud = CONNECT_STRING.split(":")
        master = mavutil.mavlink_connection(path, baud=int(baud))
    else:
        master = mavutil.mavlink_connection(CONNECT_STRING)

    log.info("Waiting for heartbeat…")
    master.wait_heartbeat(timeout=30)
    log.info(
        "Heartbeat received — system %d, component %d",
        master.target_system,
        master.target_component,
    )
    return master


def request_streams(master: mavutil.mavfile):
    """
    Ask the Pixhawk to send the message streams we need at STREAM_RATE_HZ.
    Requests all major stream groups.
    """
    stream_ids = [
        mavutil.mavlink.MAV_DATA_STREAM_ALL,          # everything
        mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,  # IMU, GPS raw
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,  # SYS_STATUS, EKF
        mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,  # RC inputs
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,     # LOCAL_POSITION_NED
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,       # ATTITUDE
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,       # VFR_HUD
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,       # AHRS, EKF_STATUS
    ]
    for sid in stream_ids:
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            sid,
            STREAM_RATE_HZ,
            1,   # 1 = start streaming
        )
    log.info("Stream requests sent at %d Hz.", STREAM_RATE_HZ)


# ===========================================================================
# Per-message parsers — each updates the shared `state` dict
# ===========================================================================

def _parse_heartbeat(msg):
    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    state["status"] = "ARMED" if armed else "DISARMED"
    state["mode"]   = COPTER_MODES.get(msg.custom_mode, str(msg.custom_mode))


def _parse_sys_status(msg):
    # voltage in millivolts → volts; current in cA → amps
    state["battery_voltage"] = round(msg.voltage_battery / 1000.0, 3)
    state["battery_current"] = round(msg.current_battery / 100.0,  3)


def _parse_attitude(msg):
    state["roll"]  = round(math.degrees(msg.roll),  1)
    state["pitch"] = round(math.degrees(msg.pitch), 1)
    state["yaw"]   = round(math.degrees(msg.yaw),   1)


def _parse_local_position_ned(msg):
    state["pos_x"] = round(msg.x,  3)
    state["pos_y"] = round(msg.y,  3)
    state["pos_z"] = round(msg.z,  3)   # negative = above ground in NED
    state["vel_x"] = round(msg.vx, 3)
    state["vel_y"] = round(msg.vy, 3)
    state["vel_z"] = round(msg.vz, 3)


def _parse_vfr_hud(msg):
    state["alt"]      = round(msg.alt,      3)
    state["climb"]    = round(msg.climb,    3)
    state["throttle"] = int(msg.throttle)
    state["heading"]  = int(msg.heading)


def _parse_rc_channels(msg):
    # Raw RC values are in microseconds (1000–2000 µs)
    state["rc_ch1"]  = msg.chan1_raw
    state["rc_ch2"]  = msg.chan2_raw
    state["rc_ch3"]  = msg.chan3_raw
    state["rc_ch4"]  = msg.chan4_raw
    state["rc_ch5"]  = msg.chan5_raw
    state["rc_rssi"] = msg.rssi     # 0–254 (255 = unknown)


def _parse_ekf_status_report(msg):
    flags = msg.flags
    state["ekf_flags"]   = hex(flags)
    state["ekf_vel_var"] = round(msg.velocity_variance,       3)
    state["ekf_pos_var"] = round(msg.pos_horiz_variance,      3)
    state["ekf_compass_var"] = round(msg.compass_variance,    3)

    # Decode which EKF solutions are active from the flags bitmask
    # (ArduPilot EKF_STATUS_REPORT flag bits)
    EKF_BITS = {
        0x01: "ATTITUDE",
        0x02: "VEL_HORIZ",
        0x04: "VEL_VERT",
        0x08: "POS_HORIZ_REL",
        0x10: "POS_HORIZ_ABS",
        0x20: "POS_VERT_ABS",
        0x40: "POS_VERT_AGL",
        0x80: "CONST_POS_MODE",
        0x100: "PRED_POS_HORIZ_REL",
        0x200: "PRED_POS_HORIZ_ABS",
    }
    active = [name for bit, name in EKF_BITS.items() if flags & bit]
    healthy = bool(flags & 0x01)   # at minimum attitude must be valid

    state["ekf_active"]  = ",".join(active)
    state["ekf_healthy"] = "true" if healthy else "false"


def _parse_optical_flow(msg):
    state["flow_x"]      = round(msg.flow_x,       3)
    state["flow_y"]      = round(msg.flow_y,       3)
    state["flow_comp_x"] = round(msg.flow_comp_m_x, 3)
    state["flow_comp_y"] = round(msg.flow_comp_m_y, 3)
    state["flow_quality"] = int(msg.quality)


def _parse_distance_sensor(msg):
    # current_distance is in centimetres
    state["range_distance"] = round(msg.current_distance / 100.0, 3)
    state["range_voltage"]  = 0.0   # MAVLink DISTANCE_SENSOR has no voltage field


# Map MAVLink message type → parser function
MSG_HANDLERS = {
    "HEARTBEAT":            _parse_heartbeat,
    "SYS_STATUS":           _parse_sys_status,
    "ATTITUDE":             _parse_attitude,
    "LOCAL_POSITION_NED":   _parse_local_position_ned,
    "VFR_HUD":              _parse_vfr_hud,
    "RC_CHANNELS":          _parse_rc_channels,
    "EKF_STATUS_REPORT":    _parse_ekf_status_report,
    "OPTICAL_FLOW":         _parse_optical_flow,
    "DISTANCE_SENSOR":      _parse_distance_sensor,
}


# ===========================================================================
# Server communication
# ===========================================================================

def send_telemetry():
    """POST current state to the Flask server."""
    try:
        r = requests.post(
            f"{SERVER_URL}/upload",
            data=state,
            timeout=REQUEST_TIMEOUT,
        )
        r.raise_for_status()
    except requests.exceptions.ConnectionError:
        log.error("Cannot reach server at %s", SERVER_URL)
    except requests.exceptions.Timeout:
        log.warning("POST timed out.")
    except Exception as exc:
        log.error("Telemetry POST error: %s", exc)


_last_mtime = 0.0

def poll_and_download():
    """Check server for a new website-uploaded image and download it."""
    global _last_mtime
    try:
        r = requests.get(f"{SERVER_URL}/get-latest-image", timeout=REQUEST_TIMEOUT)
        if r.status_code == 404:
            return
        r.raise_for_status()
        d = r.json()
        if float(d.get("modified", 0)) <= _last_mtime:
            return
        img = requests.get(
            f"{SERVER_URL}{d['image_url']}",
            timeout=REQUEST_TIMEOUT,
            stream=True,
        )
        img.raise_for_status()
        save_path = os.path.join(DOWNLOAD_DIR, "received_image.jpg")
        with open(save_path, "wb") as f:
            for chunk in img.iter_content(8192):
                f.write(chunk)
        _last_mtime = float(d["modified"])
        log.info("Downloaded new image from website → %s", save_path)
    except Exception as exc:
        log.error("Image poll error: %s", exc)


# ===========================================================================
# Main loop
# ===========================================================================

def main():
    # ── Connect and request streams ───────────────────────────────────────
    while True:
        try:
            master = connect()
            request_streams(master)
            break
        except Exception as exc:
            log.error("Connection failed: %s — retrying in 5 s…", exc)
            time.sleep(5)

    log.info("MAVLink connected. Starting telemetry loop.")
    log.info("  Server      : %s", SERVER_URL)
    log.info("  Send rate   : %.1f Hz", 1 / TELEMETRY_INTERVAL)
    log.info("  Poll interval: %.1f s", POLL_INTERVAL)

    last_send = 0.0
    last_poll = 0.0

    while True:
        # ── Read all available MAVLink messages (non-blocking) ────────────
        try:
            while True:
                msg = master.recv_match(blocking=False)
                if msg is None:
                    break
                mtype = msg.get_type()
                handler = MSG_HANDLERS.get(mtype)
                if handler:
                    handler(msg)
        except Exception as exc:
            log.error("MAVLink read error: %s — attempting reconnect…", exc)
            try:
                master = connect()
                request_streams(master)
            except Exception:
                time.sleep(3)
            continue

        now = time.time()

        # ── POST telemetry to server at TELEMETRY_INTERVAL ────────────────
        if now - last_send >= TELEMETRY_INTERVAL:
            send_telemetry()
            last_send = now

        # ── Poll server for new website image ─────────────────────────────
        if now - last_poll >= POLL_INTERVAL:
            poll_and_download()
            last_poll = now

        time.sleep(0.01)   # 100 Hz loop — keeps CPU usage low


if __name__ == "__main__":
    main()
