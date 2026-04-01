"""
Drone Client — Raspberry Pi
Simulates all sensor data and POSTs it to the Flask server every few seconds.
Replace simulation functions with real MAVLink / sensor reads when ready.
"""

import os, time, math, random, logging, requests

SERVER_URL         = "http://<YOUR_SERVER_IP>:5000"
TELEMETRY_INTERVAL = 0.2   # 5 Hz  (same as real ArduPilot output)
POLL_INTERVAL      = 4.0
DOWNLOAD_DIR       = os.path.join(os.path.dirname(os.path.abspath(__file__)), "downloads")
REQUEST_TIMEOUT    = 5

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger(__name__)
os.makedirs(DOWNLOAD_DIR, exist_ok=True)

# ── Simulation state ──────────────────────────────────────────────────────────
t0 = time.time()

def sim() -> dict:
    t = time.time() - t0

    alt     = max(0.0, 1.06 + 0.3 * math.sin(t * 0.4))
    climb   = 0.3 * 0.4 * math.cos(t * 0.4)
    heading = int((191 + t * 3) % 360)
    throttle= max(0, min(100, int(30 + 20 * math.sin(t * 0.2))))

    roll    = round(-0.3 + 2.0 * math.sin(t * 0.7),  1)
    pitch   = round( 0.5 + 1.5 * math.cos(t * 0.5),  1)
    yaw     = round(-168.8 + heading - 191,            1)

    pos_x   = round(-0.002 + 0.01 * math.sin(t * 0.3), 3)
    pos_y   = round(-0.000 + 0.01 * math.cos(t * 0.3), 3)
    pos_z   = round(-alt,                               3)
    vel_x   = round(0.001 + 0.005 * math.sin(t),       3)
    vel_y   = round(0.001 + 0.005 * math.cos(t),       3)
    vel_z   = round(-climb,                             3)

    flow_q  = int(148 + 10 * math.sin(t * 2))
    flow_x  = round(0.01 * math.sin(t * 3), 3)
    flow_y  = round(0.01 * math.cos(t * 3), 3)
    fc_x    = round(-0.001 + 0.002 * math.sin(t), 3)
    fc_y    = round( 0.002 + 0.002 * math.cos(t), 3)

    rng     = round(0.23 + 0.01 * math.sin(t * 2), 3)

    batt_v  = round(max(10.5, 12.6 - t * 0.002), 3)
    batt_a  = round(0.07 + 0.03 * abs(math.sin(t * 0.5)), 3)

    rc_center = 1500
    rc_ch1  = rc_center + int(50 * math.sin(t * 0.3))
    rc_ch2  = rc_center + int(50 * math.cos(t * 0.3))
    rc_ch3  = 1000 + throttle * 10
    rc_ch4  = rc_center + int(30 * math.sin(t * 0.2))
    rc_ch5  = 1800   # mode switch

    ekf_flags  = "0x016F"
    ekf_healthy= True
    ekf_active = "ATTITUDE,VEL_HORIZ,VEL_VERT,POS_REL,POS_VERT,POS_AGL"

    status = "ARMED" if throttle > 15 else "DISARMED"
    modes  = ["STABILIZE","LOITER","ALT_HOLD","POSHOLD"]
    mode   = modes[int(t / 20) % len(modes)]

    return {
        "status": status, "mode": mode,
        "ekf_flags": ekf_flags, "ekf_healthy": str(ekf_healthy).lower(),
        "ekf_active": ekf_active,
        "ekf_vel_var": round(random.uniform(0.0, 0.005), 3),
        "ekf_pos_var": round(random.uniform(0.0, 0.005), 3),
        "ekf_compass_var": round(0.002 + random.uniform(0, 0.001), 3),
        "flow_x": flow_x, "flow_y": flow_y,
        "flow_comp_x": fc_x, "flow_comp_y": fc_y,
        "flow_quality": flow_q,
        "range_distance": rng, "range_voltage": 0.0,
        "pos_x": pos_x, "pos_y": pos_y, "pos_z": pos_z,
        "vel_x": vel_x, "vel_y": vel_y, "vel_z": vel_z,
        "alt": round(alt, 3), "climb": round(climb, 3),
        "throttle": throttle, "heading": heading,
        "roll": roll, "pitch": pitch, "yaw": yaw,
        "rc_ch1": rc_ch1, "rc_ch2": rc_ch2, "rc_ch3": rc_ch3,
        "rc_ch4": rc_ch4, "rc_ch5": rc_ch5, "rc_rssi": 255,
        "battery_voltage": batt_v, "battery_current": batt_a,
    }


# ── Upload telemetry ──────────────────────────────────────────────────────────
def send_telemetry():
    data = sim()
    try:
        r = requests.post(f"{SERVER_URL}/upload", data=data, timeout=REQUEST_TIMEOUT)
        r.raise_for_status()
    except requests.exceptions.ConnectionError:
        log.error("Cannot reach server at %s", SERVER_URL)
    except Exception as exc:
        log.error("Telemetry error: %s", exc)


# ── Poll + download website image ─────────────────────────────────────────────
_last_mtime = 0.0
def poll_and_download():
    global _last_mtime
    try:
        r = requests.get(f"{SERVER_URL}/get-latest-image", timeout=REQUEST_TIMEOUT)
        if r.status_code == 404: return
        r.raise_for_status()
        d = r.json()
        if float(d.get("modified", 0)) <= _last_mtime: return
        img = requests.get(f"{SERVER_URL}{d['image_url']}", timeout=REQUEST_TIMEOUT, stream=True)
        img.raise_for_status()
        with open(os.path.join(DOWNLOAD_DIR, "received_image.jpg"), "wb") as f:
            for chunk in img.iter_content(8192): f.write(chunk)
        _last_mtime = float(d["modified"])
        log.info("Downloaded new image from website.")
    except Exception as exc:
        log.error("Poll error: %s", exc)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    log.info("Drone client starting → %s", SERVER_URL)
    last_telem = 0.0
    last_poll  = 0.0
    while True:
        now = time.time()
        if now - last_telem >= TELEMETRY_INTERVAL:
            send_telemetry()
            last_telem = now
        if now - last_poll >= POLL_INTERVAL:
            poll_and_download()
            last_poll = now
        time.sleep(0.05)

if __name__ == "__main__":
    main()
