"""
Drone Communication System — Flask Backend
Deployed on Render — APScheduler runs a background job every 15 seconds.
"""

import os
import json
import queue
import time
import logging
from datetime import datetime, timezone
from flask import Flask, request, jsonify, send_from_directory, render_template, Response, stream_with_context
from flask_cors import CORS
from apscheduler.schedulers.background import BackgroundScheduler
from apscheduler.triggers.interval import IntervalTrigger
import atexit

# ---------------------------------------------------------------------------
# App init
# ---------------------------------------------------------------------------
BASE_DIR      = os.path.dirname(os.path.abspath(__file__))
UPLOAD_FOLDER = os.path.join(BASE_DIR, "uploads")
WEBSITE_IMAGE = "new_image.jpg"
ALLOWED_EXTS  = {"jpg", "jpeg", "png", "gif", "webp"}

# Render uses an ephemeral filesystem — /tmp is safer than local dir
# but we keep local as fallback for local dev
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

app = Flask(__name__, template_folder=os.path.join(BASE_DIR, "templates"))
CORS(app)
app.config["UPLOAD_FOLDER"]      = UPLOAD_FOLDER
app.config["MAX_CONTENT_LENGTH"] = 16 * 1024 * 1024

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Telemetry store
# ---------------------------------------------------------------------------
telemetry = {
    "status":   "DISARMED",
    "mode":     "STABILIZE",
    "timestamp": None,

    # Connection health (set by the cron job)
    "drone_online":      False,
    "last_seen_seconds": None,   # seconds since last Pi upload

    # EKF
    "ekf_flags":       "0x0000",
    "ekf_healthy":     False,
    "ekf_active":      [],
    "ekf_vel_var":     0.0,
    "ekf_pos_var":     0.0,
    "ekf_compass_var": 0.0,

    # Optical flow
    "flow_x":       0.0,
    "flow_y":       0.0,
    "flow_comp_x":  0.0,
    "flow_comp_y":  0.0,
    "flow_quality": 0,

    # Rangefinder
    "range_distance": 0.0,
    "range_voltage":  0.0,

    # Local position
    "pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0,
    "vel_x": 0.0, "vel_y": 0.0, "vel_z": 0.0,

    # VFR HUD
    "alt": 0.0, "climb": 0.0, "throttle": 0, "heading": 0,

    # Attitude
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,

    # RC
    "rc_ch1": 0, "rc_ch2": 0, "rc_ch3": 0,
    "rc_ch4": 0, "rc_ch5": 0, "rc_rssi": 0,

    # Battery
    "battery_voltage": 0.0,
    "battery_current": 0.0,
}

# Track when we last received a Pi upload (UTC)
_last_upload_time:   datetime | None = None
_image_upload_time: float | None    = None   # set when website uploads an image
IMAGE_UPLOAD_DISPLAY_S = 10                  # show "uploading" indicator for this many seconds

# ---------------------------------------------------------------------------
# SSE client registry — one queue per connected browser tab
# ---------------------------------------------------------------------------
_sse_clients: list[queue.Queue] = []

def _push_to_sse(data: dict):
    """Broadcast latest telemetry to every open SSE connection."""
    payload = json.dumps(data)
    dead = []
    for q in _sse_clients:
        try:
            q.put_nowait(payload)
        except queue.Full:
            dead.append(q)
    for q in dead:
        _sse_clients.remove(q)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _f(v, default=0.0):
    try:    return float(v)
    except: return default

def _i(v, default=0):
    try:    return int(v)
    except: return default

def allowed(fn):
    return "." in fn and fn.rsplit(".", 1)[1].lower() in ALLOWED_EXTS


# ---------------------------------------------------------------------------
# ★  CRON JOB — runs every 15 seconds
# ---------------------------------------------------------------------------
STALE_THRESHOLD_S = 30   # mark drone offline if no upload for this many seconds

def cron_job():
    """
    Runs every 15 seconds.
    1. Checks how long ago the last telemetry upload arrived.
    2. Marks drone_online=False and status=TIMEOUT if data is stale.
    3. Logs a heartbeat so Render sees the process is alive.
    """
    global _last_upload_time

    now = datetime.now(timezone.utc)

    if _last_upload_time is None:
        # No upload received yet since server start
        telemetry["drone_online"]      = False
        telemetry["last_seen_seconds"] = None
        log.info("[CRON] Heartbeat — no telemetry received yet.")
        return

    delta = (now - _last_upload_time).total_seconds()
    telemetry["last_seen_seconds"] = round(delta, 1)

    if delta > STALE_THRESHOLD_S:
        telemetry["drone_online"] = False
        telemetry["status"]       = "TIMEOUT"
        log.warning(
            "[CRON] Drone OFFLINE — last upload %.1fs ago (threshold %ds).",
            delta, STALE_THRESHOLD_S,
        )
    else:
        telemetry["drone_online"] = True
        log.info(
            "[CRON] Heartbeat — drone online, last upload %.1fs ago, "
            "batt=%.2fV, alt=%.1fm, status=%s.",
            delta,
            telemetry["battery_voltage"],
            telemetry["alt"],
            telemetry["status"],
        )


# ---------------------------------------------------------------------------
# Start the APScheduler (runs in a daemon background thread)
# Guard against double-start when Werkzeug reloader spawns a child process.
# ---------------------------------------------------------------------------
if not os.environ.get("WERKZEUG_RUN_MAIN"):
    scheduler = BackgroundScheduler(daemon=True)
    scheduler.add_job(
        func=cron_job,
        trigger=IntervalTrigger(seconds=15),
        id="cron_15s",
        name="Drone telemetry watchdog",
        replace_existing=True,
        max_instances=1,
    )
    scheduler.start()
    log.info("[CRON] Scheduler started — job fires every 15 seconds.")
    # Shut down cleanly when the process exits
    atexit.register(lambda: scheduler.shutdown(wait=False))


# ===========================================================================
# Routes
# ===========================================================================

@app.route("/")
def index():
    return render_template("index.html")


# ── Receive telemetry from Pi ───────────────────────────────────────────────
@app.route("/upload", methods=["POST"])
def upload():
    global _last_upload_time
    b = request.get_json(force=True, silent=True) or request.form

    _last_upload_time = datetime.now(timezone.utc)

    telemetry.update({
        "status":   str(b.get("status",  "DISARMED")),
        "mode":     str(b.get("mode",    "STABILIZE")),
        "timestamp": _last_upload_time.isoformat(),
        "drone_online": True,
        "last_seen_seconds": 0,

        "ekf_flags":       str(b.get("ekf_flags", "0x0000")),
        "ekf_healthy":     str(b.get("ekf_healthy", "false")).lower() == "true",
        "ekf_active":      str(b.get("ekf_active","")).split(",") if b.get("ekf_active") else [],
        "ekf_vel_var":     _f(b.get("ekf_vel_var")),
        "ekf_pos_var":     _f(b.get("ekf_pos_var")),
        "ekf_compass_var": _f(b.get("ekf_compass_var")),

        "flow_x":       _f(b.get("flow_x")),
        "flow_y":       _f(b.get("flow_y")),
        "flow_comp_x":  _f(b.get("flow_comp_x")),
        "flow_comp_y":  _f(b.get("flow_comp_y")),
        "flow_quality": _i(b.get("flow_quality")),

        "range_distance": _f(b.get("range_distance")),
        "range_voltage":  _f(b.get("range_voltage")),

        "pos_x": _f(b.get("pos_x")), "pos_y": _f(b.get("pos_y")), "pos_z": _f(b.get("pos_z")),
        "vel_x": _f(b.get("vel_x")), "vel_y": _f(b.get("vel_y")), "vel_z": _f(b.get("vel_z")),

        "alt":      _f(b.get("alt")),
        "climb":    _f(b.get("climb")),
        "throttle": _i(b.get("throttle")),
        "heading":  _i(b.get("heading")),

        "roll":  _f(b.get("roll")),
        "pitch": _f(b.get("pitch")),
        "yaw":   _f(b.get("yaw")),

        "rc_ch1":  _i(b.get("rc_ch1")),  "rc_ch2":  _i(b.get("rc_ch2")),
        "rc_ch3":  _i(b.get("rc_ch3")),  "rc_ch4":  _i(b.get("rc_ch4")),
        "rc_ch5":  _i(b.get("rc_ch5")),  "rc_rssi": _i(b.get("rc_rssi")),

        "battery_voltage": _f(b.get("battery_voltage")),
        "battery_current": _f(b.get("battery_current")),
    })

    log.info("Telemetry — status=%s  alt=%.1fm  batt=%.2fV",
             telemetry["status"], telemetry["alt"], telemetry["battery_voltage"])

    # Push immediately to every open browser SSE connection
    _push_to_sse(telemetry)

    return jsonify({"ok": True}), 200


# ── SSE stream — browser subscribes here, receives data on every Pi upload ──
@app.route("/stream")
def stream():
    def event_generator():
        q = queue.Queue(maxsize=30)
        _sse_clients.append(q)
        # Send current state immediately so the page isn't blank on load
        yield f"data: {json.dumps(telemetry)}\n\n"
        try:
            while True:
                try:
                    payload = q.get(timeout=25)
                    yield f"data: {payload}\n\n"
                except queue.Empty:
                    # Send a keep-alive comment so the connection stays open
                    yield ": keep-alive\n\n"
        except GeneratorExit:
            pass
        finally:
            if q in _sse_clients:
                _sse_clients.remove(q)

    return Response(
        stream_with_context(event_generator()),
        mimetype="text/event-stream",
        headers={
            "Cache-Control":    "no-cache",
            "X-Accel-Buffering": "no",   # disables Nginx buffering on Render
            "Connection":       "keep-alive",
        },
    )


# ── Latest telemetry (ESP32 polls this) ────────────────────────────────────
@app.route("/data")
def data():
    # Attach image_uploading flag so ESP32 can show it on OLED
    img_up = (
        _image_upload_time is not None and
        (time.time() - _image_upload_time) < IMAGE_UPLOAD_DISPLAY_S
    )
    return jsonify({**telemetry, "image_uploading": img_up}), 200


# ── Website uploads image → Pi downloads it ────────────────────────────────
@app.route("/upload-from-website", methods=["POST"])
def upload_from_website():
    global _image_upload_time
    if "image" not in request.files:
        return jsonify({"error": "No image"}), 400
    f = request.files["image"]
    if not f or not f.filename or not allowed(f.filename):
        return jsonify({"error": "Invalid file"}), 400
    f.save(os.path.join(UPLOAD_FOLDER, WEBSITE_IMAGE))
    _image_upload_time = time.time()   # flag so ESP32 shows "uploading" notice
    log.info("Website image saved.")
    return jsonify({"ok": True, "image_url": f"/uploads/{WEBSITE_IMAGE}"}), 200


# ── Pi polls this for new website images ───────────────────────────────────
@app.route("/get-latest-image")
def get_latest_image():
    p = os.path.join(UPLOAD_FOLDER, WEBSITE_IMAGE)
    if not os.path.exists(p):
        return jsonify({"error": "No image"}), 404
    return jsonify({
        "image_url": f"/uploads/{WEBSITE_IMAGE}",
        "filename":  WEBSITE_IMAGE,
        "modified":  os.path.getmtime(p),
    }), 200


# ── Static uploads ──────────────────────────────────────────────────────────
@app.route("/uploads/<path:filename>")
def serve_upload(filename):
    return send_from_directory(UPLOAD_FOLDER, filename)


# ── Cron status endpoint (useful for monitoring) ───────────────────────────
@app.route("/cron-status")
def cron_status():
    """Returns the scheduler's next run time and last drone-seen info."""
    jobs = []
    for job in scheduler.get_jobs():
        jobs.append({
            "id":        job.id,
            "name":      job.name,
            "next_run":  str(job.next_run_time),
        })
    return jsonify({
        "scheduler_running": scheduler.running,
        "jobs": jobs,
        "drone_online":      telemetry["drone_online"],
        "last_seen_seconds": telemetry["last_seen_seconds"],
    }), 200


# ── Health (Render uses this for health checks) ────────────────────────────
@app.route("/health")
def health():
    return jsonify({
        "status":       "ok",
        "drone_online": telemetry["drone_online"],
        "uptime_check": datetime.now(timezone.utc).isoformat(),
    }), 200


# ===========================================================================
if __name__ == "__main__":
    log.info("Server → http://0.0.0.0:5000")
    app.run(host="0.0.0.0", port=5000, debug=True)
