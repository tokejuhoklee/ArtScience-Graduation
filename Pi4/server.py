#!/usr/bin/env python3
"""
Whip Installation - Pi4 Controller
- Bridges browser UI to Pico over serial
- Runs sequences autonomously (survives browser disconnect)
- WebSocket pushes live Pico serial output to browser
- Serves the single-page UI

Deploy: scp server.py pi@<pi4-ip>:~/whip/
Run:    python3 server.py
Auto:   sudo systemctl enable whip (see whip.service)
"""

import asyncio
import json
import math
import os
import serial
import serial.tools.list_ports
import time
import glob
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
from urllib.parse import urlparse, parse_qs
import threading
import websockets
import websockets.legacy.server

try:
    import cv2
    import numpy as np
    CAMERA_AVAILABLE = True
except ImportError:
    cv2 = None
    np = None
    CAMERA_AVAILABLE = False
    print("opencv-python-headless not installed — camera disabled")

try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except Exception:
    PICAMERA2_AVAILABLE = False

# ── Config ────────────────────────────────────────────────────────────────────
HTTP_PORT       = 8080
WS_PORT         = 8081
BAUD            = 115200
SEQUENCES_DIR   = Path(__file__).parent / "sequences"
STATIC_DIR      = Path(__file__).parent / "static"
MOBILE_HTML     = Path(__file__).parent / "mobile.html"
CHAOS_PRESETS   = Path(__file__).parent / "chaos_presets.json"  # presets + timeline
SEQUENCES_DIR.mkdir(exist_ok=True)

# Name of a sequence in SEQUENCES_DIR to run automatically on boot (loop mode).
# Set to "" to disable. Create the sequence from the UI and save it to the Pi.
AUTOSTART_SEQ   = ""

PULSES_PER_REV = 400
GEAR_RATIO     = 5.0
RAMP_STEPS     = 150

def find_pico_port():
    candidates = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"]
    for p in candidates:
        if os.path.exists(p):
            return p
    for p in serial.tools.list_ports.comports():
        if "USB" in (p.description or "") or "ACM" in p.device:
            return p.device
    return None

# ── Threaded HTTP server ──────────────────────────────────────────────────────
class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Each request (including the MJPEG stream) runs in its own thread."""
    daemon_threads = True

# ── Camera Manager ────────────────────────────────────────────────────────────
class CameraManager:
    def __init__(self):
        self._proc   = None   # rpicam-vid subprocess
        self._cap    = None   # OpenCV VideoCapture (USB fallback)
        self._picam  = None   # picamera2 instance
        self._frame  = None   # latest JPEG bytes
        self._lock   = threading.Lock()
        self.running = False
        self._use_subprocess = False

    def start(self, index=0, width=640, height=480, fps=20):
        if PICAMERA2_AVAILABLE:
            self._picam = Picamera2()
            cfg = self._picam.create_preview_configuration(
                main={"size": (width, height), "format": "RGB888"}
            )
            self._picam.configure(cfg)
            self._picam.start()
            self._use_subprocess = False
            print(f"Pi Camera started via picamera2 ({width}×{height})")
        else:
            # Spawn rpicam-vid and read its raw MJPEG stdout
            import subprocess, shutil
            if shutil.which("rpicam-vid"):
                self._proc = subprocess.Popen(
                    ["rpicam-vid", "-t", "0", "--inline",
                     "--width", str(width), "--height", str(height),
                     "--framerate", str(fps), "--codec", "mjpeg",
                     "--hflip", "--vflip",
                     "--nopreview", "-o", "-"],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
                )
                self._use_subprocess = True
                print(f"Pi Camera started via rpicam-vid ({width}×{height} @ {fps}fps)")
            elif CAMERA_AVAILABLE:
                self._cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
                if not self._cap.isOpened():
                    print("No camera found — stream disabled")
                    return
                self._use_subprocess = False
                print(f"USB camera started ({width}×{height})")
            else:
                print("No camera backend available — stream disabled")
                return
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        if self._use_subprocess:
            # Parse JPEG frames from the raw MJPEG stream by finding SOI/EOI markers
            buf = b""
            while self.running:
                try:
                    chunk = self._proc.stdout.read(4096)
                    if not chunk:
                        break
                    buf += chunk
                    start = buf.find(b"\xff\xd8")
                    end   = buf.find(b"\xff\xd9")
                    if start != -1 and end != -1 and end > start:
                        frame = buf[start:end + 2]
                        with self._lock:
                            self._frame = frame
                        buf = buf[end + 2:]
                        if safety.armed and cv2 is not None and np is not None:
                            gray = cv2.imdecode(np.frombuffer(frame, np.uint8),
                                                cv2.IMREAD_GRAYSCALE)
                            if gray is not None:
                                safety.process(gray)
                except Exception as e:
                    print(f"Camera read error: {e}")
                    break
        else:
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, 70]
            while self.running:
                try:
                    if self._picam is not None:
                        rgb = self._picam.capture_array()
                        frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    else:
                        ret, frame = self._cap.read()
                        if not ret:
                            time.sleep(0.05)
                            continue
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                    if safety.armed:
                        safety.process(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                    _, buf = cv2.imencode(".jpg", frame, encode_params)
                    with self._lock:
                        self._frame = buf.tobytes()
                except Exception as e:
                    print(f"Camera capture error: {e}")
                    time.sleep(1)
                time.sleep(1 / 20)

    def stop(self):
        self.running = False
        if self._proc:
            self._proc.kill()
            self._proc = None

    def get_frame(self):
        with self._lock:
            return self._frame

camera_mgr = CameraManager()

# ── Safety monitor (camera proximity cutoff) ──────────────────────────────────
# Watches a "danger zone" (one side of a horizontal trip line) on the raw,
# grayscale camera frame and stops the whip if anything intrudes. Static
# reference differencing (captured at arm time) — conservative: a stationary
# intruder stays detected (never absorbed). The whip's own arc is excluded via
# an optional ignore box. Fail-safe: stale frame or detector error -> trip.
SAFETY_CONFIG = Path(__file__).parent / "safety_config.json"

# ── Show log ──────────────────────────────────────────────────────────────────
# Privacy-clean activity journal: timestamps of start/stop transitions only
# (no images, no counts). Lets the user see active vs. empty time and spot
# overheating-length runs. Appends JSONL to show_log.jsonl.
SHOW_LOG = Path(__file__).parent / "show_log.jsonl"

class ShowLog:
    def __init__(self):
        self._lock = threading.Lock()
        self.reason = None         # set by the actor right before a start/stop
        self._run_started = None   # wall time the current run began

    def log(self, ev, **kw):
        rec = {"t": time.strftime("%Y-%m-%d %H:%M:%S"),
               "ts": round(time.time(), 1), "ev": ev, **kw}
        try:
            with self._lock:
                with SHOW_LOG.open("a") as f:
                    f.write(json.dumps(rec) + "\n")
        except Exception as e:
            print(f"Show log error: {e}")

    def tick(self, running):
        """Called from the broadcast loop — logs run/stop transitions with the
        reason the responsible code path set just beforehand."""
        if running and self._run_started is None:
            self._run_started = time.time()
            self.log("start", why=self.reason or "manual")
            self.reason = None
        elif not running and self._run_started is not None:
            dur = round(time.time() - self._run_started, 1)
            self._run_started = None
            self.log("stop", why=self.reason or "manual", ran_s=dur)
            self.reason = None

show_log = ShowLog()

class SafetyMonitor:
    STALE_S = 1.0   # no fresh frame this long while armed -> trip

    def __init__(self):
        self.armed         = False
        self.tripped       = False
        self.trip_reason   = ""
        self.trip_y        = 0.55    # trip line, fraction of frame height
        self.danger_below  = True    # danger = below the line (toward bottom)
        self.min_area_frac = 0.03    # foreground fraction of danger zone to trip
        self.diff_thresh   = 25      # per-pixel grayscale diff threshold
        self.exclude       = None    # whip ignore box [x0,y0,x1,y1] fractions, or None
        self.auto_resume   = True    # restart motion once the zone clears
        self.resume_delay  = 3.0     # zone must stay clear this long before resuming
        self.arm_on_boot   = True    # arm automatically when the server starts
        self.fg_frac       = 0.0     # current foreground fraction (for UI tuning)
        self.last_frame_t  = 0.0
        self._clear_since  = None    # when the zone first became clear (while tripped)
        self._trip_at      = 0.0     # when the last trip fired (settle gate)
        self._was_running  = False   # was the oscillator running at trip time
        self._resume_params= None    # params to restart with on auto-resume
        # Presence (attract) mode: start the stored timeline as soon as the
        # approach zone (far side of the trip line) differs from the armed
        # background — same static-reference scheme as the intrusion detector,
        # so a person standing still keeps counting as present. Park once the
        # whole frame has matched the background for idle_timeout.
        self.presence_enabled = False
        self.presence_thresh  = 0.02   # differing fraction that counts as presence
        self.idle_timeout     = 60.0   # park after this long with nobody in view (s)
        self.presence_state   = "off"
        self.upper_frac       = 0.0    # live background-diff in the approach zone
        self.room_frac        = 0.0    # live background-diff across the whole frame
        self.last_activity_t  = time.time()
        self._ref_full        = None   # full-frame background captured at arm time
        self._prev_small      = None   # previous frame, for micro-motion detection
        self._last_motion_t   = time.time()   # gates ghost recovery (see process)
        self._upper_hits      = 0      # consecutive frames of approach presence
        self._hold_start_until= 0.0    # refractory after parking
        self._not_running_since = None # engine-idle debounce (restart gaps)
        self._tl_resume_offset= 0.0    # continue the show here on next approach
        self.presence_hold    = False  # manual stop: don't auto-start while the
                                       # operator is in the room; clears once the
                                       # room empties (or presence is re-toggled)
        self._lock         = threading.Lock()
        self._load()

    def configure(self, d):
        with self._lock:
            if "trip_y" in d:        self.trip_y = max(0.05, min(0.95, float(d["trip_y"])))
            if "danger_below" in d:  self.danger_below = bool(d["danger_below"])
            if "min_area_frac" in d: self.min_area_frac = max(0.005, min(0.5, float(d["min_area_frac"])))
            if "exclude" in d:       self.exclude = d["exclude"]
            if "auto_resume" in d:   self.auto_resume = bool(d["auto_resume"])
            if "resume_delay" in d:  self.resume_delay = max(0.0, min(60.0, float(d["resume_delay"])))
            if "arm_on_boot" in d:   self.arm_on_boot = bool(d["arm_on_boot"])
            if "presence_enabled" in d:
                self.presence_enabled = bool(d["presence_enabled"])
                self.presence_hold = False   # toggling re-arms autonomy
            if "presence_thresh" in d:  self.presence_thresh = max(0.002, min(0.3, float(d["presence_thresh"])))
            if "idle_timeout" in d:     self.idle_timeout = max(5.0, min(900.0, float(d["idle_timeout"])))
            if any(k in d for k in ("trip_y", "danger_below", "exclude")):
                self._ref_full = None   # geometry changed -> recapture background
        self._save()

    def arm(self):
        with self._lock:
            self._ref_full = None
            self.tripped = False; self.trip_reason = ""
            self.armed = True; self.last_frame_t = time.time()

    def disarm(self):
        with self._lock:
            self.armed = False

    def rearm(self):
        with self._lock:
            self._ref_full = None
            self.tripped = False; self.trip_reason = ""
            self.last_frame_t = time.time()

    def _region(self, gray):
        h, w = gray.shape[:2]
        y = int(self.trip_y * h)
        if self.danger_below: reg, y0 = gray[y:, :].copy(), y
        else:                 reg, y0 = gray[:y, :].copy(), 0
        if self.exclude:
            ex0 = int(self.exclude[0]*w); ey0 = int(self.exclude[1]*h)
            ex1 = int(self.exclude[2]*w); ey1 = int(self.exclude[3]*h)
            reg[max(0, ey0-y0):max(0, ey1-y0), ex0:ex1] = 0  # blank whip arc
        return reg

    # Adaptive background: pixels that match the background keep blending
    # toward the current frame, so gradual lighting drift (weather, day/night)
    # never invalidates the reference. Pixels that differ (a person) are only
    # re-absorbed once the room has been MOTION-quiet for RECOVER_QUIET_S —
    # a watching audience micro-moves constantly, so it is never absorbed,
    # while a lighting-step ghost in an empty room heals in ~2 min.
    BG_ALPHA_MATCH   = 0.001     # matched pixels: τ ≈ 50 s at 20 fps
    BG_ALPHA_RECOVER = 0.0005    # differing pixels, motion-quiet room only
    RECOVER_QUIET_S  = 30.0      # no motion for this long before recovery runs
    MOTION_THR       = 15        # per-pixel frame-to-frame motion threshold

    def process(self, gray):
        if not self.armed or cv2 is None:
            return
        self.last_frame_t = time.time()
        try:
            small = cv2.GaussianBlur(cv2.resize(gray, (320, 240)), (21, 21), 0)

            with self._lock:
                if self._ref_full is None or self._ref_full.shape != small.shape:
                    self._ref_full = small.astype(np.float32)   # baseline capture
                    self._prev_small = small
                    return
                refarr = self._ref_full          # local handle: safe vs re-arm race
                thr, min_frac = self.diff_thresh, self.min_area_frac

            now = time.time()
            full_diff = cv2.absdiff(small, cv2.convertScaleAbs(refarr))
            # Danger mask keeps the strict threshold (false trips are costly);
            # presence gets half of it — still, low-contrast people were falling
            # below the strict one, which made activation look motion-based.
            fm_d = cv2.threshold(full_diff, thr, 255, cv2.THRESH_BINARY)[1]
            fm_p = cv2.threshold(full_diff, max(10, thr // 2), 255,
                                 cv2.THRESH_BINARY)[1]

            # Frame-to-frame motion: a watching audience shifts constantly even
            # when "standing still" — motion counts as activity too, and gates
            # the background recovery below.
            mo_room = mo_upper = 0.0
            if self._prev_small is not None and self._prev_small.shape == small.shape:
                mm = cv2.threshold(cv2.absdiff(small, self._prev_small),
                                   self.MOTION_THR, 255, cv2.THRESH_BINARY)[1]
                my = int(self.trip_y * mm.shape[0])
                m_up = mm[:my, :] if self.danger_below else mm[my:, :]
                mo_room  = cv2.countNonZero(mm) / max(1, mm.size)
                mo_upper = cv2.countNonZero(m_up) / max(1, m_up.size)
                if mo_room > 0.002:
                    self._last_motion_t = now
            self._prev_small = small

            # Presence = background difference OR motion, per zone
            fy = int(self.trip_y * fm_p.shape[0])
            upper = fm_p[:fy, :] if self.danger_below else fm_p[fy:, :]
            self.room_frac  = max(cv2.countNonZero(fm_p) / max(1, fm_p.size), mo_room)
            self.upper_frac = max(cv2.countNonZero(upper) / max(1, upper.size), mo_upper)
            if self.room_frac > self.presence_thresh:
                self.last_activity_t = now
            self._upper_hits = (self._upper_hits + 1
                                if self.upper_frac > self.presence_thresh else 0)

            # Background update: matched pixels always adapt (lighting drift);
            # differing pixels recover only in a motion-quiet room, so people
            # are never absorbed while they watch.
            fg_area = cv2.dilate(fm_p, None, iterations=2)
            cv2.accumulateWeighted(small, refarr, self.BG_ALPHA_MATCH,
                                   mask=cv2.bitwise_not(fg_area))
            if now - self._last_motion_t > self.RECOVER_QUIET_S:
                cv2.accumulateWeighted(small, refarr, self.BG_ALPHA_RECOVER,
                                       mask=fg_area)

            # Danger zone: crop of the strict mask (+ whip exclude box), dilated
            reg  = self._region(fm_d)
            mask = cv2.dilate(reg, None, iterations=2)
            self.fg_frac = cv2.countNonZero(mask) / max(1, reg.size)
            clear = self.fg_frac <= min_frac
            if not self.tripped:
                if not clear:
                    self._trip("intrusion")
            elif self.auto_resume:
                # tripped: resume once the zone has stayed clear long enough —
                # and no sooner than 8s after the trip. A trip DROPS the arm
                # mid-swing; the wake-up re-zero is only correct once it hangs
                # still, and 8s is the settle time boot-home has proven out
                # (5s left it swaying -> re-zeroed off-centre after pauses).
                if clear:
                    if self._clear_since is None:
                        self._clear_since = time.time()
                    elif (time.time() - self._clear_since >= self.resume_delay
                          and time.time() - self._trip_at >= 8.0):
                        self._resume()
                else:
                    self._clear_since = None
        except Exception as e:
            print(f"Safety process error: {e}")
            self._trip("detector error")

    def check_stale(self):
        if self.armed and not self.tripped and cv2 is not None:
            if time.time() - self.last_frame_t > self.STALE_S:
                self._trip("no camera frame")

    def presence_tick(self):
        """Attract-mode FSM (called from the broadcast loop). Start the stored
        timeline on approach motion; park the motor after idle_timeout of
        stillness. Requires the safety monitor to be armed (it supplies the
        camera analysis) and never starts while tripped."""
        if not (self.presence_enabled and self.armed and cv2 is not None):
            self.presence_state = "off" if not self.presence_enabled else "off (needs arm)"
            return
        if self.tripped:
            self.presence_state = "paused (intrusion)"
            return
        now = time.time()
        if osc_engine.running:
            self._not_running_since = None
            if now - self.last_activity_t > self.idle_timeout:
                print("Presence: room idle — parking")
                show_log.reason = "room empty"
                if osc_engine.tl_active:   # remember where the show was —
                    self._tl_resume_offset = osc_engine.tl_elapsed
                osc_engine.stop()          # — it continues there next approach
                serial_mgr.send("park")          # to neutral, then de-energize
                self._hold_start_until = now + 8.0   # let the park move finish
                self._upper_hits = 0
                self.presence_state = "idle (parked)"
            else:
                self.presence_state = "playing"
        else:
            if self.presence_hold:
                # Manually stopped with people (the operator) still in the room:
                # stay quiet. Autonomy resumes once the room has emptied.
                if now - self.last_activity_t > self.idle_timeout:
                    self.presence_hold = False
                self.presence_state = "held (manual stop)"
                return
            # Debounce: only consider starting once the engine has been idle
            # a full second — restart gaps must not read as "show ended".
            if self._not_running_since is None:
                self._not_running_since = now
            if now - self._not_running_since < 1.0:
                self.presence_state = "waiting"
                return
            self.presence_state = "waiting"
            if now < self._hold_start_until:
                return
            if self._upper_hits >= 3 and _event_loop is not None:
                bundle = load_show_bundle()
                if bundle.get('timeline'):
                    print(f"Presence: approach — resuming timeline at "
                          f"{self._tl_resume_offset:.1f}s")
                    show_log.reason = "approach"
                    self._upper_hits = 0
                    self.last_activity_t = now
                    osc_engine.start({'mode': 'timeline',
                                      'tl_offset': self._tl_resume_offset},
                                     _event_loop)
                    self._hold_start_until = now + 3.0   # one start, then patience
                    self._not_running_since = None
                    self.presence_state = "playing"

    def _trip(self, reason):
        self.tripped = True
        self.trip_reason = reason
        self._clear_since = None
        try:
            self._was_running  = osc_engine.running
            self._resume_params = dict(osc_engine.params) if osc_engine.params else None
            # A server-driven timeline pauses: freeze its position so the resume
            # continues from where it was interrupted, not from the beginning.
            if self._resume_params and self._resume_params.get('mode') == 'timeline':
                self._resume_params['tl_offset'] = osc_engine.tl_elapsed
            # De-energize on trip: the whip drops limp (compliant — safer near a
            # person, and it reads as intentional). The arm settles at gravity
            # rest, and the firmware re-zeros there on the next enable
            # (REST_RECAL_MS), so every trip now re-centres instead of drifting.
            show_log.reason = f"safety: {reason}"
            osc_engine.stop()
            serial_mgr.send("off")
            self._trip_at = time.time()
        except Exception as e:
            print(f"Safety trip stop error: {e}")
        print(f"!! SAFETY TRIP: {reason}")

    def _resume(self):
        self._clear_since = None
        if self._was_running and self._resume_params and _event_loop is not None:
            try:
                params = dict(self._resume_params)
                # Overlay motor settings adjusted during the pause (the browser
                # keeps pushing them while stopped), so the show resumes with
                # the current centre/limit/speed — not the trip-time snapshot.
                for k in ('center', 'speed', 'limit'):
                    if k in osc_engine.params:
                        params[k] = osc_engine.params[k]
                show_log.reason = "zone clear"
                self._upper_hits = 0     # presence must not double-start
                osc_engine.start(params, _event_loop)
                print("Safety: zone clear — motion resumed")
            except Exception as e:
                print(f"Safety resume error: {e}")
        self._was_running = False
        # Cleared AFTER the restart: while tripped the presence FSM is blocked,
        # so it can't fire a competing start in the gap (seen live in the log
        # as a start/stop/start churn within 0.3s).
        self.tripped = False
        self.trip_reason = ""

    def state(self):
        return {
            "armed": self.armed, "tripped": self.tripped, "reason": self.trip_reason,
            "trip_y": self.trip_y, "danger_below": self.danger_below,
            "min_area_frac": self.min_area_frac, "exclude": self.exclude,
            "auto_resume": self.auto_resume, "resume_delay": self.resume_delay,
            "arm_on_boot": self.arm_on_boot,
            "presence_enabled": self.presence_enabled,
            "presence_thresh": self.presence_thresh,
            "idle_timeout": self.idle_timeout,
            "presence_state": self.presence_state,
            "presence_hold": self.presence_hold,
            "upper": round(self.upper_frac, 4), "room": round(self.room_frac, 4),
            "fg": round(self.fg_frac, 4), "cv": cv2 is not None,
        }

    def _save(self):
        try:
            SAFETY_CONFIG.write_text(json.dumps({
                "trip_y": self.trip_y, "danger_below": self.danger_below,
                "min_area_frac": self.min_area_frac, "exclude": self.exclude,
                "auto_resume": self.auto_resume, "resume_delay": self.resume_delay,
                "arm_on_boot": self.arm_on_boot,
                "presence_enabled": self.presence_enabled,
                "presence_thresh": self.presence_thresh,
                "idle_timeout": self.idle_timeout,
            }, indent=2))
        except Exception as e:
            print(f"Safety config save error: {e}")

    def _load(self):
        try:
            if SAFETY_CONFIG.exists():
                d = json.loads(SAFETY_CONFIG.read_text())
                self.trip_y = d.get("trip_y", self.trip_y)
                self.danger_below = d.get("danger_below", self.danger_below)
                self.min_area_frac = d.get("min_area_frac", self.min_area_frac)
                self.exclude = d.get("exclude", self.exclude)
                self.auto_resume = d.get("auto_resume", self.auto_resume)
                self.resume_delay = d.get("resume_delay", self.resume_delay)
                self.arm_on_boot = d.get("arm_on_boot", self.arm_on_boot)
                self.presence_enabled = d.get("presence_enabled", self.presence_enabled)
                self.presence_thresh = d.get("presence_thresh", self.presence_thresh)
                self.idle_timeout = d.get("idle_timeout", self.idle_timeout)
        except Exception as e:
            print(f"Safety config load error: {e}")

safety = SafetyMonitor()

# ── Serial Manager ────────────────────────────────────────────────────────────
class SerialManager:
    def __init__(self):
        self.ser = None
        self.lock = threading.Lock()
        self.rx_lines = []
        self._buf = ""
        self.connected = False
        self.last_target_time = 0.0
        self.pending_wait_t0 = None  # set just before a movement command is sent

    def connect(self, port=None):
        port = port or find_pico_port()
        if not port:
            print("No Pico found - running in offline mode")
            return False
        try:
            with self.lock:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(port, BAUD, timeout=0.05)
            time.sleep(2)
            self.connected = True
            print(f"Connected to Pico on {port}")
            return True
        except Exception as e:
            print(f"Serial error: {e}")
            return False

    def send(self, cmd: str):
        with self.lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write((cmd.strip() + "\n").encode())
                except Exception as e:
                    print(f"Write error: {e}")
                    self.connected = False

    def flush_and_stop(self):
        """Discard all queued outgoing bytes, then send stop.
        Prevents a flood of 'cw'/'ccw' commands from blocking the stop."""
        with self.lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.reset_output_buffer()   # drop queued tx bytes
                    self.ser.write(b"stop\n")
                except Exception as e:
                    print(f"flush_and_stop error: {e}")
                    self.connected = False

    def read_available(self) -> list:
        lines = []
        with self.lock:
            if not (self.ser and self.ser.is_open):
                return lines
            try:
                while self.ser.in_waiting:
                    c = self.ser.read(1).decode("utf-8", errors="ignore")
                    if c in ("\n", "\r"):
                        line = self._buf.strip()
                        if line:
                            lines.append(line)
                            self.rx_lines.append(line)
                            if len(self.rx_lines) > 200:
                                self.rx_lines = self.rx_lines[-200:]
                            if "target reached" in line.lower() or "pend" in line.lower():
                                self.last_target_time = time.time()
                        self._buf = ""
                    else:
                        self._buf += c
            except Exception as e:
                print(f"Read error: {e}")
                self.connected = False
        return lines

serial_mgr = SerialManager()

# ── Sequence Engine ───────────────────────────────────────────────────────────
class SequenceEngine:
    def __init__(self):
        self.running = False
        self.loop_mode = False
        self.current_name = ""
        self.step_index = 0
        self.total_steps = 0
        self._stop = False
        self._task = None
        self._gen  = 0   # incremented on every start(); stale tasks self-abort

    def start(self, steps: list, name: str, loop: bool, event_loop):
        # Invalidate any in-flight task BEFORE touching serial
        self._gen += 1
        gen = self._gen
        self._stop = True
        if self._task and not self._task.done():
            self._task.cancel()
        serial_mgr.flush_and_stop()        # flush queued commands, then send stop
        serial_mgr.pending_wait_t0 = None
        time.sleep(0.15)                   # let cancellation + Pico stop propagate

        self._stop = False
        self.loop_mode = loop
        self.current_name = name
        self._task = asyncio.run_coroutine_threadsafe(
            self._run(steps, gen), event_loop
        )

    def stop(self):
        self._gen += 1
        self._stop = True
        if self._task:
            self._task.cancel()
        self.running = False
        serial_mgr.flush_and_stop()

    def _dead(self, gen: int) -> bool:
        return self._stop or self._gen != gen

    async def _run(self, steps: list, gen: int):
        self.running = True
        self.total_steps = len(steps)
        loop_start = next(
            (i + 1 for i, s in enumerate(steps) if s and s[0] == "loop_start"), 0
        )
        first = True
        try:
            while True:
                start = 0 if first else loop_start
                first = False
                for i in range(start, len(steps)):
                    if self._dead(gen):
                        return
                    step = steps[i]
                    self.step_index = i + 1
                    cmd = step[0] if len(step) > 0 else ""
                    val = step[1] if len(step) > 1 else None
                    await self._execute(cmd, val, gen)
                if not self.loop_mode:
                    break
        finally:
            self.running = False
            self.step_index = 0

    async def _execute(self, cmd: str, val, gen: int):
        if self._dead(gen):
            return

        if cmd == "wait":
            ms = int(val) if val is not None else 500
            await asyncio.sleep(ms / 1000.0)

        elif cmd == "wait_pend":
            t0 = serial_mgr.pending_wait_t0 if serial_mgr.pending_wait_t0 is not None \
                 else (time.time() - 0.1)
            serial_mgr.pending_wait_t0 = None
            deadline = t0 + 15.0
            while time.time() < deadline and not self._dead(gen):
                if serial_mgr.last_target_time > t0:
                    break
                await asyncio.sleep(0.02)

        elif cmd == "loop_start":
            pass

        elif cmd == "ramp_speed":
            if isinstance(val, dict):
                frm = int(val.get("from", 500))
                to  = int(val.get("to", 2000))
                n   = int(val.get("steps", 10))
                ms  = int(val.get("step_ms", 200))
                for i in range(n + 1):
                    if self._dead(gen):
                        return
                    spd = int(frm + (to - frm) * i / n)
                    serial_mgr.send(f"speed {spd}")
                    await asyncio.sleep(ms / 1000.0)

        elif cmd == "repeat":
            if isinstance(val, dict):
                times = int(val.get("times", 1))
                inner = val.get("steps", [])
                for _ in range(times):
                    for s in inner:
                        if self._dead(gen):
                            return
                        await self._execute(s[0], s[1] if len(s) > 1 else None, gen)

        else:
            _MOVE_CMDS = {"left", "right", "move", "offset", "cw", "ccw"}
            is_move = cmd in _MOVE_CMDS
            if is_move:
                serial_mgr.pending_wait_t0 = time.time()
            if val is not None:
                serial_mgr.send(f"{cmd} {val}")
            else:
                serial_mgr.send(cmd)
            await asyncio.sleep(0.02 if is_move else 0.01)
            # After the sleep a stale task must not continue — check generation
            if self._dead(gen):
                return

engine = SequenceEngine()

# ── Double-pendulum ODE (RK4) ─────────────────────────────────────────────────
class DoublePendulum:
    def __init__(self, L1, L2, m1, m2, th1_deg, th2_deg, w1=0.0, damping=0.0):
        self.L1=L1; self.L2=L2; self.m1=m1; self.m2=m2
        self.th1=math.radians(th1_deg); self.th2=math.radians(th2_deg)
        self.w1=w1; self.w2=0.0; self.damping=damping

    def _deriv(self, th1, th2, w1, w2):
        L1,L2,m1,m2,g = self.L1,self.L2,self.m1,self.m2,9.81
        D = 2*m1+m2-m2*math.cos(2*th1-2*th2)
        dw1 = (-g*(2*m1+m2)*math.sin(th1)
               -m2*g*math.sin(th1-2*th2)
               -2*math.sin(th1-th2)*m2*(w2**2*L2+w1**2*L1*math.cos(th1-th2))
               ) / (L1*D)
        dw2 = (2*math.sin(th1-th2)*(w1**2*L1*(m1+m2)
               +g*(m1+m2)*math.cos(th1)
               +w2**2*L2*m2*math.cos(th1-th2))
               ) / (L2*D)
        return w1, w2, dw1-self.damping*w1, dw2-self.damping*w2

    def step(self, dt):
        th1,th2,w1,w2 = self.th1,self.th2,self.w1,self.w2
        k1 = self._deriv(th1,th2,w1,w2)
        k2 = self._deriv(th1+k1[0]*dt/2,th2+k1[1]*dt/2,w1+k1[2]*dt/2,w2+k1[3]*dt/2)
        k3 = self._deriv(th1+k2[0]*dt/2,th2+k2[1]*dt/2,w1+k2[2]*dt/2,w2+k2[3]*dt/2)
        k4 = self._deriv(th1+k3[0]*dt,  th2+k3[1]*dt,  w1+k3[2]*dt,  w2+k3[3]*dt)
        self.th1 += dt/6*(k1[0]+2*k2[0]+2*k3[0]+k4[0])
        self.th2 += dt/6*(k1[1]+2*k2[1]+2*k3[1]+k4[1])
        self.w1  += dt/6*(k1[2]+2*k2[2]+2*k3[2]+k4[2])
        self.w2  += dt/6*(k1[3]+2*k2[3]+2*k3[3]+k4[3])

# ── Server-side timeline playback ─────────────────────────────────────────────
# Python twin of the client's easing/interpolation, so the stored show
# (chaos_presets.json: presets + timeline + loop + motor) can play without a
# browser — needed for presence mode and pause/continue across safety trips.
_EASINGS = {
    'linear':      lambda t: t,
    'ease-in':     lambda t: t*t,
    'ease-out':    lambda t: t*(2-t),
    'ease-in-out': lambda t: 2*t*t if t < 0.5 else -1+(4-2*t)*t,
}

def _ease(t, name):
    t = max(0.0, min(1.0, t))
    return _EASINGS.get(name, _EASINGS['linear'])(t)

def load_show_bundle():
    try:
        if CHAOS_PRESETS.exists():
            return json.loads(CHAOS_PRESETS.read_text())
    except Exception as e:
        print(f"Show bundle load error: {e}")
    return {}

_TL_DEFAULTS = {'s1a':80.0,'s1f':0.4,'s1p':0.0,'s2a':25.0,'s2f':1.1,'s2p':90.0,
                'cL1':1.0,'cL2':0.8,'cm1':1.0,'cm2':0.8,'cth1':130.0,'cth2':95.0,
                'cw1':0.0,'cdamp':0.01,'cscale':0.58,'csim':1.0}

def _tl_val(pr, key):
    try:
        return float(pr.get(key))
    except (TypeError, ValueError):
        return _TL_DEFAULTS[key]

def _tl_build(cur, nxt, q):
    ip = lambda k: _tl_val(cur, k) + (_tl_val(nxt, k) - _tl_val(cur, k)) * q
    if cur.get('mode', 'chaos') == 'sine':
        return {'mode': 'sine',
                'osc1': {'amp': ip('s1a'), 'freq': ip('s1f'), 'phase': ip('s1p')},
                'osc2': {'amp': ip('s2a'), 'freq': ip('s2f'), 'phase': ip('s2p')}}
    return {'mode': 'chaos',
            'L1': ip('cL1'), 'L2': ip('cL2'), 'm1': ip('cm1'), 'm2': ip('cm2'),
            'th1': ip('cth1'), 'th2': ip('cth2'), 'w1': ip('cw1'),
            'damping': ip('cdamp'), 'scale': ip('cscale'), 'simSpeed': ip('csim')}

def timeline_engine_params(presets, timeline, elapsed):
    """Interpolated engine params at `elapsed` s into the timeline.
    Returns (params, segment_index) or (None, -1)."""
    if not timeline:
        return None, -1
    acc = 0.0
    for i, item in enumerate(timeline):
        dur = float(item.get('duration', 5) or 5)
        if elapsed < acc + dur:
            q   = _ease((elapsed - acc) / dur if dur > 0 else 1.0,
                        item.get('easing', 'linear'))
            cur = presets.get(item.get('presetName'))
            nxt = presets.get(timeline[i+1].get('presetName')) if i+1 < len(timeline) else cur
            if not cur:
                return None, -1
            return _tl_build(cur, nxt or cur, q), i
        acc += dur
    cur = presets.get(timeline[-1].get('presetName'))   # past the end: hold last
    if not cur:
        return None, -1
    return _tl_build(cur, cur, 0.0), len(timeline) - 1

# ── Oscillator engine ─────────────────────────────────────────────────────────
class OscillatorEngine:
    RATE      = 25       # Hz — move commands sent to Pico
    SIM_DT    = 0.003    # chaos ODE integration step (s)
    MAX_DEG   = 180
    MIN_SPS   = 50       # Pico's minSpeed floor
    MAX_SPS   = 3500     # Pico's maxSpeed ceiling
    SPS_QUANT = 25       # round commanded speed to this grid (gates resends)

    def __init__(self):
        self.running      = False
        self.current_angle= 0.0
        self.params       = {}
        self._gen         = 0
        self._task        = None
        self._reseed      = False   # recreate the chaos sim from current ICs
        self.live_motor   = {}      # last live-pushed center/speed/limit
        # Server-driven timeline playback state (mode == 'timeline')
        self.tl_active    = False
        self.tl_idx       = -1
        self.tl_name      = ""
        self.tl_count     = 0
        self.tl_elapsed   = 0.0     # frozen at stop -> safety resume continues here

    def start(self, params: dict, event_loop):
        self._gen += 1
        gen = self._gen
        # Held true across the restart gap (cancel -> new task), so watchers
        # (presence FSM, activity log) don't see a phantom stop and react to it
        self.running = True
        if self._task and not self._task.done():
            self._task.cancel()
        engine.stop()              # silence sequence engine if running
        serial_mgr.flush_and_stop()
        time.sleep(0.12)
        self._reseed = False
        self.params = params.copy()
        # Presence/resume starts pass only mode+offset — inherit the last
        # live-pushed motor values so the engine's centre always matches the UI
        for k, v in self.live_motor.items():
            self.params.setdefault(k, v)
        self._task = asyncio.run_coroutine_threadsafe(
            self._run(params, gen), event_loop
        )

    def update(self, params: dict):
        if params.get("reseed"):
            self._reseed = True   # chaos segment (re)started — re-seed ICs
        for k in ('center', 'speed', 'limit'):
            if k in params:
                self.live_motor[k] = params[k]
        self.params.update(params)

    def stop(self):
        self._gen += 1
        if self._task and not self._task.done():
            self._task.cancel()
        serial_mgr.flush_and_stop()
        serial_mgr.send("quietoff")   # restore verbose logging for manual control
        self.running = False
        self.tl_active = False        # tl_elapsed keeps its value for resume

    async def _run(self, initial_params, gen):
        self.running = True
        DT = 1.0 / self.RATE

        init_mode = initial_params.get('mode', 'sine')

        # Timeline mode: the server plays the stored show itself (presets +
        # timeline + loop + motor from chaos_presets.json) — no browser needed.
        # tl_offset lets a safety trip pause and continue where it left off.
        tl = None
        if init_mode == 'timeline':
            b  = load_show_bundle()
            tl = {
                'presets':  b.get('presets', {}) or {},
                'timeline': b.get('timeline', []) or [],
                'loop':     bool(b.get('loop', False)),
                'motor':    b.get('motor', {}) or {},
                'offset':   float(initial_params.get('tl_offset', 0.0) or 0.0),
                'prev_idx': -1, 'prev_e': -1.0,
            }
            tl['total'] = sum(float(it.get('duration', 5) or 5) for it in tl['timeline'])
            if not tl['timeline'] or tl['total'] <= 0:
                print("Timeline: nothing stored to play")
                self.running = False
                return
            self.tl_active = True
            self.tl_count  = len(tl['timeline'])

        # Motor setup: set hard travel limit, disable ramps for smooth tracking
        limit = int(tl['motor'].get('limit', 140) if tl else initial_params.get('limit', 140))
        limit = max(10, min(180, limit))
        for cmd in ("on", f"angle {limit}", "softstartoff", "brakeoff", "quieton"):
            serial_mgr.send(cmd)
            await asyncio.sleep(0.04)

        dp = None

        def _make_dp(p):
            return DoublePendulum(
                L1=p.get('L1',1.0), L2=p.get('L2',0.8),
                m1=p.get('m1',1.0), m2=p.get('m2',0.8),
                th1_deg=p.get('th1',130), th2_deg=p.get('th2',95),
                w1=p.get('w1',0.0), damping=p.get('damping',0.01)
            )

        if init_mode == 'chaos':
            dp = _make_dp(self.params)

        t0         = time.time()
        next_tick  = t0
        last_speed = -1
        last_steps = None   # previous target sent — for velocity matching
        phase1 = phase2 = 0.0   # accumulated sine phase (continuous across freq changes)
        prev_t = 0.0

        try:
            while self._gen == gen:
                now = time.time()
                t   = now - t0
                dt_real = max(0.0, t - prev_t)
                prev_t  = t
                p   = self.params

                if tl:   # server-driven timeline: compute this tick's params
                    e = tl['offset'] + (now - t0)
                    e = (e % tl['total']) if tl['loop'] else min(e, tl['total'])
                    self.tl_elapsed = e
                    seg, idx = timeline_engine_params(tl['presets'], tl['timeline'], e)
                    if seg is None:      # missing preset in the bundle — hold still
                        next_tick += DT
                        await asyncio.sleep(max(0.0, next_tick - time.time()))
                        continue
                    if idx != tl['prev_idx'] or e < tl['prev_e']:   # segment entry / loop wrap
                        tl['prev_idx'] = idx
                        self.tl_idx  = idx
                        self.tl_name = tl['timeline'][idx].get('presetName', '')
                        if seg['mode'] == 'chaos':
                            self._reseed = True   # fresh ICs each (re)entry — no taper
                    tl['prev_e'] = e
                    # Motor settings: bundle values at start, but live slider
                    # pushes (osc/update) override — so Centre/Limit/Speed stay
                    # adjustable while a server-driven show is playing.
                    p = {**seg,
                         'limit':  self.params.get('limit',  tl['motor'].get('limit', 140)),
                         'speed':  self.params.get('speed',  tl['motor'].get('speed', 2250)),
                         'center': self.params.get('center', tl['motor'].get('center', 0.0))}

                # Mode can change tick-to-tick (timeline sequencing sine↔chaos)
                mode = p.get('mode', init_mode)
                lim  = max(10, min(180, int(p.get('limit', 140))))

                center = p.get('center', 0.0)   # zero-offset calibration (degrees)
                if mode == 'chaos':
                    if dp is None or self._reseed:   # seed / re-seed ICs (segment restart)
                        dp = _make_dp(p)
                        self._reseed = False
                    dp.damping   = p.get('damping', 0.01)
                    sim_speed    = p.get('simSpeed', 1.0)
                    steps_needed = max(1, round(DT * sim_speed / self.SIM_DT))
                    for _ in range(steps_needed):
                        dp.step(self.SIM_DT)
                    scale = p.get('scale', 0.58)
                    angle = max(-lim, min(lim, math.degrees(dp.th1) * scale + center))
                else:  # sine — accumulate phase so freq changes don't jump position
                    o1 = p.get('osc1', {}); o2 = p.get('osc2', {})
                    phase1 += 2*math.pi * o1.get('freq', 0.4) * dt_real
                    phase2 += 2*math.pi * o2.get('freq', 1.1) * dt_real
                    angle = max(-lim, min(lim, self._sine(phase1, phase2, t, p) + center))

                self.current_angle = angle
                steps = int(angle * GEAR_RATIO / 360.0 * PULSES_PER_REV)

                # Velocity matching: pick the step rate that covers this segment
                # in exactly one tick, so the motor glides instead of dashing and
                # idling. The user's speed slider is the upper bound.
                cap = max(self.MIN_SPS, min(self.MAX_SPS, int(p.get('speed', 2250))))
                # Gentle spin-up: ramp the speed ceiling over the first 1.5 s so
                # (re)starts glide to the target — presence restarts resume
                # mid-piece and were slamming 0→target at full cap (jolts that
                # also strain the mount and the driver's tracking).
                if t < 1.5:
                    cap = int(self.MIN_SPS + (cap - self.MIN_SPS) * (t / 1.5))
                if last_steps is None:
                    target_sps = cap        # first move: go at full cap to seed
                else:
                    required   = abs(steps - last_steps) / DT
                    target_sps = max(self.MIN_SPS, min(cap, required))
                # quantise so steady motion doesn't resend speed every tick
                target_sps = int(round(target_sps / self.SPS_QUANT)) * self.SPS_QUANT
                target_sps = max(self.MIN_SPS, min(cap, target_sps))

                if target_sps != last_speed:
                    serial_mgr.send(f"speed {target_sps}")
                    last_speed = target_sps
                serial_mgr.send(f"move {steps}")
                last_steps = steps

                next_tick += DT
                await asyncio.sleep(max(0.0, next_tick - time.time()))
        finally:
            self.running = False
            self.tl_active = False

    @staticmethod
    def _sine(phase1, phase2, t, p):
        o1  = p.get('osc1', {})
        o2  = p.get('osc2', {})
        A1  = o1.get('amp',80);  ph1=math.radians(o1.get('phase',0))
        A2  = o2.get('amp',25);  ph2=math.radians(o2.get('phase',90))
        raw = A1*math.sin(phase1+ph1) + A2*math.sin(phase2+ph2)

        env = p.get('envelope')
        if env is None:
            return raw
        atk=env.get('attack',8); hld=env.get('hold',30); dcy=env.get('decay',10)
        if   t < atk:       factor = t/atk if atk>0 else 1
        elif t < atk+hld:   factor = 1.0
        else:
            dt = t-atk-hld; factor = max(0.0, 1-dt/dcy) if dcy>0 else 0
        return raw * factor

osc_engine = OscillatorEngine()

# ── WebSocket broadcaster ─────────────────────────────────────────────────────
ws_clients: set = set()

async def ws_handler(websocket):
    ws_clients.add(websocket)
    try:
        async for _ in websocket:
            pass
    except Exception:
        pass
    finally:
        ws_clients.discard(websocket)

async def broadcast_loop():
    while True:
        safety.check_stale()     # fail-safe: trip if the camera feed went stale
        safety.presence_tick()   # attract mode: start on approach / park on idle
        show_log.tick(osc_engine.running)   # journal run/stop transitions
        lines = serial_mgr.read_available()
        msg = json.dumps({
            "lines": lines,
            "engine": {
                "running": engine.running,
                "name":    engine.current_name,
                "step":    engine.step_index,
                "total":   engine.total_steps,
                "loop":    engine.loop_mode,
            },
            "osc": {
                "running": osc_engine.running,
                "angle":   round(osc_engine.current_angle, 2),
                "tl": ({"idx": osc_engine.tl_idx, "name": osc_engine.tl_name,
                        "count": osc_engine.tl_count,
                        "elapsed": round(osc_engine.tl_elapsed, 1)}
                       if osc_engine.tl_active else None),
            },
            "safety": safety.state(),
            "serial": serial_mgr.connected,
        })
        dead = set()
        for ws in list(ws_clients):
            try:
                await ws.send(msg)
            except Exception:
                dead.add(ws)
        for ws in dead:
            ws_clients.discard(ws)
        await asyncio.sleep(0.05)

# ── Sequence Simulator ────────────────────────────────────────────────────────
def calc_move_time(p0, p1, speed, braking, softstart):
    dist = abs(p1 - p0)
    if dist == 0:
        return 0.0
    steps  = (dist * GEAR_RATIO / 360.0) * PULSES_PER_REV
    t_base = steps / max(speed, 1)
    extra  = 0.0
    if softstart and steps > RAMP_STEPS:
        extra += 0.15
    if braking and steps > RAMP_STEPS:
        extra += 0.15
    return t_base + extra

def simulate(steps: list) -> dict:
    events    = []
    t         = 0.0
    pos       = 0.0
    angle     = 90      # degrees — mirrors Pico's currentAngle
    speed     = 2000
    braking   = True
    softstart = True

    def move_to(target):
        nonlocal pos, t
        dt = calc_move_time(pos, target, speed, braking, softstart)
        if dt > 0:
            events.append({"type":"move","t0":t,"t1":t+dt,
                           "p0":pos,"p1":target,"speed":speed,
                           "braking":braking,"softstart":softstart})
        pos = target; t += dt

    for step in steps:
        cmd = step[0] if len(step) > 0 else ""
        val = step[1] if len(step) > 1 else None

        # left/right mirror Pico exactly: absolute ±(angle*GR/360*PPR) converted back to degrees
        if cmd == "left":
            move_to(angle)          # absolute +angle (same as Pico's +moveRange)

        elif cmd == "right":
            move_to(-angle)         # absolute -angle

        elif cmd == "neutral":
            move_to(0.0)

        elif cmd == "move":
            # val is in motor steps; convert to arm-degrees for display
            s = int(val) if val is not None else 0
            move_to(s / (GEAR_RATIO * PULSES_PER_REV / 360.0))

        elif cmd == "offset":
            s = int(val) if val is not None else 0
            move_to(pos + s / (GEAR_RATIO * PULSES_PER_REV / 360.0))

        elif cmd in ("cw", "ccw"):
            dt = calc_move_time(0, 360, speed, False, False)
            events.append({"type":"continuous","t0":t,"t1":t+dt,
                           "dir": 1 if cmd=="cw" else -1,"speed":speed})
            t += dt

        elif cmd == "wait":
            ms = int(val) if val is not None else 500
            events.append({"type":"wait","t0":t,"t1":t+ms/1000.0,"pos":pos})
            t += ms / 1000.0

        elif cmd == "wait_pend":
            events.append({"type":"wait_pend","t0":t,"t1":t+0.1,"pos":pos})
            t += 0.1

        elif cmd == "angle":
            angle = int(val) if val is not None else 90

        elif cmd == "speed":
            speed = int(val) if val is not None else 2000

        elif cmd == "brakeon":   braking   = True
        elif cmd == "brakeoff":  braking   = False
        elif cmd == "softstarton":  softstart = True
        elif cmd == "softstartoff": softstart = False

        elif cmd == "ramp_speed":
            if isinstance(val, dict):
                frm = val.get("from", speed)
                to  = val.get("to", speed)
                n   = val.get("steps", 10)
                ms  = val.get("step_ms", 200)
                dt  = n * ms / 1000.0
                events.append({"type":"ramp","t0":t,"t1":t+dt,
                               "from":frm,"to":to,"pos":pos})
                speed = to; t += dt

    # Flag fast reversals as tangle risk
    for i, e in enumerate(events):
        e["tangle_risk"] = False
        if e["type"] == "move" and i > 0:
            prev = events[i-1]
            if prev["type"] == "move":
                prev_dir = 1 if (prev["p1"] - prev["p0"]) > 0 else -1
                this_dir = 1 if (e["p1"] - e["p0"]) > 0 else -1
                gap = e["t0"] - prev["t1"]
                if prev_dir != this_dir and gap < 0.3:
                    e["tangle_risk"] = True

    return {"events": events, "duration": t, "final_pos": pos}

# ── HTTP handler ──────────────────────────────────────────────────────────────
_event_loop = None

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def send_json(self, data, status=200):
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, DELETE")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path   = parsed.path
        qs     = parse_qs(parsed.query)

        if path in ("/", "/index.html"):
            self.send_response(302)
            self.send_header("Location", "/chaos")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            return

        if path == "/mobile":
            f = MOBILE_HTML
            data = f.read_bytes() if f.exists() else b"<h1>mobile.html not found</h1>"
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return

        if path == "/chaos":
            f = Path(__file__).parent / "chaos.html"
            data = f.read_bytes() if f.exists() else b"<h1>chaos.html not found</h1>"
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return

        if path == "/stream":
            if not CAMERA_AVAILABLE or not camera_mgr.running:
                self.send_json({"error": "camera not available"}, 503)
                return
            try:
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                while True:
                    frame = camera_mgr.get_frame()
                    if frame:
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                        )
                        self.wfile.flush()
                    time.sleep(0.05)  # ~20 fps
            except (BrokenPipeError, ConnectionResetError):
                pass  # client disconnected
            return

        if path == "/api/status":
            self.send_json({
                "serial": serial_mgr.connected,
                "engine": {
                    "running": engine.running,
                    "name":    engine.current_name,
                    "step":    engine.step_index,
                    "total":   engine.total_steps,
                    "loop":    engine.loop_mode,
                },
                "log": serial_mgr.rx_lines[-80:],
            })
            return

        if path == "/api/sequences":
            files = sorted(SEQUENCES_DIR.glob("*.json"))
            self.send_json([f.stem for f in files])
            return

        if path == "/api/presets":
            if CHAOS_PRESETS.exists():
                try:
                    data = json.loads(CHAOS_PRESETS.read_text())
                except Exception:
                    data = {"presets": {}, "timeline": [], "loop": False}
            else:
                data = {"presets": {}, "timeline": [], "loop": False}
            self.send_json(data)
            return

        if path == "/api/safety":
            self.send_json(safety.state())
            return

        if path == "/api/showlog":
            # Activity summary over the last N hours (?h=24): events + how much
            # of the window the show was running. Timestamps only — no images.
            try:
                hours = max(0.1, min(24*30, float(qs.get("h", ["24"])[0])))
            except ValueError:
                hours = 24.0
            cutoff = time.time() - hours*3600
            events = []
            try:
                if SHOW_LOG.exists():
                    for line in SHOW_LOG.read_text().splitlines():
                        try:
                            r = json.loads(line)
                            if r.get("ts", 0) >= cutoff:
                                events.append(r)
                        except Exception:
                            pass
            except Exception as e:
                print(f"Show log read error: {e}")
            active = 0.0; runs = 0; longest = 0.0; t_open = None
            for r in events:
                if r["ev"] == "start":
                    t_open = r["ts"]; runs += 1
                elif r["ev"] == "stop":
                    d = r.get("ran_s")
                    if d is None and t_open is not None:
                        d = r["ts"] - t_open
                    if d:
                        active += d; longest = max(longest, d)
                    t_open = None
            if t_open is not None:            # still running right now
                d = time.time() - t_open
                active += d; longest = max(longest, d)
            self.send_json({
                "events": events[-60:], "active_s": round(active),
                "runs": runs, "longest_s": round(longest),
                "window_h": hours, "running": osc_engine.running,
                "pct": round(100*active/(hours*3600), 1),
            })
            return

        if path.startswith("/api/sequences/"):
            name = path.split("/api/sequences/")[1]
            f = SEQUENCES_DIR / f"{name}.json"
            if f.exists():
                self.send_json(json.loads(f.read_text()))
            else:
                self.send_json({"error": "not found"}, 404)
            return

        if path == "/api/cmd":
            cmd = qs.get("c", [""])[0].strip()
            if cmd:
                # Manual hands-on control: hold presence auto-start until the
                # room next empties, so the FSM doesn't fight the operator.
                if cmd.lower().split()[0] in ("stop", "off", "park", "reset", "left",
                                              "right", "neutral", "move", "offset",
                                              "cw", "ccw", "calibrate"):
                    safety.presence_hold = True
                # stop/off/reset must also kill the sequence engine,
                # otherwise it immediately re-issues movement commands
                if cmd.lower() in ("stop", "off", "reset"):
                    engine.stop()
                    # flush_and_stop sends "stop" — also forward off/reset to Pico
                    if cmd.lower() in ("off", "reset"):
                        time.sleep(0.05)
                        serial_mgr.send(cmd)
                else:
                    serial_mgr.send(cmd)
            self.send_json({"ok": True})
            return

        self.send_json({"error": "not found"}, 404)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body   = self.rfile.read(length)
        path   = urlparse(self.path).path
        try:
            data = json.loads(body) if body else {}
        except Exception:
            data = {}

        if path == "/api/sequences":
            name  = data.get("name", "untitled").strip().replace("/", "-")
            steps = data.get("steps", [])
            # Preserve blocks + globals so the UI can load them back as editable
            payload = {"name": name, "steps": steps}
            if "blocks"  in data: payload["blocks"]  = data["blocks"]
            if "globals" in data: payload["globals"] = data["globals"]
            f = SEQUENCES_DIR / f"{name}.json"
            f.write_text(json.dumps(payload, indent=2))
            self.send_json({"ok": True, "name": name})
            return

        if path.startswith("/api/sequences/") and path.endswith("/run"):
            name  = path.split("/api/sequences/")[1].replace("/run", "")
            f     = SEQUENCES_DIR / f"{name}.json"
            if not f.exists():
                self.send_json({"error": "not found"}, 404)
                return
            seq   = json.loads(f.read_text())
            loop  = data.get("loop", False)
            engine.start(seq.get("steps", []), name, loop, _event_loop)
            self.send_json({"ok": True, "running": name})
            return

        if path == "/api/stop":
            engine.stop()
            self.send_json({"ok": True})
            return

        if path == "/api/osc/start":
            safety.presence_hold = False   # manual run: autonomy continues after
            osc_engine.start(data, _event_loop)
            self.send_json({"ok": True})
            return

        if path == "/api/osc/update":
            osc_engine.update(data)
            self.send_json({"ok": True})
            return

        if path == "/api/osc/stop":
            safety.presence_hold = True    # manual stop: don't auto-restart
            osc_engine.stop()              # while the operator is in the room
            self.send_json({"ok": True})
            return

        if path == "/api/safety/config":
            safety.configure(data)
            self.send_json({"ok": True, **safety.state()})
            return
        if path == "/api/safety/arm":
            safety.arm()
            self.send_json({"ok": True, **safety.state()})
            return
        if path == "/api/safety/disarm":
            safety.disarm()
            self.send_json({"ok": True})
            return
        if path == "/api/safety/rearm":
            safety.rearm()
            self.send_json({"ok": True, **safety.state()})
            return

        if path == "/api/home":
            # Gravity home: de-energize so the arm hangs to its rest, let it
            # settle, then zero there. Runs in a thread so it survives the
            # browser disconnecting (consistent with the engine design).
            settle = float(data.get("settle", 8.0))
            settle = max(1.0, min(30.0, settle))
            osc_engine.stop()
            def _home():
                serial_mgr.send("off")        # gravity pulls the arm down
                time.sleep(settle)            # let the pendulum settle
                serial_mgr.send("calibrate")  # this rest point becomes 0
            threading.Thread(target=_home, daemon=True).start()
            self.send_json({"ok": True, "settle": settle})
            return

        if path == "/api/presets":
            motor = data.get("motor")
            if motor is None:   # older client payload — keep the stored motor cfg
                motor = load_show_bundle().get("motor", {})
            payload = {
                "presets":  data.get("presets", {}),
                "timeline": data.get("timeline", []),
                "loop":     bool(data.get("loop", False)),
                "motor":    motor,   # limit/speed/center for server-side playback
            }
            CHAOS_PRESETS.write_text(json.dumps(payload, indent=2))
            self.send_json({"ok": True})
            return

        if path == "/api/shutdown":
            engine.stop()
            serial_mgr.send("stop")
            self.send_json({"ok": True, "msg": "Shutting down Pi4..."})
            def _do_shutdown():
                import subprocess, time as _t
                _t.sleep(1.5)  # let response reach the client first
                subprocess.run(["sudo", "shutdown", "-h", "now"])
            threading.Thread(target=_do_shutdown, daemon=True).start()
            return

        if path == "/api/preview":
            result = simulate(data.get("steps", []))
            self.send_json(result)
            return

        self.send_json({"error": "not found"}, 404)

    def do_DELETE(self):
        path = urlparse(self.path).path
        if path.startswith("/api/sequences/"):
            name = path.split("/api/sequences/")[1]
            f = SEQUENCES_DIR / f"{name}.json"
            if f.exists():
                f.unlink()
            self.send_json({"ok": True})
            return
        self.send_json({"error": "not found"}, 404)

# ── Main ──────────────────────────────────────────────────────────────────────
async def main():
    global _event_loop
    _event_loop = asyncio.get_event_loop()

    show_log.log("boot")   # restarts are context for gaps in the journal
    serial_mgr.connect()

    # Auto-start a named sequence on boot (if it exists)
    if AUTOSTART_SEQ:
        f = SEQUENCES_DIR / f"{AUTOSTART_SEQ}.json"
        if f.exists():
            await asyncio.sleep(4.0)   # let Pico finish initialising
            seq = json.loads(f.read_text())
            engine.start(seq.get("steps", []), AUTOSTART_SEQ, loop=True,
                         event_loop=_event_loop)
            print(f"Autostart: running '{AUTOSTART_SEQ}' on loop")
        else:
            print(f"Autostart: no sequence named '{AUTOSTART_SEQ}' found — skipping")

    # 820×616 = exactly half of the 1640×1232 binned full-FOV mode — same 4:3
    # framing (no extra crop), ~4× fewer pixels to MJPEG-encode than full res.
    camera_mgr.start(width=820, height=616)

    # Auto-arm safety once the camera has warmed up (scene assumed clear at boot)
    if safety.arm_on_boot:
        threading.Timer(6.0, safety.arm).start()
        print("Safety: auto-arming in 6s")

    httpd = ThreadedHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(f"HTTP  → http://0.0.0.0:{HTTP_PORT}")

    ws_server = await websockets.legacy.server.serve(ws_handler, "0.0.0.0", WS_PORT)
    print(f"WS    → ws://0.0.0.0:{WS_PORT}")
    print("Ready.")

    await broadcast_loop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        camera_mgr.stop()