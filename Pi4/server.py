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
    CAMERA_AVAILABLE = True
except ImportError:
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
            self.send_header("Location", "/mobile")
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

    camera_mgr.start(width=1640, height=1232)

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