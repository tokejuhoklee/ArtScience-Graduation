/*
  Pico W 2 — JSS57 Stepper Web Controller (FIXED)
  - Fixed interrupt handling
  - Added BOOTSEL button reset
  - Fixed enable pin logic options
  - GP2=STEP (PUL+), GP3=DIR (DIR+), GP4=ENA (ENA+ optional), GP5=PEND (PEND+ input)
  - AP IP: http://192.168.4.1
*/

#include <WiFi.h>
#include <WebServer.h>
#include <hardware/watchdog.h>
#include <pico/bootrom.h>

//
// === HARDWARE CONFIG ===
//
const uint8_t STEP_PIN   = 2;   // PUL+ -> GP2
const uint8_t DIR_PIN    = 3;   // DIR+ -> GP3
const int8_t  ENABLE_PIN = 4;   // ENA+ -> GP4 (set -1 if ENA+ tied to 3.3V)
const uint8_t PEND_PIN   = 5;   // PEND+ -> GP5 (input) ; PEND- -> GND

// IMPORTANT: Set this based on your driver's enable logic
// true = HIGH enables motor, false = LOW enables motor
const bool ENABLE_ACTIVE_HIGH = false;  // JSS57 typically uses active-LOW enable

//
// === NETWORK (AP) ===
//
const char* ap_ssid     = "Pico-Stepper";
const char* ap_password = "thankswouter";

//
// === MOTOR / MOTION DEFAULTS ===
//
const int PULSES_PER_REV = 800;      // your chosen microstep
int currentAngle = 45;               // degrees
const int minAngle = 5, maxAngle = 360;

int moveRange = 0;                   // computed from currentAngle
int currentSpeed = 700;              // steps per second (user tunable)
int minSpeed = 50, maxSpeed = 4000;
int brakeDistance = 200;             // steps to start braking
int minBrakeSpeed = 50;
bool enableBraking = true;           // NEW: toggle braking on/off

unsigned long pendulumDelay = 250;   // ms pause at endpoints
bool disableMotorAtRest = false;     // NEW: disable motor during pendulum rest

//
// === STATE ===
//
String logBuffer = "";
const int MAX_LOG_LINES = 300;

volatile long currentPosition = 0;   // step counter (0 = neutral after calibrate)
volatile bool motorEnabled = false;

bool pendulumMode = true;
long currentTarget = 0;
bool goingRight = true;
bool pendulumInitialized = false;
unsigned long positionReachedTime = 0;
bool positionJustReached = false;

//
// === STEPPER ENGINE (non-blocking) ===
//
volatile long stepsRemaining = 0;
volatile bool stepDir = 0;            // 0 = DIR LOW, 1 = DIR HIGH
volatile unsigned long stepIntervalMicros = 1000; // micros between steps
const unsigned int STEP_PULSE_WIDTH_US = 4;       // pulse HIGH width microseconds
volatile bool stepPulseInProgress = false;
volatile unsigned long nextStepTime = 0;

//
// === WEB SERVER ===
//
WebServer server(80);

//
// === UTILITIES (Forward declarations) ===
//
void appendLog(const String &m);

//
// === BOOTSEL BUTTON ===
//
unsigned long lastBootselCheck = 0;
const unsigned long BOOTSEL_CHECK_INTERVAL = 100; // ms

void checkBootselButton() {
  // BOOTSEL button is on GPIO 25 internally, read via special bootrom function
  // Hold for >500ms to trigger restart
  static unsigned long bootselPressStart = 0;
  static bool wasPressed = false;
  
  bool isPressed = (BOOTSEL) ? true : false; // BOOTSEL is defined in pico SDK
  
  if (isPressed && !wasPressed) {
    bootselPressStart = millis();
    wasPressed = true;
  } else if (!isPressed && wasPressed) {
    wasPressed = false;
  } else if (isPressed && wasPressed) {
    if (millis() - bootselPressStart > 500) {
      appendLog("BOOTSEL held - RESTARTING...");
      delay(100); // Give time to send response
      watchdog_reboot(0, 0, 0);
      while(1); // Wait for reset
    }
  }
}

//
// === UTILITIES ===
//
void appendLog(const String &m) {
  Serial.println(m);
  logBuffer += m + "\n";
  // trim
  int lines = 0;
  for (int i = 0; i < (int)logBuffer.length(); ++i) if (logBuffer[i] == '\n') lines++;
  while (lines > MAX_LOG_LINES) {
    int idx = logBuffer.indexOf('\n');
    if (idx >= 0) {
      logBuffer = logBuffer.substring(idx + 1);
      lines--;
    } else break;
  }
}

long angleToSteps(float angle) {
  float perDeg = float(PULSES_PER_REV) / 360.0f;
  return (long)round(angle * perDeg);
}

void updateMoveRangeFromAngle() {
  moveRange = (int)angleToSteps((float)currentAngle);
  appendLog("🎯 Angle: " + String(currentAngle) + "° → ±" + String(moveRange) + " steps");
}

unsigned long speedToIntervalMicros(int stepsPerSec) {
  if (stepsPerSec <= 0) return 1000000UL;
  double s = 1.0 / (double)stepsPerSec;
  return (unsigned long)round(s * 1e6);
}

void setMotorEnabled(bool on) {
  motorEnabled = on;
  if (ENABLE_PIN >= 0) {
    // Apply the correct enable logic based on your driver
    bool pinState = ENABLE_ACTIVE_HIGH ? on : !on;
    digitalWrite(ENABLE_PIN, pinState ? HIGH : LOW);
  }
  appendLog(String(on ? "✓ MOTOR ON" : "✓ MOTOR OFF"));
}

void scheduleSteps(long steps) {
  if (steps == 0) return;
  
  noInterrupts();
  if (steps > 0) {
    stepDir = 0;
    stepsRemaining = steps;
  } else {
    stepDir = 1;
    stepsRemaining = -steps;
  }
  stepIntervalMicros = speedToIntervalMicros(currentSpeed);
  nextStepTime = micros(); // Set next step time BEFORE re-enabling interrupts
  interrupts(); // FIXED: Now properly re-enables interrupts
}

void smartMoveTo(long target) {
  if (!motorEnabled) { 
    appendLog("⚠ Motor disabled - enable first!"); 
    return; 
  }
  long delta = target - currentPosition;
  if (delta == 0) return;
  appendLog("→ Scheduling " + String(delta) + " steps to target " + String(target));
  scheduleSteps(delta);
}

void emergencyStop() {
  noInterrupts();
  stepsRemaining = 0;
  interrupts();
  appendLog("*** EMERGENCY STOP ***");
}

//
// === WEB HANDLERS ===
//
void handleRoot() {
  const char *html = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Pico Stepper</title>
  <style>
    :root{--accent:#7c4dff;--bg:#0b0b12;--muted:#a99bd1}
    body{margin:0;font-family:Inter,system-ui,Arial;background:var(--bg);color:#fff}
    .wrap{max-width:900px;margin:18px auto;padding:18px}
    header{display:flex;align-items:center;gap:12px}
    h1{margin:0;font-size:1.25rem;color:var(--accent)}
    .card{background:#0f0f15;border-radius:12px;padding:12px;margin-top:12px;box-shadow:0 6px 20px rgba(0,0,0,.6)}
    .row{display:flex;gap:8px;flex-wrap:wrap}
    button{background:var(--accent);border:none;color:#fff;padding:10px 12px;border-radius:8px;cursor:pointer}
    button.ghost{background:transparent;border:1px solid #2a2540;color:var(--muted)}
    button:active{transform:scale(0.95)}
    .log{height:220px;background:#05050a;border-radius:8px;padding:10px;overflow:auto;font-family:monospace;font-size:13px}
    .controls{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:8px;margin-top:10px}
    label{font-size:12px;color:var(--muted);display:block;margin-bottom:6px}
    input[type=range]{width:100%}
    .small{font-size:12px;color:var(--muted)}
    footer{margin-top:12px;color:var(--muted);font-size:12px}
    .status{padding:8px;background:#1a1a2e;border-radius:6px;margin-bottom:8px;font-size:13px}
  </style>
</head>
<body>
  <div class="wrap">
    <header><h1>Pico Stepper Controller</h1></header>

    <div class="card">
      <div class="status">
        <strong>Status:</strong> Motor must be <strong>ENABLED</strong> to move!<br>
        <small>Press BOOTSEL button on Pico for >500ms to restart</small>
      </div>
      
      <div class="row">
        <div style="flex:1">
          <div class="log" id="log">Loading log...</div>
        </div>
        <div style="width:300px">
          <div class="controls">
            <button onclick="cmd('on')" style="background:#4caf50">Enable Motor</button>
            <button onclick="cmd('off')" class="ghost">Disable Motor</button>
            <button onclick="cmd('stop')" style="background:#ff3860">EMERGENCY STOP</button>
            <button onclick="cmd('left')">LEFT</button>
            <button onclick="cmd('right')">RIGHT</button>
            <button onclick="cmd('neutral')">NEUTRAL</button>
            <button onclick="cmd('pend')">Toggle Pendulum</button>
          </div>

          <div style="margin-top:12px">
            <label>Speed: <span id="speedVal"></span> sps</label>
            <input id="speed" type="range" min="50" max="4000" value="700" oninput="updateSpeed(this.value)">
            <label>Angle: <span id='angleVal'></span> deg</label>
            <input id="angle" type="range" min="5" max="360" value="45" oninput="updateAngle(this.value)">
            <label>Rest Time: <span id='delayVal'></span> ms</label>
            <input id="delay" type="range" min="0" max="5000" step="50" value="250" oninput="updateDelay(this.value)">
            <div style="margin-top:8px">
              <label style="display:inline;margin-right:10px">
                <input type="checkbox" id="restToggle" onchange="toggleRest(this.checked)"> Disable motor at rest
              </label>
              <label style="display:inline">
                <input type="checkbox" id="brakeToggle" checked onchange="toggleBrake(this.checked)"> Enable braking
              </label>
            </div>
            <div style="margin-top:8px;font-size:11px;color:#888">
              💡 For whip cracking: disable braking for sharp reversals!
            </div>
            <div style="margin-top:8px">
              <input id="cmdtext" style="width:66%;padding:8px;border-radius:6px;border:1px solid #222;background:#07070b;color:#fff" placeholder="raw command">
              <button onclick="sendManual()">Send</button>
            </div>
          </div>
        </div>
      </div>
      <footer>
        Connect to <strong>Pico-Stepper</strong> WiFi. Open <strong>http://192.168.4.1</strong><br>
        <small>Wiring: GP2=STEP, GP3=DIR, GP4=ENA, GP5=PEND</small>
      </footer>
    </div>
  </div>

<script>
function refreshLog(){ fetch('/log').then(r=>r.text()).then(t=>{document.getElementById('log').textContent=t;document.getElementById('log').scrollTop=1e9;});}
function cmd(c){ fetch('/cmd?c='+encodeURIComponent(c)).then(()=>setTimeout(refreshLog,120)); }
function sendManual(){ let v=document.getElementById('cmdtext').value; if(!v) return; cmd(v); document.getElementById('cmdtext').value=''; }
function updateSpeed(v){ document.getElementById('speedVal').textContent=v; fetch('/cmd?c='+encodeURIComponent('speed '+v)); }
function updateAngle(v){ document.getElementById('angleVal').textContent=v; fetch('/cmd?c='+encodeURIComponent('angle '+v)); }
function updateDelay(v){ document.getElementById('delayVal').textContent=v; fetch('/cmd?c='+encodeURIComponent('delay '+v)); }
function toggleRest(checked){ cmd(checked ? 'reston' : 'restoff'); }
function toggleBrake(checked){ cmd(checked ? 'brakeon' : 'brakeoff'); }
setInterval(refreshLog,500);
window.onload=function(){ refreshLog(); document.getElementById('speedVal').textContent=document.getElementById('speed').value; document.getElementById('angleVal').textContent=document.getElementById('angle').value; document.getElementById('delayVal').textContent=document.getElementById('delay').value; }
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

void handleLog() {
  server.send(200, "text/plain", logBuffer);
}

void handleCmd() {
  if (!server.hasArg("c")) { server.send(400, "text/plain", "missing cmd"); return; }
  String cmd = server.arg("c");
  cmd.toLowerCase(); cmd.trim();
  appendLog("> " + cmd);

  if (cmd == "help" || cmd == "h") {
    appendLog("Commands: left,right,neutral,on,off,stop,pend,config,speed <n>,angle <n>,move <steps>,offset <n>,calibrate");
    server.send(200, "text/plain", "OK"); return;
  }
  if (cmd == "on") { setMotorEnabled(true); server.send(200, "text/plain", "OK"); return; }
  if (cmd == "off") { setMotorEnabled(false); pendulumMode=false; server.send(200, "text/plain", "OK"); return; }
  if (cmd == "stop") { emergencyStop(); server.send(200, "text/plain", "OK"); return; }

  if (cmd == "left") {
    if (!motorEnabled) { appendLog("Motor disabled"); server.send(200,"text/plain","disabled"); return;}
    currentTarget = moveRange; appendLog("LEFT -> "+String(currentTarget)); smartMoveTo(currentTarget); server.send(200,"text/plain","OK"); return;
  }
  if (cmd == "right") {
    if (!motorEnabled) { appendLog("Motor disabled"); server.send(200,"text/plain","disabled"); return;}
    currentTarget = -moveRange; appendLog("RIGHT -> "+String(currentTarget)); smartMoveTo(currentTarget); server.send(200,"text/plain","OK"); return;
  }
  if (cmd == "neutral") {
    if (!motorEnabled) { appendLog("Motor disabled"); server.send(200,"text/plain","disabled"); return;}
    appendLog("NEUTRAL -> 0"); smartMoveTo(0); server.send(200,"text/plain","OK"); return;
  }
  if (cmd == "pend") {
    if (!motorEnabled) { appendLog("Motor disabled"); server.send(200,"text/plain","disabled"); return;}
    pendulumMode = !pendulumMode;
    pendulumInitialized = false;
    appendLog(String(pendulumMode ? "PENDULUM ON" : "PENDULUM OFF"));
    server.send(200,"text/plain","OK"); return;
  }

  if (cmd.startsWith("speed ")) {
    int v = cmd.substring(6).toInt();
    currentSpeed = constrain(v, minSpeed, maxSpeed);
    appendLog("Speed: " + String(currentSpeed));
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd.startsWith("angle ")) {
    int v = cmd.substring(6).toInt();
    currentAngle = constrain(v, minAngle, maxAngle);
    updateMoveRangeFromAngle();
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd.startsWith("delay ")) {
    int v = cmd.substring(6).toInt();
    pendulumDelay = constrain(v, 0, 5000);
    appendLog("Pendulum delay: " + String(pendulumDelay) + "ms");
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd == "restoff") {
    disableMotorAtRest = false;
    appendLog("Motor stays ON during rest");
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd == "reston") {
    disableMotorAtRest = true;
    appendLog("Motor disables during rest");
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd == "brakeon") {
    enableBraking = true;
    appendLog("Braking ENABLED (smooth stop)");
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd == "brakeoff") {
    enableBraking = false;
    appendLog("Braking DISABLED (sharp stop for whip crack!)");
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd.startsWith("brakedist ")) {
    int v = cmd.substring(10).toInt();
    brakeDistance = constrain(v, 0, 1000);
    appendLog("Brake distance: " + String(brakeDistance) + " steps");
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd.startsWith("move ")) {
    if (!motorEnabled) { appendLog("Motor disabled"); server.send(200,"text/plain","disabled"); return;}
    long pos = cmd.substring(5).toInt();
    appendLog("Move to: " + String(pos));
    smartMoveTo(pos);
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd.startsWith("offset ")) {
    if (!motorEnabled) { appendLog("Motor disabled"); server.send(200,"text/plain","disabled"); return;}
    long off = cmd.substring(7).toInt();
    long newt = currentPosition + off;
    appendLog("Offset " + String(off) + " -> " + String(newt));
    smartMoveTo(newt);
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd == "calibrate") {
    appendLog("Calibrate: shifting coordinate so current position becomes 0.");
    long shift = currentPosition;
    currentPosition = 0;
    currentTarget -= shift;
    appendLog("Calibrated. Current == 0.");
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd == "config") {
    appendLog("=== CONFIG ===");
    appendLog("moveRange: " + String(moveRange));
    appendLog("angle: " + String(currentAngle));
    appendLog("speed: " + String(currentSpeed));
    appendLog("pendulumDelay: " + String(pendulumDelay) + "ms");
    appendLog("braking: " + String(enableBraking ? "ON" : "OFF"));
    appendLog("brakeDistance: " + String(brakeDistance));
    appendLog("disableMotorAtRest: " + String(disableMotorAtRest ? "ON" : "OFF"));
    appendLog("motorEnabled: " + String(motorEnabled ? "ON" : "OFF"));
    appendLog("================");
    server.send(200,"text/plain","OK"); return;
  }

  appendLog("Unknown cmd: " + cmd);
  server.send(200,"text/plain","Unknown");
}

//
// === STEPPER TICK ===
//
void stepperTick() {
  unsigned long now = micros();

  if (stepPulseInProgress) {
    if (now >= nextStepTime) {
      digitalWrite(STEP_PIN, LOW);
      stepPulseInProgress = false;
      nextStepTime = now + (stepIntervalMicros - STEP_PULSE_WIDTH_US);
    }
    return;
  }

  if (stepsRemaining > 0 && now >= nextStepTime) {
    // braking logic (optional)
    unsigned long interval = stepIntervalMicros;
    if (enableBraking) {
      long rem = stepsRemaining;
      if (rem <= brakeDistance) {
        int targetSpeed = minBrakeSpeed + (int)((double)(currentSpeed - minBrakeSpeed) * ((double)rem / (double)brakeDistance));
        if (targetSpeed < minBrakeSpeed) targetSpeed = minBrakeSpeed;
        interval = speedToIntervalMicros(targetSpeed);
      }
    }

    digitalWrite(DIR_PIN, stepDir ? HIGH : LOW);
    digitalWrite(STEP_PIN, HIGH);
    stepPulseInProgress = true;
    nextStepTime = now + STEP_PULSE_WIDTH_US;

    if (stepDir == 0) currentPosition++;
    else currentPosition--;
    stepsRemaining--;
  }
}

//
// === SETUP / LOOP ===
//
void setupPins() {
  pinMode(STEP_PIN, OUTPUT); digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);  digitalWrite(DIR_PIN, LOW);
  if (ENABLE_PIN >= 0) { 
    pinMode(ENABLE_PIN, OUTPUT); 
    // Initialize disabled (inverted if active-low)
    digitalWrite(ENABLE_PIN, ENABLE_ACTIVE_HIGH ? LOW : HIGH);
  }
  pinMode(PEND_PIN, INPUT_PULLUP); // PEND is open-collector-like: treat with pullup
}

void setup() {
  Serial.begin(115200);
  delay(200);
  appendLog("=== PicoW Stepper Controller FIXED ===");

  setupPins();
  updateMoveRangeFromAngle();

  // start AP
  WiFi.softAP(ap_ssid, ap_password);
  IPAddress ip = WiFi.softAPIP();
  appendLog("AP started: " + String(ap_ssid) + " IP: " + ip.toString());

  // web routes
  server.on("/", handleRoot);
  server.on("/log", handleLog);
  server.on("/cmd", handleCmd);
  server.begin();
  appendLog("Web server started");

  setMotorEnabled(false);
  appendLog("Ready. Open http://192.168.4.1");
  appendLog("⚠ REMEMBER TO CLICK 'Enable Motor' FIRST!");
}

unsigned long pendulumLastCheck = 0;
void loop() {
  server.handleClient();
  stepperTick();

  // Check BOOTSEL button
  if (millis() - lastBootselCheck >= BOOTSEL_CHECK_INTERVAL) {
    lastBootselCheck = millis();
    checkBootselButton();
  }

  // read PEND (in-position) optionally to improve pendulum detection
  bool pendActive = (digitalRead(PEND_PIN) == LOW); // closed -> LOW -> in-position

  if (pendulumMode && motorEnabled) {
    if (!pendulumInitialized) {
      currentTarget = moveRange;
      goingRight = true;
      pendulumInitialized = true;
      appendLog("→ Pendulum start to RIGHT: " + String(currentTarget));
      smartMoveTo(currentTarget);
    }

    long distance = abs(currentPosition - currentTarget);
    bool atTarget = (distance <= 2) || pendActive;

    if (atTarget) {
      if (!positionJustReached) {
        positionReachedTime = millis();
        positionJustReached = true;
        // Optionally disable motor during rest
        if (disableMotorAtRest) {
          setMotorEnabled(false);
        }
      } else {
        if (millis() - positionReachedTime >= pendulumDelay) {
          // Re-enable motor if it was disabled
          if (disableMotorAtRest && !motorEnabled) {
            setMotorEnabled(true);
          }
          if (goingRight) currentTarget = -moveRange;
          else currentTarget = moveRange;
          goingRight = !goingRight;
          positionJustReached = false;
          appendLog(String(goingRight ? "→ Moving RIGHT: " : "← Moving LEFT: ") + String(currentTarget));
          smartMoveTo(currentTarget);
        }
      }
    }
  }

  // tiny cooperative delay
  delay(0);
}
