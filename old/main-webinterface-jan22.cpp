/*
  Pico W 2 — JSS57 Stepper Web Controller (GEAR REDUCTION VERSION)
  - Gear reduction support: specify driving/driven teeth for accurate angle control
  - Simple speed profile with angle-based braking
  - Auto re-enable motor before moves when disable-at-rest is used
  - Quick UI buttons for setting angle to 90° and 180° (updates slider)
  - GP2=STEP (PUL+), GP3=DIR (DIR+), GP4=ENA (ENA+ optional), GP5=PEND (PEND+ input)
  - AP IP: http://192.168.4.1 (Pico AP default)
*/

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
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
const bool ENABLE_ACTIVE_HIGH = false;  // JSS57 usually active-LOW

//
// === NETWORK (AP) ===
//
const char* ap_ssid     = "Pico-Stepper";
// const char* ap_password = "thankswouter";  // Commented out - will be open network

//
// === MOTOR / MOTION DEFAULTS ===
//
const int PULSES_PER_REV = 400;      // Set to your DIP microstep configuration (800, 1600, 3200, ...)

// GEAR REDUCTION (pulley/belt/gearbox)
int drivingTeeth = 20;    // Teeth on motor pulley (smaller)
int drivenTeeth = 60;     // Teeth on arm/load pulley (larger)
float gearReduction = 3.0; // Calculated: drivenTeeth / drivingTeeth

int currentAngle = 45;               // default degrees for slider (ARM angle, not motor)
const int minAngle = 5, maxAngle = 360;

int moveRange = 0;                   // computed from currentAngle (in MOTOR steps)

// speed units: steps per second (Hz)
int currentSpeed = 50;             // max cruise speed
int minSpeed = 50, maxSpeed = 4000;

int minBrakeSpeed = 50;
bool enableBraking = true;           // braking toggle

unsigned long pendulumDelay = 500;   // ms pause at endpoints
bool disableMotorAtRest = false;     // disable motor during pendulum rest (user option)

//
// === STATE ===
//
String logBuffer = "";
const int MAX_LOG_LINES = 300;

volatile long currentPosition = 0;   // step counter (software-only, motor steps)
volatile bool motorEnabled = false;

bool pendulumMode = true;
bool continuousMode = false;  // New: continuous rotation mode
long currentTarget = 0;
bool goingRight = true;
bool pendulumInitialized = false;
unsigned long positionReachedTime = 0;
bool positionJustReached = false;
bool motorWasDisabledAtRest = false;  // Track if we already disabled for this rest period

//
// === STEPPER ENGINE (non-blocking, simple speed control) ===
//
volatile long stepsRemaining = 0;          // steps left to execute (always >= 0)
volatile bool stepDir = 0;                // 0 = DIR LOW, 1 = DIR HIGH
volatile unsigned long stepIntervalMicros = 1000; // current interval micros between step edges
const unsigned int STEP_PULSE_WIDTH_US = 4;       // pulse HIGH width microseconds
volatile bool stepPulseInProgress = false;
volatile unsigned long nextStepTime = 0;

// Simple braking: track initial steps for this move
volatile long initialStepsScheduled = 0;

// Soft-start protection (reduces initial torque spike)
int startupRampSteps = 150;  // Steps to reach full speed (adjustable)
bool enableSoftStart = true;  // Toggle for soft acceleration

//
// === WEB SERVER ===
//
WebServer server(80);
DNSServer dnsServer;
const byte DNS_PORT = 53;

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
  static unsigned long bootselPressStart = 0;
  static bool wasPressed = false;
  bool isPressed = false;
  #ifdef BOOTSEL
    isPressed = (BOOTSEL) ? true : false;
  #endif

  if (isPressed && !wasPressed) {
    bootselPressStart = millis();
    wasPressed = true;
  } else if (!isPressed && wasPressed) {
    wasPressed = false;
  } else if (isPressed && wasPressed) {
    if (millis() - bootselPressStart > 500) {
      appendLog("BOOTSEL held - RESTARTING...");
      delay(100);
      watchdog_reboot(0, 0, 0);
      while (1);
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

void updateGearReduction() {
  if (drivenTeeth > 0 && drivingTeeth > 0) {
    gearReduction = (float)drivenTeeth / (float)drivingTeeth;
    appendLog("⚙️ Gear ratio: " + String(drivingTeeth) + ":" + String(drivenTeeth) + 
              " (reduction " + String(gearReduction, 2) + ":1)");
  }
}

long armAngleToMotorSteps(float armAngle) {
  // Arm angle -> motor angle (multiply by reduction) -> motor steps
  float motorAngle = armAngle * gearReduction;
  float stepsPerDegree = float(PULSES_PER_REV) / 360.0f;
  return (long)round(motorAngle * stepsPerDegree);
}

void updateMoveRangeFromAngle() {
  moveRange = armAngleToMotorSteps((float)currentAngle);
  float actualMotorAngle = currentAngle * gearReduction;
  
  // Auto-limit speed for larger angles (safety feature for high inertia)
  int safeMaxSpeed = maxSpeed;
  if (currentAngle > 180) safeMaxSpeed = 1500;
  else if (currentAngle > 90) safeMaxSpeed = 2000;
  else if (currentAngle > 45) safeMaxSpeed = 3000;
  
  // Cap current speed if it exceeds safe limit
  if (currentSpeed > safeMaxSpeed) {
    currentSpeed = safeMaxSpeed;
    appendLog("⚠️ Speed auto-limited to " + String(safeMaxSpeed) + " for safety (large angle)");
  }
  
  appendLog("🎯 Arm: " + String(currentAngle) + "° → Motor: " + String(actualMotorAngle, 1) + 
            "° → ±" + String(moveRange) + " steps (max speed: " + String(safeMaxSpeed) + ")");
}

unsigned long speedToIntervalMicros(int stepsPerSec) {
  if (stepsPerSec <= 0) return 1000000UL;
  double s = 1.0 / (double)stepsPerSec;
  return (unsigned long)round(s * 1e6);
}

//
// === DRIVER ENABLE/STATE ===
//
void setMotorEnabled(bool on) {
  motorEnabled = on;
  if (ENABLE_PIN >= 0) {
    bool pinState = ENABLE_ACTIVE_HIGH ? on : !on;
    digitalWrite(ENABLE_PIN, pinState ? HIGH : LOW);
  }
  appendLog(String(on ? "✓ MOTOR ON" : "✓ MOTOR OFF"));
}

//
// === SCHEDULING ===
//
void scheduleSteps(long steps) {
  if (steps == 0) return;

  // auto re-enable motor if it was disabled (so moves work after disable-at-rest)
  if (!motorEnabled) {
    appendLog("scheduleSteps: Motor disabled, enabling...");
    setMotorEnabled(true);
    delay(10);  // settle time for motor to energize
  }

  noInterrupts();
  if (steps > 0) {
    stepDir = 0;
    stepsRemaining = steps;
  } else {
    stepDir = 1;
    stepsRemaining = -steps;
  }

  // Track total steps for braking calculation
  initialStepsScheduled = stepsRemaining;

  // Start at configured speed
  stepIntervalMicros = speedToIntervalMicros(currentSpeed);
  nextStepTime = micros();
  interrupts();
}

void smartMoveTo(long target) {
  // auto-enable motor if disabled
  if (!motorEnabled) {
    appendLog("Motor was disabled — enabling for move");
    setMotorEnabled(true);
    delay(2);
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
void handleRoot(); // forward declare
void handleLog() { server.send(200, "text/plain", logBuffer); }

void handleCmd() {
  if (!server.hasArg("c")) { server.send(400, "text/plain", "missing cmd"); return; }
  String cmd = server.arg("c");
  cmd.toLowerCase(); cmd.trim();
  appendLog("> " + cmd);

  if (cmd == "help" || cmd == "h") {
    appendLog("Commands: left,right,neutral,on,off,stop,pend,cw,ccw,continuous,config,speed <n>,angle <n>,move <steps>,offset <n>,calibrate,set90,set180,gear <driving>:<driven>");
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
    continuousMode = false;  // Disable continuous when pendulum starts
    appendLog(String(pendulumMode ? "PENDULUM ON" : "PENDULUM OFF"));
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd == "continuous" || cmd == "cont") {
    if (!motorEnabled) { appendLog("Motor disabled"); server.send(200,"text/plain","disabled"); return;}
    continuousMode = !continuousMode;
    pendulumMode = false;  // Disable pendulum when continuous starts
    if (continuousMode) {
      appendLog("CONTINUOUS ROTATION ON - motor will spin indefinitely");
      // Start continuous rotation
      long largeTarget = currentPosition + 1000000;  // Essentially infinite
      smartMoveTo(largeTarget);
    } else {
      appendLog("CONTINUOUS ROTATION OFF");
      emergencyStop();
    }
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd == "cw") {
    if (!motorEnabled) { appendLog("Motor disabled"); server.send(200,"text/plain","disabled"); return;}
    continuousMode = true;
    pendulumMode = false;
    appendLog("Spinning CLOCKWISE continuously");
    long largeTarget = currentPosition + 1000000;
    smartMoveTo(largeTarget);
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd == "ccw") {
    if (!motorEnabled) { appendLog("Motor disabled"); server.send(200,"text/plain","disabled"); return;}
    continuousMode = true;
    pendulumMode = false;
    appendLog("Spinning COUNTER-CLOCKWISE continuously");
    long largeTarget = currentPosition - 1000000;
    smartMoveTo(largeTarget);
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
  
  // New gear command: "gear 20:60" or "gear 1:3"
  if (cmd.startsWith("gear ")) {
    String gearStr = cmd.substring(5);
    int colonIdx = gearStr.indexOf(':');
    if (colonIdx > 0) {
      int driving = gearStr.substring(0, colonIdx).toInt();
      int driven = gearStr.substring(colonIdx + 1).toInt();
      if (driving > 0 && driven > 0) {
        drivingTeeth = driving;
        drivenTeeth = driven;
        updateGearReduction();
        updateMoveRangeFromAngle();  // Recalculate with new ratio
        server.send(200,"text/plain","OK"); return;
      }
    }
    appendLog("Invalid gear format. Use: gear 20:60");
    server.send(400,"text/plain","Invalid format"); return;
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
  if (cmd == "softstarton") {
    enableSoftStart = true;
    appendLog("Soft-start ENABLED (gentle acceleration)");
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd == "softstartoff") {
    enableSoftStart = false;
    appendLog("Soft-start DISABLED (instant acceleration)");
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd.startsWith("rampsteps ")) {
    int v = cmd.substring(10).toInt();
    startupRampSteps = constrain(v, 0, 500);
    appendLog("Soft-start ramp: " + String(startupRampSteps) + " steps");
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd.startsWith("move ")) {
    if (!motorEnabled) { appendLog("Motor disabled - enabling..."); setMotorEnabled(true); }
    long pos = cmd.substring(5).toInt();
    appendLog("Move to: " + String(pos));
    smartMoveTo(pos);
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd.startsWith("offset ")) {
    if (!motorEnabled) { appendLog("Motor disabled - enabling..."); setMotorEnabled(true); }
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

  // Set angle to 90° or 180° (updates slider, doesn't move motor)
  if (cmd == "set90") {
    currentAngle = 90;
    updateMoveRangeFromAngle();
    server.send(200,"text/plain","OK"); return;
  }
  if (cmd == "set180") {
    currentAngle = 180;
    updateMoveRangeFromAngle();
    server.send(200,"text/plain","OK"); return;
  }
  
  if (cmd == "setneutral") {
    appendLog("Setting current position as new neutral (0)");
    long shift = currentPosition;
    currentPosition = 0;
    currentTarget -= shift;
    appendLog("New neutral set. Current position is now 0.");
    server.send(200,"text/plain","OK"); return;
  }

  if (cmd == "config") {
    appendLog("=== CONFIG ===");
    appendLog("Gear: " + String(drivingTeeth) + ":" + String(drivenTeeth) + 
              " (reduction " + String(gearReduction, 2) + ":1)");
    appendLog("moveRange: " + String(moveRange) + " motor steps");
    appendLog("angle: " + String(currentAngle) + "° (arm angle)");
    appendLog("speed: " + String(currentSpeed));
    appendLog("pendulumDelay: " + String(pendulumDelay) + "ms");
    appendLog("braking: " + String(enableBraking ? "ON" : "OFF"));
    appendLog("softStart: " + String(enableSoftStart ? "ON" : "OFF"));
    appendLog("rampSteps: " + String(startupRampSteps));
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
    long stepsCompleted = initialStepsScheduled - stepsRemaining;
    int stepSpeed = currentSpeed;
    
    // SOFT START: Gentle ramp-up at beginning (reduces torque spike)
    if (enableSoftStart && stepsCompleted < startupRampSteps && initialStepsScheduled > startupRampSteps) {
      float rampFraction = (float)stepsCompleted / (float)startupRampSteps;
      stepSpeed = minBrakeSpeed + (int)((currentSpeed - minBrakeSpeed) * rampFraction);
      stepSpeed = constrain(stepSpeed, minBrakeSpeed, currentSpeed);
    }
    // BRAKING: Slow down as approaching target
    else if (enableBraking && initialStepsScheduled > 0) {
      float remainingFraction = (float)stepsRemaining / (float)initialStepsScheduled;
      stepSpeed = minBrakeSpeed + (int)((currentSpeed - minBrakeSpeed) * remainingFraction);
      stepSpeed = constrain(stepSpeed, minBrakeSpeed, currentSpeed);
    }

    // Set direction & pulse
    digitalWrite(DIR_PIN, stepDir ? HIGH : LOW);
    digitalWrite(STEP_PIN, HIGH);
    stepPulseInProgress = true;
    nextStepTime = now + STEP_PULSE_WIDTH_US;

    // Update software position counters
    if (stepDir == 0) currentPosition++;
    else currentPosition--;
    stepsRemaining--;
    
    // Set interval for next step
    stepIntervalMicros = speedToIntervalMicros(stepSpeed);
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
    digitalWrite(ENABLE_PIN, ENABLE_ACTIVE_HIGH ? LOW : HIGH);
  }
  pinMode(PEND_PIN, INPUT_PULLUP);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  appendLog("=== PicoW Stepper Controller (Gear Reduction) ===");

  setupPins();
  updateGearReduction();
  updateMoveRangeFromAngle();

  // start AP (open network - no password)
  WiFi.softAP(ap_ssid);  // Call without password parameter for open network
  IPAddress ip = WiFi.softAPIP();
  appendLog("AP started: " + String(ap_ssid) + " (OPEN) IP: " + ip.toString());

  // Start DNS server for captive portal (redirects all DNS requests to our IP)
  dnsServer.start(DNS_PORT, "*", ip);
  appendLog("Captive portal DNS started");

  // web routes
  server.on("/", handleRoot);
  server.on("/log", handleLog);
  server.on("/cmd", handleCmd);
  
  // Captive portal detection endpoints
  server.on("/generate_204", handleRoot);  // Android
  server.on("/fwlink", handleRoot);        // Microsoft
  server.on("/hotspot-detect.html", handleRoot);  // Apple
  server.on("/connecttest.txt", handleRoot);  // Windows
  server.on("/success.txt", handleRoot);   // Firefox
  
  // Catch-all handler for captive portal - redirect everything else to root
  server.onNotFound([]() {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "-1");
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "Redirecting to captive portal");
  });
  server.begin();
  appendLog("Web server started");

  setMotorEnabled(false);
  appendLog("Ready. Connect to WiFi - portal opens automatically!");
  appendLog("Or open http://192.168.4.1 manually");
}

void loop() {
  dnsServer.processNextRequest();  // Handle DNS requests for captive portal
  server.handleClient();
  stepperTick();

  if (millis() - lastBootselCheck >= BOOTSEL_CHECK_INTERVAL) {
    lastBootselCheck = millis();
    checkBootselButton();
  }

  bool pendActive = (digitalRead(PEND_PIN) == LOW);

  // Continuous rotation mode: keep feeding new targets
  if (continuousMode && motorEnabled) {
    // If we're getting close to the target, schedule a new far-away target
    if (stepsRemaining < 1000) {
      long direction = (currentTarget > currentPosition) ? 1 : -1;
      currentTarget = currentPosition + (direction * 1000000);
      scheduleSteps(currentTarget - currentPosition);
    }
  }

  if (pendulumMode && motorEnabled) {
    if (!pendulumInitialized) {
      currentTarget = moveRange;
      goingRight = true;
      pendulumInitialized = true;
      motorWasDisabledAtRest = false;
      appendLog("→ Pendulum start to RIGHT: " + String(currentTarget));
      smartMoveTo(currentTarget);
    }

    long distance = abs(currentPosition - currentTarget);
    bool atTarget = (distance <= 2) || pendActive;

    if (atTarget && stepsRemaining == 0) {  // Check stepsRemaining to ensure motion complete
      if (!positionJustReached) {
        positionReachedTime = millis();
        positionJustReached = true;
        appendLog("Target reached. Position: " + String(currentPosition) + " Target: " + String(currentTarget));
        // Disable motor during rest if option enabled (only once per rest period)
        if (disableMotorAtRest && !motorWasDisabledAtRest) {
          setMotorEnabled(false);
          motorWasDisabledAtRest = true;
        }
      } else {
        if (millis() - positionReachedTime >= pendulumDelay) {
          // Re-enable motor before next move if it was disabled during rest
          if (disableMotorAtRest && motorWasDisabledAtRest) {
            appendLog("Rest period over, re-enabling motor for next swing");
            setMotorEnabled(true);
            delay(10);  // Give motor time to energize before moving
            motorWasDisabledAtRest = false;  // Reset flag for next rest period
          }
          
          // Switch direction
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

  delay(0);
}

//
// === WEB UI (root) ===
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
    input[type=number]{width:60px;padding:4px;background:#07070b;border:1px solid #222;color:#fff;border-radius:4px}
    .small{font-size:12px;color:var(--muted)}
    footer{margin-top:12px;color:var(--muted);font-size:12px}
    .status{padding:8px;background:#1a1a2e;border-radius:6px;margin-bottom:8px;font-size:13px}
    .gear-section{background:#1a1a2e;padding:10px;border-radius:6px;margin:10px 0}
  </style>
</head>
<body>
  <div class="wrap">
    <header><h1>⚙️ Pico Stepper Controller</h1></header>

    <div class="card">
      <div class="status">
        <strong>Status:</strong> Motor must be <strong>ENABLED</strong> to move!<br>
        <small>Press BOOTSEL button on Pico for >500ms to restart (if supported)</small>
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
            <button onclick="cmd('cw')" style="background:#ff9800">Spin CW ↻</button>
            <button onclick="cmd('ccw')" style="background:#ff9800">Spin CCW ↺</button>
          </div>

          <div class="gear-section">
            <label style="font-weight:bold;color:#7c4dff">⚙️ Gear Reduction</label>
            <div style="display:flex;gap:8px;align-items:center;margin-top:6px">
              <input type="number" id="drivingTeeth" value="20" min="1" max="200">
              <span>:</span>
              <input type="number" id="drivenTeeth" value="60" min="1" max="200">
              <button onclick="updateGear()" style="padding:6px 10px">Set</button>
            </div>
            <div style="font-size:11px;margin-top:4px;color:#888">
              Motor teeth : Arm teeth (e.g. 20:60 = 3:1 reduction)
            </div>
          </div>

          <div style="margin-top:12px">
            <div style="display:flex;gap:8px;margin-bottom:8px">
              <button onclick="setAngle(90)">Set 90°</button>
              <button onclick="setAngle(180)">Set 180°</button>
            </div>
            <div style="margin-bottom:8px">
              <button onclick="cmd('setneutral')" style="width:100%;background:#ff9800">Set Current as Neutral</button>
            </div>

            <label>Speed: <span id="speedVal"></span> sps</label>
            <input id="speed" type="range" min="50" max="4000" value="2000" oninput="updateSpeed(this.value)">
            <label>Arm Angle: <span id='angleVal'></span>° (arm motion, not motor)</label>
            <input id="angle" type="range" min="5" max="360" value="45" oninput="updateAngle(this.value)">
            <label>Rest Time: <span id='delayVal'></span> ms</label>
            <input id="delay" type="range" min="0" max="5000" step="50" value="250" oninput="updateDelay(this.value)">
            <div style="margin-top:8px">
              <label style="display:inline;margin-right:10px">
                <input type="checkbox" id="restToggle" onchange="toggleRest(this.checked)"> Disable motor at rest
              </label>
              <label style="display:inline;margin-right:10px">
                <input type="checkbox" id="brakeToggle" checked onchange="toggleBrake(this.checked)"> Enable braking
              </label>
              <label style="display:inline">
                <input type="checkbox" id="softStartToggle" checked onchange="toggleSoftStart(this.checked)"> Soft-start
              </label>
            </div>
            <div style="margin-top:8px">
              <label style="font-size:11px">Soft-start ramp: <span id='rampVal'></span> steps</label>
              <input id="rampsteps" type="range" min="0" max="500" value="150" oninput="updateRamp(this.value)" style="width:100%">
            </div>
            <div style="margin-top:8px;font-size:11px;color:#888">
              💡 Soft-start reduces initial torque. For whip: disable soft-start + braking for sharp motion.<br>
              ⚠️ Large angles auto-limit max speed for safety until JSS57 settings are tuned.
            </div>
            <div style="margin-top:8px">
              <input id="cmdtext" style="width:66%;padding:8px;border-radius:6px;border:1px solid #222;background:#07070b;color:#fff" placeholder="raw command">
              <button onclick="sendManual()">Send</button>
            </div>
          </div>
        </div>
      </div>
      <footer>
        Connect to <strong>Pico-Stepper</strong> WiFi (OPEN). Open <strong>http://192.168.4.1</strong><br>
        <small>Wiring: GP2=STEP, GP3=DIR, GP4=ENA, GP5=PEND | Gear reduction accounts for pulley ratio</small>
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
function setAngle(deg){ 
  document.getElementById('angle').value = deg; 
  document.getElementById('angleVal').textContent = deg; 
  fetch('/cmd?c='+encodeURIComponent('angle '+deg)).then(()=>setTimeout(refreshLog,120)); 
}
function updateGear(){
  let driving = document.getElementById('drivingTeeth').value;
  let driven = document.getElementById('drivenTeeth').value;
  fetch('/cmd?c='+encodeURIComponent('gear '+driving+':'+driven)).then(()=>setTimeout(refreshLog,150));
}
function updateRamp(v){ document.getElementById('rampVal').textContent=v; fetch('/cmd?c='+encodeURIComponent('rampsteps '+v)); }
function toggleSoftStart(checked){ cmd(checked ? 'softstarton' : 'softstartoff'); }
setInterval(refreshLog,500);
window.onload=function(){ 
  refreshLog(); 
  document.getElementById('speedVal').textContent=document.getElementById('speed').value; 
  document.getElementById('angleVal').textContent=document.getElementById('angle').value; 
  document.getElementById('delayVal').textContent=document.getElementById('delay').value;
  document.getElementById('rampVal').textContent=document.getElementById('rampsteps').value;
}
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}