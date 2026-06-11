/*
  Pico (non-W) — JSS57 Stepper Serial Controller
  - Serial USB control only (no WiFi)
  - Fast polling loop for smooth stepping
  - 5:1 gearbox support
  - GP2=STEP (PUL+), GP3=DIR (DIR+), GP4=ENA (ENA+), GP5=PEND (PEND+ input)
*/

#include <Arduino.h>

//
// === HARDWARE CONFIG ===
//
String serialBuffer = "";

const uint8_t STEP_PIN   = 2;   // PUL+ -> GP2
const uint8_t DIR_PIN    = 3;   // DIR+ -> GP3
const int8_t  ENABLE_PIN = 4;   // ENA+ -> GP4
const uint8_t PEND_PIN   = 5;   // PEND+ -> GP5 (input) ; PEND- -> GND

const bool ENABLE_ACTIVE_HIGH = false;  // JSS57 usually active-LOW

//
// === MOTOR / MOTION DEFAULTS ===
//
const int PULSES_PER_REV = 400;

// GEAR REDUCTION (5:1 gearbox)
int drivingTeeth = 1;
int drivenTeeth = 5;
float gearReduction = 5.0;

int currentAngle = 90;
const int minAngle = 5, maxAngle = 170;
int moveRange = 0;

int currentSpeed = 100;
int minSpeed = 50, maxSpeed = 3000;
int minBrakeSpeed = 50;
bool enableBraking = true;

unsigned long pendulumDelay = 500;
bool disableMotorAtRest = false;

//
// === STATE ===
//
volatile long currentPosition = 0;
volatile bool motorEnabled = false;

bool pendulumMode = false;
bool continuousMode = false;
long currentTarget = 0;
bool goingRight = true;
bool pendulumInitialized = false;
unsigned long positionReachedTime = 0;
bool positionJustReached = false;
bool motorWasDisabledAtRest = false;

//
// === STEPPER ENGINE ===
//
volatile long stepsRemaining = 0;
volatile bool stepDir = 0;
volatile unsigned long stepIntervalMicros = 1000;
const unsigned int STEP_PULSE_WIDTH_US = 4;
volatile bool stepPulseInProgress = false;
volatile unsigned long nextStepTime = 0;
volatile long initialStepsScheduled = 0;

int startupRampSteps = 150;
bool enableSoftStart = true;

//
// === UTILITIES ===
//
void printLog(const String &m) {
  Serial.println(m);
}

void updateGearReduction() {
  if (drivenTeeth > 0 && drivingTeeth > 0) {
    gearReduction = (float)drivenTeeth / (float)drivingTeeth;
    printLog("Gear ratio: " + String(drivingTeeth) + ":" + String(drivenTeeth) + 
              " (reduction " + String(gearReduction, 2) + ":1)");
  }
}

long armAngleToMotorSteps(float armAngle) {
  float motorAngle = armAngle * gearReduction;
  float stepsPerDegree = float(PULSES_PER_REV) / 360.0f;
  return (long)round(motorAngle * stepsPerDegree);
}

void updateMoveRangeFromAngle() {
  moveRange = armAngleToMotorSteps((float)currentAngle);
  float actualMotorAngle = currentAngle * gearReduction;
  
  printLog("Arm: " + String(currentAngle) + "° -> Motor: " + String(actualMotorAngle, 1) + 
            "° -> ±" + String(moveRange) + " steps");
}

unsigned long speedToIntervalMicros(int stepsPerSec) {
  if (stepsPerSec <= 0) return 1000000UL;
  double s = 1.0 / (double)stepsPerSec;
  return (unsigned long)round(s * 1e6);
}

//
// === MOTOR CONTROL ===
//
void setMotorEnabled(bool on) {
  motorEnabled = on;
  if (ENABLE_PIN >= 0) {
    bool pinState = ENABLE_ACTIVE_HIGH ? on : !on;
    digitalWrite(ENABLE_PIN, pinState ? HIGH : LOW);
  }
  printLog(String(on ? "MOTOR ON" : "MOTOR OFF"));
}

void scheduleSteps(long steps) {
  if (steps == 0) return;

  if (!motorEnabled) {
    printLog("Motor disabled, enabling...");
    setMotorEnabled(true);
    delay(10);
  }

  noInterrupts();
  if (steps > 0) {
    stepDir = 0;
    stepsRemaining = steps;
  } else {
    stepDir = 1;
    stepsRemaining = -steps;
  }

  initialStepsScheduled = stepsRemaining;
  stepIntervalMicros = speedToIntervalMicros(currentSpeed);
  nextStepTime = micros();
  interrupts();
}

void smartMoveTo(long target) {
  if (!motorEnabled) {
    printLog("Enabling motor for move");
    setMotorEnabled(true);
    delay(2);
  }

  long delta = target - currentPosition;
  if (delta == 0) {
    // Already at target — signal completion so wait_pend doesn't hang
    printLog("Target reached. Pos: " + String(currentPosition));
    return;
  }
  printLog("Scheduling " + String(delta) + " steps to " + String(target));
  scheduleSteps(delta);
}

void emergencyStop() {
  noInterrupts();
  stepsRemaining = 0;
  interrupts();
  printLog("EMERGENCY STOP");
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
    
    // SOFT START — ramp up over first startupRampSteps
    if (enableSoftStart && stepsCompleted < startupRampSteps && initialStepsScheduled > startupRampSteps) {
      float rampFraction = (float)stepsCompleted / (float)startupRampSteps;
      stepSpeed = minBrakeSpeed + (int)((currentSpeed - minBrakeSpeed) * rampFraction);
      stepSpeed = constrain(stepSpeed, minBrakeSpeed, currentSpeed);
    }
    // BRAKING — ramp down over last startupRampSteps only (not the whole move)
    else if (enableBraking && stepsRemaining < startupRampSteps && initialStepsScheduled > startupRampSteps) {
      float brakeFraction = (float)stepsRemaining / (float)startupRampSteps;
      stepSpeed = minBrakeSpeed + (int)((currentSpeed - minBrakeSpeed) * brakeFraction);
      stepSpeed = constrain(stepSpeed, minBrakeSpeed, currentSpeed);
    }

    digitalWrite(DIR_PIN, stepDir ? HIGH : LOW);
    digitalWrite(STEP_PIN, HIGH);
    stepPulseInProgress = true;
    nextStepTime = now + STEP_PULSE_WIDTH_US;

    if (stepDir == 0) currentPosition++;
    else currentPosition--;
    stepsRemaining--;
    
    stepIntervalMicros = speedToIntervalMicros(stepSpeed);
  }
}

//
// === SERIAL COMMAND HANDLER ===
//
void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        serialBuffer.trim();
        serialBuffer.toLowerCase();
        
        String cmd = serialBuffer;
        printLog("> " + cmd);
        
        // Commands
        if (cmd == "on") { 
          setMotorEnabled(true); 
        }
        else if (cmd == "off") { 
          setMotorEnabled(false); 
          pendulumMode = false; 
          continuousMode = false; 
        }
        else if (cmd == "stop") {
          emergencyStop();
          continuousMode = false;
          pendulumMode = false;
        }
        else if (cmd == "reset") {
          // Clear driver alarm: stop motion, toggle enable pin, re-enable.
          // Remove any mechanical obstruction before sending this.
          emergencyStop();
          continuousMode = false;
          pendulumMode = false;
          setMotorEnabled(false);
          delay(300);
          currentPosition = 0;  // re-zero since position may have been lost
          currentTarget   = 0;
          setMotorEnabled(true);
          printLog("Motor reset. Position zeroed.");
        }
        else if (cmd == "left") {
          if (motorEnabled) { 
            currentTarget = moveRange; 
            smartMoveTo(currentTarget); 
          } else {
            printLog("Motor disabled");
          }
        }
        else if (cmd == "right") {
          if (motorEnabled) { 
            currentTarget = -moveRange; 
            smartMoveTo(currentTarget); 
          } else {
            printLog("Motor disabled");
          }
        }
        else if (cmd == "neutral") {
          if (motorEnabled) { 
            smartMoveTo(0); 
          } else {
            printLog("Motor disabled");
          }
        }
        else if (cmd == "pend") {
          if (motorEnabled) {
            pendulumMode = !pendulumMode;
            pendulumInitialized = false;
            continuousMode = false;
            printLog(String(pendulumMode ? "PENDULUM ON" : "PENDULUM OFF"));
          } else {
            printLog("Motor disabled");
          }
        }
        else if (cmd == "cw") {
          if (motorEnabled) {
            continuousMode = true;
            pendulumMode = false;
            long largeTarget = currentPosition + 1000000;
            smartMoveTo(largeTarget);
          } else {
            printLog("Motor disabled");
          }
        }
        else if (cmd == "ccw") {
          if (motorEnabled) {
            continuousMode = true;
            pendulumMode = false;
            long largeTarget = currentPosition - 1000000;
            smartMoveTo(largeTarget);
          } else {
            printLog("Motor disabled");
          }
        }
        else if (cmd.startsWith("speed ")) {
          int v = cmd.substring(6).toInt();
          currentSpeed = constrain(v, minSpeed, maxSpeed);
          printLog("Speed: " + String(currentSpeed) + " sps");
        }
        else if (cmd.startsWith("angle ")) {
          int v = cmd.substring(6).toInt();
          currentAngle = constrain(v, minAngle, maxAngle);
          updateMoveRangeFromAngle();
        }
        else if (cmd.startsWith("delay ")) {
          int v = cmd.substring(6).toInt();
          pendulumDelay = constrain(v, 0, 5000);
          printLog("Pendulum delay: " + String(pendulumDelay) + "ms");
        }
        else if (cmd.startsWith("gear ")) {
          String gearStr = cmd.substring(5);
          int colonIdx = gearStr.indexOf(':');
          if (colonIdx > 0) {
            int driving = gearStr.substring(0, colonIdx).toInt();
            int driven = gearStr.substring(colonIdx + 1).toInt();
            if (driving > 0 && driven > 0) {
              drivingTeeth = driving;
              drivenTeeth = driven;
              updateGearReduction();
              updateMoveRangeFromAngle();
            }
          }
        }
        else if (cmd.startsWith("move ")) {
          if (!motorEnabled) { setMotorEnabled(true); }
          long pos = cmd.substring(5).toInt();
          smartMoveTo(pos);
        }
        else if (cmd.startsWith("offset ")) {
          if (!motorEnabled) { setMotorEnabled(true); }
          long off = cmd.substring(7).toInt();
          smartMoveTo(currentPosition + off);
        }
        else if (cmd == "brakeon") { 
          enableBraking = true; 
          printLog("Braking ON"); 
        }
        else if (cmd == "brakeoff") { 
          enableBraking = false; 
          printLog("Braking OFF"); 
        }
        else if (cmd == "softstarton") { 
          enableSoftStart = true; 
          printLog("Soft-start ON"); 
        }
        else if (cmd == "softstartoff") { 
          enableSoftStart = false; 
          printLog("Soft-start OFF"); 
        }
        else if (cmd == "calibrate") {
          long shift = currentPosition;
          currentPosition = 0;
          currentTarget -= shift;
          printLog("Calibrated. Current = 0");
        }
        else if (cmd == "status") {
          printLog("=== STATUS ===");
          printLog("Position: " + String(currentPosition));
          printLog("Target: " + String(currentTarget));
          printLog("Remaining: " + String(stepsRemaining));
          printLog("Motor: " + String(motorEnabled ? "ON" : "OFF"));
          printLog("Speed: " + String(currentSpeed) + " sps");
          printLog("Angle: " + String(currentAngle) + "°");
          printLog("Gear: " + String(drivingTeeth) + ":" + String(drivenTeeth));
        }
        else if (cmd == "help") {
          printLog("=== COMMANDS ===");
          printLog("on, off, stop");
          printLog("left, right, neutral");
          printLog("pend, cw, ccw");
          printLog("speed <n>, angle <n>, delay <n>");
          printLog("move <steps>, offset <steps>");
          printLog("gear <d>:<r>");
          printLog("brakeon, brakeoff");
          printLog("softstarton, softstartoff");
          printLog("calibrate, status, help");
        }
        else {
          printLog("Unknown: " + cmd + " (type 'help')");
        }
        
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }
}

//
// === SETUP ===
//
void setup() {
  Serial.begin(115200);
  delay(200);
  
  // Setup pins
  pinMode(STEP_PIN, OUTPUT); digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);  digitalWrite(DIR_PIN, LOW);
  if (ENABLE_PIN >= 0) {
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, ENABLE_ACTIVE_HIGH ? LOW : HIGH);
  }
  pinMode(PEND_PIN, INPUT_PULLUP);

  updateGearReduction();
  updateMoveRangeFromAngle();
  
  printLog("=== Pico Stepper Controller ===");
  printLog("Serial control ready");
  printLog("Type 'help' for commands");
  
  setMotorEnabled(false);
}

//
// === LOOP ===
//
void loop() {
  // Call stepperTick as fast as possible for smooth stepping
  stepperTick();
  
  // Handle serial commands (but don't let it block stepping)
  if (Serial.available() > 0) {
    handleSerialCommands();
  }

  // Read volatile variables safely for logic
  static unsigned long lastLogicCheck = 0;
  if (micros() - lastLogicCheck >= 1000) {  // Check logic every 1ms
    lastLogicCheck = micros();
    
    long localStepsRemaining;
    long localCurrentPosition;
    noInterrupts();
    localStepsRemaining = stepsRemaining;
    localCurrentPosition = currentPosition;
    interrupts();

    // Continuous rotation mode
    if (continuousMode && motorEnabled) {
      if (localStepsRemaining < 1000) {
        long direction = (currentTarget > localCurrentPosition) ? 1 : -1;
        currentTarget = localCurrentPosition + (direction * 1000000);
        scheduleSteps(currentTarget - localCurrentPosition);
      }
    }

    // Pendulum mode
    if (pendulumMode && motorEnabled) {
      if (!pendulumInitialized) {
        currentTarget = moveRange;
        goingRight = true;
        pendulumInitialized = true;
        motorWasDisabledAtRest = false;
        printLog("Pendulum start RIGHT: " + String(currentTarget));
        smartMoveTo(currentTarget);
      }

      long distance = abs(localCurrentPosition - currentTarget);
      bool atTarget = (distance <= 2);

      if (atTarget && localStepsRemaining == 0) {
        if (!positionJustReached) {
          positionReachedTime = millis();
          positionJustReached = true;
          printLog("Target reached. Pos: " + String(localCurrentPosition));

          if (disableMotorAtRest && !motorWasDisabledAtRest) {
            setMotorEnabled(false);
            motorWasDisabledAtRest = true;
          }
        } else {
          if (millis() - positionReachedTime >= pendulumDelay) {
            if (disableMotorAtRest && motorWasDisabledAtRest) {
              printLog("Re-enabling motor");
              setMotorEnabled(true);
              delay(10);
              motorWasDisabledAtRest = false;
            }

            // Switch direction
            if (goingRight) currentTarget = -moveRange;
            else currentTarget = moveRange;
            goingRight = !goingRight;
            positionJustReached = false;
            printLog(String(goingRight ? "Moving RIGHT: " : "Moving LEFT: ") + String(currentTarget));
            smartMoveTo(currentTarget);
          }
        }
      }
    }

    // Detect move completion for direct commands (left/right/move/offset).
    // Pendulum mode prints its own "Target reached" above.
    static bool prevMoving = false;
    bool nowMoving = (localStepsRemaining > 0);
    if (!pendulumMode && !continuousMode && prevMoving && !nowMoving && motorEnabled) {
      printLog("Target reached. Pos: " + String(localCurrentPosition));
    }
    prevMoving = nowMoving;
  }
}