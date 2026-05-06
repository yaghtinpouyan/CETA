// CETA_V3.ino
// Improved line-following V3 based on CETA_V1
// - Uses QTRSensors readLineBlack (0..3000, center ~1500)
// - PID implemented with floats, integral windup guard, correct derivative
// - Maps PID correction to servo microseconds around stop (1500us)
// - Non-blocking turn state machine (no busy-wait)
// - Simple serial tuning and debug prints
// - Calibration hooks left in; does not overwrite V1/V2

#include <elapsedMillis.h>
#include <Servo.h>
#include <QTRSensors.h>

// --- Configuration (matched to your wiring) ---
const int pinCalibrateButton = 19; // INPUT_PULLUP (user)
const int pinStartButton = 21;     // INPUT_PULLUP (user)
const int pinResetButton = 20;     // INPUT_PULLUP (user)

const int qtrSensorL = 28; // left (user)
const int qtrSensorM = 27; // middle (user)
const int qtrSensorR = 26; // right (user)

const int trigPin = 2;  // ultrasonic trig (user)
const int echoPin = 3;  // ultrasonic echo (user)

const int servoLpin = 15; // left drive servo (user)
const int servoRpin = 14; // right drive servo (user)

// Servo microsecond baseline values measured on your robot
// Left servo is reversed (lower microseconds -> forward). Right is normal (higher -> forward)
const int BASE_US_LEFT = 1300;  // baseline forward-ish microseconds for left (user-provided approx)
const int BASE_US_RIGHT = 1700; // baseline forward-ish microseconds for right (user-provided approx)
const int SERVO_US_MIN = 1000;  // absolute min safe microseconds
const int SERVO_US_MAX = 2000;  // absolute max safe microseconds
const int SERVO_US_STOP = 1500; // universal stop microsecond (center)
const int SERVO_US_MAX_DELTA = 500; // max delta from stop

// QTR / PID constants (start conservative and tune)
float Kp = 0.8; // proportional
float Ki = 0.0; // integral
float Kd = 0.4; // derivative

// PID state
float pidP = 0, pidI = 0, pidD = 0;
float lastError = 0;
unsigned long lastPidTime = 0;
float pidOutput = 0;
const float I_MAX = 500.0; // cap integral term to avoid windup

// QTR sensor object
QTRSensors qtr;
uint16_t sensors[3];
int16_t position = 1500; // 0..3000, center ~1500

// Servos
Servo servoL;
Servo servoR;

// State machine
enum RobotState { IDLE, RUNNING, TURNING };
RobotState state = IDLE;

// Turn handling
elapsedMillis turnTimer;
bool turnRequested = false;

// Misc
elapsedMillis loopTimer;
elapsedMillis calibrateTimer;
bool calibrated = false;
int lineCrossed = -1;
int requirement = 4; // laps/turns requirement

// Debug
bool debugPrint = true;

// --- Helper functions ---
float clampf(float v, float a, float b) {
  if (v < a) return a;
  if (v > b) return b;
  return v;
}

void applyMotorOutput(float correction) {
  // correction is in PID units; positive -> steer right
  // We'll apply correction as microsecond offset around per-servo base values.
  // Left servo is reversed: to increase forward we decrease microseconds.

  // Scale correction to microseconds (tunable multiplier)
  int corrUs = (int)round(clampf(correction * 0.6f, -SERVO_US_MAX_DELTA, SERVO_US_MAX_DELTA));

  // Apply correction: left is reversed, so subtract correction; right normal so add correction
  int outL = BASE_US_LEFT - corrUs;
  int outR = BASE_US_RIGHT + corrUs;

  // Clamp to safe physical bounds
  outL = constrain(outL, SERVO_US_MIN, SERVO_US_MAX);
  outR = constrain(outR, SERVO_US_MIN, SERVO_US_MAX);

  servoL.writeMicroseconds(outL);
  servoR.writeMicroseconds(outR);

  if (debugPrint) {
    Serial.print("pos="); Serial.print(position);
    Serial.print(" err="); Serial.print(lastError);
    Serial.print(" corrUs="); Serial.print(corrUs);
    Serial.print(" L="); Serial.print(outL);
    Serial.print(" R="); Serial.println(outR);
  }
}

void computePID() {
  unsigned long now = micros();
  float dt = (lastPidTime == 0) ? 0.01f : (now - lastPidTime) / 1000000.0f; // seconds
  if (dt <= 0) dt = 0.01f;

  // QTR position: 0..3000, center ~1500
  float error = 1500.0f - (float)position; // positive => line is to the right (we'll steer right)

  pidP = error;
  pidI += error * dt;
  pidI = clampf(pidI, -I_MAX, I_MAX);
  pidD = (error - lastError) / dt;

  pidOutput = Kp * pidP + Ki * pidI + Kd * pidD;

  lastError = error;
  lastPidTime = now;
}

void requestTurn180() {
  if (state == TURNING) return;
  state = TURNING;
  turnTimer = 0;
  turnRequested = true;
  lineCrossed++;
}

void doNonBlockingTurn() {
  // Gentle pivot turn: one wheel moves (forward), the other stays near stop to pivot around it.
  // Use left wheel forward and right wheel stopped (or slight reverse if needed).

  if (!turnRequested) {
    state = RUNNING;
    return;
  }

  // Phase 1: initial pivot burst to start rotating
  if (turnTimer < 250) {
    // Left forward (remember left is reversed: lower us -> forward)
    int leftUs = constrain(BASE_US_LEFT - 300, SERVO_US_MIN, SERVO_US_MAX);
    int rightUs = SERVO_US_STOP; // keep right near stop for pivot
    servoL.writeMicroseconds(leftUs);
    servoR.writeMicroseconds(rightUs);
    return;
  }

  // Phase 2: continue pivot slower while checking sensors
  int16_t pos = qtr.readLineBlack(sensors);
  position = pos;

  // If centered on the line, finish the turn
  if (pos >= 1400 && pos <= 1600) {
    // Nudge forward a bit to clear intersection
    servoL.writeMicroseconds(BASE_US_LEFT - 120);
    servoR.writeMicroseconds(BASE_US_RIGHT + 120);
    delay(80);
    // stop
    servoL.writeMicroseconds(SERVO_US_STOP);
    servoR.writeMicroseconds(SERVO_US_STOP);

    turnRequested = false;
    state = RUNNING;
    return;
  }

  // Keep pivoting slowly
  int leftUs = constrain(BASE_US_LEFT - 220, SERVO_US_MIN, SERVO_US_MAX);
  servoL.writeMicroseconds(leftUs);
  servoR.writeMicroseconds(SERVO_US_STOP);
}

void setup() {
  Serial.begin(115200);
  pinMode(pinCalibrateButton, INPUT_PULLUP);
  pinMode(pinStartButton, INPUT_PULLUP);
  pinMode(pinResetButton, INPUT_PULLUP);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  servoL.attach(servoLpin);
  servoR.attach(servoRpin);
  servoL.writeMicroseconds(SERVO_US_STOP);
  servoR.writeMicroseconds(SERVO_US_STOP);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){qtrSensorL, qtrSensorM, qtrSensorR}, 3);

  lastPidTime = micros();

  Serial.println("CETA_V3 ready. Press start button to begin.");
}

void loop() {
  // Calibration: if calibrate button held for >2s, run calibration
  if (digitalRead(pinCalibrateButton) == LOW) {
    if (calibrateTimer > 2000) {
      Serial.println("Starting QTR calibration: sweep sensors over BLACK for 10s, then WHITE for 10s...");
      calibrateTimer = 0;
      qtr.resetCalibration();

      unsigned long startCal = millis();
      while (millis() - startCal < 10000UL) {
        qtr.calibrate();
        if ((millis() - startCal) % 500 < 250) digitalWrite(LED_BUILTIN, HIGH);
        else digitalWrite(LED_BUILTIN, LOW);
        delay(20);
      }
      startCal = millis();
      while (millis() - startCal < 10000UL) {
        qtr.calibrate();
        if ((millis() - startCal) % 250 < 125) digitalWrite(LED_BUILTIN, HIGH);
        else digitalWrite(LED_BUILTIN, LOW);
        delay(20);
      }
      digitalWrite(LED_BUILTIN, LOW);
      calibrated = true;
      Serial.println("Calibration complete.");
    }
  } else {
    calibrateTimer = 0;
  }

  // Start/reset buttons
  if (digitalRead(pinStartButton) == LOW) {
    state = RUNNING;
    Serial.println("Starting run.");
    delay(200); // simple debounce
  }
  if (digitalRead(pinResetButton) == LOW) {
    state = IDLE;
    lineCrossed = -1;
    requirement = 4;
    pidI = 0;
    Serial.println("Reset.");
    delay(200);
  }

  // Main state machine
  if (state == IDLE) {
    // ensure motors stopped
    servoL.writeMicroseconds(SERVO_US_STOP);
    servoR.writeMicroseconds(SERVO_US_STOP);
    return;
  }

  if (state == TURNING) {
    doNonBlockingTurn();
    return; // during turning we update sensors inside doNonBlockingTurn
  }

  // RUNNING state: normal line follow
  position = qtr.readLineBlack(sensors);
  computePID();

  // Convert pidOutput to microsecond correction. Tunable scale.
  // We scale pidOutput to match servo microsecond range; you can adjust the multiplier.
  float correctionUs = pidOutput * 0.6; // scaling factor; tune as needed

  // constrain correction
  correctionUs = clampf(correctionUs, -SERVO_US_MAX_DELTA, SERVO_US_MAX_DELTA);

  // Apply motor outputs around stop
  applyMotorOutput(correctionUs);

  // Detect full-line crossing (all sensors see dark)
  if ((sensors[0] > 950) && (sensors[1] > 950) && (sensors[2] > 950)) {
    // simple debounce with elapsedMillis
    if (!turnRequested) {
      requestTurn180();
    }
  }

  // Optional: check ultrasonic (left as future addition)

  // Small loop delay to control update rate — tune as required
  delay(5);
}
