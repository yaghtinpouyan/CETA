// Simplified version of nishant-pls-clutch.ino
// Keeps calibration (hold calibrate), start/stop/reset behavior,
// and movement helper functions. Replaces loop() with the requested
// simplified decision logic and minimal variables.

#include <Servo.h>
#include <QTRSensors.h>

Servo servoL;
Servo servoR;

// Sensor pins
const int sensorL = 28;
const int sensorM = 27;
const int sensorR = 26;

QTRSensors qtr;
uint16_t sensors[3];
int leftVal, centerVal, rightVal;

// Buttons
int calibrateButton = 19;
int resetButton     = 20;
int startButton     = 21;

int buttonState = 0;
int previousState = 0;
unsigned long pressStartTime = 0;
const long holdDuration = 3000; // 3s hold
bool running = false; // holds whether robot is running

// New simplified state variables
int countLines = 0; // 'count' in user's description
bool moving_forward = false;
bool turning_around = false;
bool turning_right = false;
bool turning_left = false;

// Helper to reset move flags
void clearMoveFlags() {
  moving_forward = false;
  turning_around = false;
  turning_right = false;
  turning_left = false;
}

// Movement helpers (copied from original)
void robot_forward() {
  servoL.writeMicroseconds(1300);
  servoR.writeMicroseconds(1800);
}

void robot_stop() {
  servoL.writeMicroseconds(1500);
  servoR.writeMicroseconds(1500);
}

void turn_right() {
  servoL.writeMicroseconds(1350);
  servoR.writeMicroseconds(1800);
}

void turn_left() {
  servoL.writeMicroseconds(1800);
  servoR.writeMicroseconds(1350);
}

void turn_around() {
  servoL.writeMicroseconds(1800);
  servoR.writeMicroseconds(1200);
}

// Minimal calibration placeholder: user wanted hold to calibrate like before.
// We'll keep a simple calibration routine that reads sensors a few times and prints values.
void calibrate_sensor() {
  Serial.println("Calibrating (placeholder)...");
  // In this simplified file, we won't store calibration values.
  // A real calibration could populate thresholds or QTR calibration data.
  for (int i = 0; i < 10; i++) {
    qtr.read(sensors);
    Serial.print("cal: ");
    Serial.print(sensors[0]); Serial.print(',');
    Serial.print(sensors[1]); Serial.print(',');
    Serial.println(sensors[2]);
    delay(50);
  }
  Serial.println("Calibration complete.");
}

void setup() {
  Serial.begin(115200);

  pinMode(calibrateButton, INPUT_PULLUP);
  pinMode(startButton,     INPUT_PULLUP);
  pinMode(resetButton,     INPUT_PULLUP);

  servoL.attach(15);
  servoR.attach(14);
  robot_stop();

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){sensorL, sensorM, sensorR}, 3);
}

void loop() {
  // Read sensors
  qtr.read(sensors);
  leftVal   = sensors[0];
  centerVal = sensors[1];
  rightVal  = sensors[2];

  // Handle calibrate (hold 3s)
  if (digitalRead(calibrateButton) == LOW) {
    unsigned long start = millis();
    while (digitalRead(calibrateButton) == LOW) {
      if (millis() - start >= holdDuration) {
        calibrate_sensor();
        break;
      }
      delay(10);
    }
  }

  // Start/stop toggle behavior (press to start, press again to stop)
  buttonState = digitalRead(startButton); // LOW when pressed
  if (buttonState == LOW && previousState == HIGH) {
    pressStartTime = millis();
    // short press toggles running
  }
  if (buttonState == HIGH && previousState == LOW) {
    unsigned long pressDuration = millis() - pressStartTime;
    if (pressDuration < holdDuration) {
      running = !running;
    } else {
      // long press handled elsewhere if needed
    }
  }
  previousState = buttonState;

  // Reset button: force stop and clear states
  if (digitalRead(resetButton) == LOW) {
    running = false;
    countLines = 0;
    clearMoveFlags();
    robot_stop();
    delay(200); // debounce
    return;
  }

  if (!running) {
    robot_stop();
    return;
  }

  // Decision logic per user's specification
  // Treat sensor values >= 1000 as 'on black'
  bool allAbove = (leftVal >= 1000 && centerVal >= 1000 && rightVal >= 1000);

  if (allAbove && countLines == 0) {
    // first time encountering all-black
    robot_forward();
    countLines++;
    clearMoveFlags();
    moving_forward = true;
    Serial.println("ALL BLACK: forward (count 0 -> 1)");
    return;
  } else if (allAbove && countLines > 0) {
    // subsequent all-black -> turn around
    turn_around();
    clearMoveFlags();
    turning_around = true;
    Serial.println("ALL BLACK: turn around (count > 0)");
    return;
  } else if (rightVal >= 1000) {
    turn_right();
    clearMoveFlags();
    turning_right = true;
    Serial.println("RIGHT sensor active: turning right");
    return;
  } else if (leftVal >= 1000) {
    turn_left();
    clearMoveFlags();
    turning_left = true;
    Serial.println("LEFT sensor active: turning left");
    return;
  } else if (centerVal >= 1000) {
    robot_forward();
    clearMoveFlags();
    moving_forward = true;
    Serial.println("CENTER sensor active: moving forward");
    return;
  } else {
    // All sensors below 1000: decide based on previous move
    if (turning_right) {
      // if last was turning right, now do opposite -> turn left
      turn_left();
      clearMoveFlags();
      turning_left = true;
      Serial.println("RECOVER from right: turning left");
      return;
    } else if (turning_left) {
      turn_right();
      clearMoveFlags();
      turning_right = true;
      Serial.println("RECOVER from left: turning right");
      return;
    } else if (moving_forward) {
      turn_around();
      clearMoveFlags();
      turning_around = true;
      Serial.println("RECOVER from forward: turning around");
      return;
    } else if (turning_around) {
      // keep turning around
      turn_around();
      Serial.println("Continuing to turn around");
      return;
    } else {
      // default fallback
      robot_stop();
      Serial.println("DEFAULT: stop (no previous move)");
      return;
    }
  }
}
