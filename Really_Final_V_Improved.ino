
// CETA_V1 - simplified logic with stability improvements
// Based on nishant-pls-clutch_simplified.ino

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
int previousState = HIGH; // initialize to HIGH to match INPUT_PULLUP idle state
unsigned long pressStartTime = 0;
const long holdDuration = 3000; // 3s hold
bool running = false; // holds whether robot is running

// Threshold (configurable)
const int BLACK_THRESHOLD = 1000;

// Move state tracking
const int MOVE_NONE = 0;
const int MOVE_FORWARD = 1;
const int MOVE_TURN_AROUND = 2;
const int MOVE_RIGHT = 3;
const int MOVE_LEFT = 4;

int lastMove = MOVE_NONE;

// Track whether we've already handled the first all-black intersection
bool firstIntersectionHandled = false;

// Edge-detect the all-black condition so we only trigger on entry
bool previouslyAllAbove = false;

// Recovery timeout to avoid oscillation
bool inRecovery = false;
unsigned long recoveryStart = 0;
const unsigned long recoveryTimeout = 400; // ms to continue same turning before reversing

// Helper to reset move flags (we primarily use lastMove integer)
void clearMoveFlags() {
	lastMove = MOVE_NONE;
}

// Movement helpers
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

// Minimal calibration placeholder
void calibrate_sensor() {
	Serial.println("Calibrating (placeholder)...");
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

	// Read actual pin state at startup to avoid false transition
	previousState = digitalRead(startButton);

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

	// Start/stop toggle behavior (press to start/stop)
	buttonState = digitalRead(startButton); // LOW when pressed
	if (buttonState == LOW && previousState == HIGH) {
		pressStartTime = millis();
	}
	if (buttonState == HIGH && previousState == LOW) {
		unsigned long pressDuration = millis() - pressStartTime;
		if (pressDuration < holdDuration) {
			running = !running;
		}
	}
	previousState = buttonState;

	// Reset button: force stop and clear states
	if (digitalRead(resetButton) == LOW) {
		running = false;
		clearMoveFlags();
		inRecovery = false;
		firstIntersectionHandled = false;
		previouslyAllAbove = false;
		robot_stop();
		delay(200);
		return;
	}

	if (!running) {
		robot_stop();
		return;
	}

	// Decision logic
	bool allAbove = (leftVal >= BLACK_THRESHOLD && centerVal >= BLACK_THRESHOLD && rightVal >= BLACK_THRESHOLD);

	// If we're not on an all-black area, clear the previous-all flag so we detect the next rising edge
	if (!allAbove) {
		previouslyAllAbove = false;
	}

	// Only trigger intersection handling when we JUST entered an all-black region
	if (allAbove && !previouslyAllAbove) {
		previouslyAllAbove = true; // mark we've seen the rising edge
		if (!firstIntersectionHandled) {
			// first time encountering all-black intersection
			robot_forward();
			lastMove = MOVE_FORWARD;
			firstIntersectionHandled = true;
			inRecovery = false;
			Serial.println("ALL BLACK: forward (first)");
			return;
		} else {
			// subsequent all-black -> turn around
			turn_around();
			lastMove = MOVE_TURN_AROUND;
			inRecovery = false;
			Serial.println("ALL BLACK: turn around (subsequent)");
			return;
		}
	
	} else if (rightVal >= BLACK_THRESHOLD) {
		turn_right();
		lastMove = MOVE_RIGHT;
		inRecovery = false;
		Serial.println("RIGHT sensor active: turning right");
		return;
	} else if (leftVal >= BLACK_THRESHOLD) {
		turn_left();
		lastMove = MOVE_LEFT;
		inRecovery = false;
		Serial.println("LEFT sensor active: turning left");
		return;
	} else if (centerVal >= BLACK_THRESHOLD) {
		robot_forward();
		lastMove = MOVE_FORWARD;
		inRecovery = false;
		Serial.println("CENTER sensor active: moving forward");
		return;
	} else {
		// All sensors below threshold: recovery behavior with timeout to avoid oscillation
		if (lastMove == MOVE_RIGHT) {
			if (!inRecovery) {
				// start recovery by continuing the same turn for a short period
				inRecovery = true;
				recoveryStart = millis();
				turn_right();
				Serial.println("RECOVER start (right): continue turning right");
				return;
			} else {
				// already in recovery; check timeout
				if (millis() - recoveryStart < recoveryTimeout) {
					turn_right();
					return;
				} else {
					// reverse direction after timeout
					turn_left();
					lastMove = MOVE_LEFT;
					inRecovery = false;
					Serial.println("RECOVER timeout (right->left): turning left");
					return;
				}
			}
		} else if (lastMove == MOVE_LEFT) {
			if (!inRecovery) {
				inRecovery = true;
				recoveryStart = millis();
				turn_left();
				Serial.println("RECOVER start (left): continue turning left");
				return;
			} else {
				if (millis() - recoveryStart < recoveryTimeout) {
					turn_left();
					return;
				} else {
					turn_right();
					lastMove = MOVE_RIGHT;
					inRecovery = false;
					Serial.println("RECOVER timeout (left->right): turning right");
					return;
				}
			}
		} else if (lastMove == MOVE_FORWARD) {
			// if last was forward, try turning around to relocate the line
			turn_around();
			lastMove = MOVE_TURN_AROUND;
			inRecovery = false;
			Serial.println("RECOVER from forward: turning around");
			return;
		} else if (lastMove == MOVE_TURN_AROUND) {
			// keep turning around until a sensor sees the line
			turn_around();
			Serial.println("Continuing to turn around");
			return;
		} else {
			// No previous move recorded
			robot_stop();
			Serial.println("DEFAULT: stop (no previous move)");
			return;
		}
	}
}

