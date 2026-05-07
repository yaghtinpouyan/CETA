// CETA 2025
// Hardware: Servos on pins 14/15, QTR analog IR sensors on pins 26/27/28
// Logic structure adapted from Devious Birds (Challenge 1: Running The Fairway)

#include <elapsedMillis.h>
elapsedMillis TaskTimer;
#include <Servo.h>
#include <QTRSensors.h>

// ── Servos ──────────────────────────────────────────────────────────────────
Servo servoL;
Servo servoR;

// ── IR Sensor Pins ───────────────────────────────────────────────────────────
const int sensorL = 28;
const int sensorM = 27;
const int sensorR = 26;

QTRSensors qtr;
uint16_t sensors[3];

int leftVal, centerVal, rightVal;

// ── Buttons ──────────────────────────────────────────────────────────────────
int calibrateButton = 19;
int resetButton     = 20;
int startButton     = 21;

// ── Button / toggle state ────────────────────────────────────────────────────
int buttonState    = 0;
int previousState  = 0;
unsigned long pressStartTime = 0;
const long holdDuration = 3000;
bool toggleState = false; // false = stopped, true = running

// ── Counters ─────────────────────────────────────────────────────────────────
int lineCounter  = 0;
int counter      = 0;      // remembers last turn direction (1=left, 2=right)
bool inBlackArea = false;

// ── US Sensor ────────────────────────────────────────────────────────────────
const int trigpin = 2;
const int echopin = 3;
volatile unsigned long echoStart = 0;
volatile unsigned long echoEnd   = 0;
volatile bool echoReady = false;
float distance = 0.0;
elapsedMillis trigTimer;

void echoISR() {
  if (gpio_get(echopin)) {
    echoStart = micros();
  } else {
    echoEnd   = micros();
    echoReady = true;
  }
}

// ── Adafruit IO (IoT challenge placeholder) ───────────────────────────────────
#define IO_USERNAME  "placeholder"
#define IO_KEY       "placeholder"
#define WIFI_SSID    "your_wifi_name"
#define WIFI_PASS    "your_wifi_password"

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(calibrateButton, INPUT_PULLUP);
  pinMode(startButton,     INPUT_PULLUP);
  pinMode(resetButton,     INPUT_PULLUP);
  pinMode(LED_BUILTIN,     OUTPUT);

  pinMode(echopin, INPUT);
  pinMode(trigpin, OUTPUT);
  digitalWrite(trigpin, LOW);
  attachInterrupt(digitalPinToInterrupt(echopin), echoISR, CHANGE);

  servoL.attach(15);
  servoR.attach(14);
  servoL.write(90);   // neutral / stopped
  servoR.write(90);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){sensorL, sensorM, sensorR}, 3);
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  // Read raw sensor values
  qtr.read(sensors);
  leftVal   = sensors[0];
  centerVal = sensors[1];
  rightVal  = sensors[2];

  // Debug
    Serial.println("Line Counter: ");
    // Serial.print(lineCounter);
    // Serial.print(" Position: (");
    Serial.print(leftVal);
    Serial.print(", ");
    Serial.print(centerVal);
    Serial.print(", ");
    Serial.print(rightVal);
    Serial.print(")");

  // ── Start button: tap = toggle run/stop, hold 3s = calibrate ────────────
  buttonState = digitalRead(startButton);    // LOW when pressed (INPUT_PULLUP)

  if (buttonState == LOW && previousState == HIGH) {
    pressStartTime = millis();
    toggleState = true;
  }
  if (buttonState == HIGH && previousState == LOW) {
    unsigned long pressDuration = millis() - pressStartTime;
    if (pressDuration >= holdDuration) {
      detachInterrupt(digitalPinToInterrupt(echopin));
      calibrate_sensor();
      attachInterrupt(digitalPinToInterrupt(echopin), echoISR, CHANGE);
    } //else {
      //toggleState = !toggleState;
    //}
  }

  // ── Dedicated calibrate button (hold 3 s) ────────────────────────────────
  if (digitalRead(calibrateButton) == LOW) {
    delay(3000);
    if (digitalRead(calibrateButton) == LOW) {
      detachInterrupt(digitalPinToInterrupt(echopin));
      calibrate_sensor();
      attachInterrupt(digitalPinToInterrupt(echopin), echoISR, CHANGE);
    }
  }

  // ── Reset button ─────────────────────────────────────────────────────────
  if (digitalRead(resetButton) == LOW) {
    robot_stop();
    lineCounter  = 0;
    counter      = 0;
    inBlackArea  = false;
    toggleState  = false;
  }

  previousState = buttonState;

  // ── Main movement logic ───────────────────────────────────────────────────
  if (toggleState) {

    // ── Completion: 5 line crossings done ──────────────────────────────────
    /*
      if (lineCounter == 5) {
      robot_forward();
      Serial.println("FINAL FORWARD");
      delay(500);
      robot_stop();
      toggleState = false;
      return;
    }
    */

    // ── Initial launch nudge ────────────────────────────────────────────────
    if (lineCounter == 0) {
      robot_forward();
      Serial.println("FORWARD INIT");
      delay(150);
      lineCounter++;
    }

    // ── All-black = end-of-track line ───────────────────────────────────────
    else if (leftVal >= 1000 && centerVal >= 1000 && rightVal >= 1000) {
      if (!inBlackArea) {
        lineCounter++;
        inBlackArea = true;
        Serial.print("Line Counter incremented to: ");
        Serial.println(lineCounter);
      }
      /*
        if (lineCounter < 5) {
        turn_around();
        delay(450);
        // Keep turning until left sensor sees white again
        while (analogRead(sensorL) <= 1000) {
          turn_around();
        }
      }
      */
    }

    // ── Normal line-following ────────────────────────────────────────────────
    else {
      inBlackArea = false;

      if (leftVal >= 1000) {
        turn_left();
        Serial.println("LEFT");
      }
      
      else if (rightVal >= 1000) {
        turn_right();
        Serial.println("RIGHT");
      

      } else if (leftVal <= 1000 && rightVal <= 1000 && centerVal <= 1000) {
        turn_around();
        Serial.println("recovering")

  } else {
    robot_forward();
    Serial.println("Moving along")
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Calibration
// ─────────────────────────────────────────────────────────────────────────────
void calibrate_sensor() {
  Serial.println("\nCALIBRATING...");
  long sumLeft = 0, sumCenter = 0, sumRight = 0;
  int sensorReadings = 1000;

  for (int i = 0; i < sensorReadings; i++) {
    sumLeft   += analogRead(sensorL);
    sumCenter += analogRead(sensorM);
    sumRight  += analogRead(sensorR);
    delay(10);
    if (TaskTimer % 500 < 250) {
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
  for (int i = 0; i < sensorReadings; i++) {
    sumLeft   += analogRead(sensorL);
    sumCenter += analogRead(sensorM);
    sumRight  += analogRead(sensorR);
    delay(10);
    if (TaskTimer % 250 < 125) {
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  avgWhite = (sumLeft + sumCenter + sumRight) / (3 * sensorReadings);
  Serial.print("Average White Value: ");
  Serial.println(avgWhite);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Movement helpers
//  Servos are continuous rotation:
//    1500 µs = stopped
//    >1500   = one direction, <1500 = other direction
//  Left and right servos are mirrored, so they use opposite values to go forward
// ─────────────────────────────────────────────────────────────────────────────
void robot_forward() {
  servoL.writeMicroseconds(1300);   // left servo forward
  servoR.writeMicroseconds(1800);   // right servo forward (mirrored)
}

void robot_stop() {
  servoL.writeMicroseconds(1500);
  servoR.writeMicroseconds(1500);
}

void turn_right() {
  servoL.writeMicroseconds(1350);   // left faster
  servoR.writeMicroseconds(1800);   // right slower
}

void turn_left() {
  servoL.writeMicroseconds(1800);   // left slower
  servoR.writeMicroseconds(1350);   // right faster
}

void sharp_right() {
  servoL.writeMicroseconds(1500);   // left full forward
  servoR.writeMicroseconds(2000);   // right stopped
}

void sharp_left() {
  servoL.writeMicroseconds(1000);   // left stopped
  servoR.writeMicroseconds(1500);   // right full forward
}

void turn_around() {
  servoL.writeMicroseconds(1800);   // both reverse = pivot turn
  servoR.writeMicroseconds(1200);
}
