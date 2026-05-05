//non blocking delays !!!
#include <elapsedMillis.h>
elapsedMillis TaskTimer;
bool pulse = true;
// motors and arm
#include <Servo.h>
Servo servoL;
Servo servoR;
Servo armServo;
int lposFinal, rposFinal;
//some stuff required for logic \/
int lineCrossed = -1; //tallying line crosses except for the starting line
int requirement = 4; //4 turns, 2 laps
bool start = false;
int calibrateButton = 19;
int resetButton = 20;
int startButton = 21;
//int buttonVal = 1;
// ir sensor
#include <QTRSensors.h>
QTRSensors qtr;
int16_t position;
const int sensorL = 28; //pre 26
const int sensorM = 27; // pre 25
const int sensorR = 26; //pre 24
bool calibratedCheck = false;

void calibrate() {
  qtr.resetCalibration();
  Serial.println("Calibrate Function Works!");
  for (uint8_t i = 0; i < 100; i++) { //10 seconds on black, 10 on white
    qtr.calibrate();
    if (TaskTimer >= 250) {
      digitalWrite(LED_BUILTIN, HIGH);
      if (TaskTimer >= 500) {
        digitalWrite(LED_BUILTIN, LOW);
        TaskTimer = 0;
      }
    }
    delay(100);
  }
  Serial.println("Move robot");
  for (uint8_t i = 0; i < 100; i++) { //10 seconds on black, 10 on white
    qtr.calibrate();
    if (TaskTimer >= 100) {
      digitalWrite(LED_BUILTIN, HIGH);
      if (TaskTimer >= 500) {
        digitalWrite(LED_BUILTIN, LOW);
        TaskTimer = 0;
      }
    }
    delay(100);
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Calibrate Function Finished!");
}
// PID settings and runtime vars
float Kp = 0.60; // tune this
float Ki = 0.00;
float Kd = 0.15;
float P = 0.0, I = 0.0, D = 0.0, lastError = 0.0, servoSpeed = 0.0;
// debug and tuning helpers
bool invertCorrection = false; // flip to true if correction sign is inverted
const float I_WINDUP_LIMIT = 200.0; // limit integral term
const int DEADZONE = 30; // small deadband around center to avoid hunting

// QTR scaling constants (3 sensors -> 0..2000, center ~1000)
const int NUM_SENSORS = 3;
const int QTR_MAX = 1000 * (NUM_SENSORS - 1); // 2000 for 3 sensors
const int QTR_MID = QTR_MAX / 2; // 1000

// motor speed range in the same scale as QTR mapping (use a 1000-based range)
const int MOTOR_MIN = 1000;
const int MOTOR_MAX = 2000;
const int MOTOR_STOP = 1500; // this corresponds to servo=90 (stop)

void pidController() {
  //Read the position using sensors/library objects
  float error = (float)position - (float)QTR_MID; // center on QTR_MID
  // Apply deadzone
  if (abs((int)error) < DEADZONE) error = 0;
  P = error;
  I = I + error;
  // integral windup clamp
  if (I > I_WINDUP_LIMIT) I = I_WINDUP_LIMIT;
  if (I < -I_WINDUP_LIMIT) I = -I_WINDUP_LIMIT;
  D = error - lastError;
  servoSpeed = Kp * P + Ki * I + Kd * D; // Calculates the correction value (float)
  if (invertCorrection) servoSpeed = -servoSpeed;
  lastError = error;
}
// us sensor
  const int trigpin = 2; 
  const int echopin = 3; 
  float timing = 0.0;
  float distance = 0.0;
/*
issues and to do
calibration sequence
wireless start for IoT challenge ("Adafruit IO via a WiFi connection")
start button
conditinal for challenge three to prevent unneccessary movement (?)
literally all the code
buttons are on 31, 32 and 34
*/
void setup() {
  /*
  initialize all pins
  set up ultrasonic sensor
  set up ir sensors
  set up servo motors
  Reset arm to starting position if need be
  */
  //adafruit
  #define IO_USERNAME "placeholder"
  #define IO_KEY "placeholder"
  #define WIFI_SSID "your_wifi_name" //Wifi Name
  #define WIFI_PASS "your_wifi_password" //Wifi Password
  /*
    AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
    io.connect();
  */
  //random stuff
  pinMode(calibrateButton, INPUT_PULLUP);
  pinMode(startButton, INPUT_PULLUP);
  pinMode(resetButton, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  //potential defined delay value
  pinMode(echopin, INPUT);
  pinMode(trigpin, OUTPUT);
  digitalWrite(trigpin, LOW);

  servoL.attach(15); 
  servoR.attach(14);
  armServo.attach(13);

  // Start servos in stopped position
  servoL.write(90);
  servoR.write(90);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){sensorL, sensorM, sensorR}, 3);

  Serial.begin(115200);
}

void loop() {
  //Serial.println("Serial Monitor Begin");
  //Serial.println(buttonVal);
  //buttonVal = digitalRead(calibrateButton);
  // pseudocode
  /*
  ir sensor
    get position of sensors (0 - 3000)
    calculate error value
    use error value to find correction value
  servo motors
    robot turns on based on button press
    regular drive code
      apply correction value to servos
      turning code and other edge cases
    apply conditionals based on if motorspeed exceeds min/max
    apply conditionals on the amnt of revolutions (? just go fast until the end? i dunno)
  us sensor
    if distance less than 10, perform 180 turn
  arm (conditional ?)
    (needs discussion on design and looking at rules)
  */
  //calibrate and start
  if (digitalRead(calibrateButton) == LOW) {
    /*
      if (TaskTimer >= 3000) {
      if (digitalRead(calibrateButton) == LOW) {
        calibrate();
        TaskTimer = 0;
      } else {TaskTimer = 0;}
    }
    */
    Serial.println("Calibrate Button Works!");
    calibrate();
  }
  if (digitalRead(startButton) == LOW) {
    start = true;
    Serial.println("Start True");
    Serial.println("Start button works!");
  }
  if (digitalRead(resetButton) == LOW) {
    Serial.println("Reset Button Press");
    servoL.write(90); 
    servoR.write(90);
    while(true) {}
  }
  if (start == false) {
    return;
  }
  Serial.println("The code is working");
  //ir sensor detection \/
  uint16_t sensors[3];
  position = qtr.readLineBlack(sensors);

  pidController();
  //motor code (All detection logic will go here) \/
  /*
  finalServoL = servoSpeed + baseServoSpeed
  finalServoR = servoSpeed + baseServoSpeed
  If servo exceeds maxspeed, limit servo to maxspeed
  Apply new values to servo, preferably with a function
  */
  // Base motor speeds (stop is MOTOR_STOP)
  // We apply servoSpeed as correction: left = base + correction, right = base - correction
  int baseSpeed = MOTOR_STOP; // neutral
  float lMotorSpeedF = (float)baseSpeed + servoSpeed;
  float rMotorSpeedF = (float)baseSpeed - servoSpeed;

  // clamp to allowed range
  if (lMotorSpeedF < MOTOR_MIN) lMotorSpeedF = MOTOR_MIN;
  if (lMotorSpeedF > MOTOR_MAX) lMotorSpeedF = MOTOR_MAX;
  if (rMotorSpeedF < MOTOR_MIN) rMotorSpeedF = MOTOR_MIN;
  if (rMotorSpeedF > MOTOR_MAX) rMotorSpeedF = MOTOR_MAX;

  // map to servo write range 0-180
  lposFinal = map((int)lMotorSpeedF, MOTOR_MIN, MOTOR_MAX, 0, 180);
  rposFinal = map((int)rMotorSpeedF, MOTOR_MIN, MOTOR_MAX, 0, 180);
  // Telemetry for tuning: sensors[], position, PID components and servo outputs
  Serial.print("S:"); Serial.print(sensors[0]); Serial.print(","); Serial.print(sensors[1]); Serial.print(","); Serial.print(sensors[2]);
  Serial.print(" P"); Serial.print(P);
  Serial.print(" I"); Serial.print(I);
  Serial.print(" D"); Serial.print(D);
  Serial.print(" pos:"); Serial.print(position);
  Serial.print(" servoSpeed:"); Serial.print(servoSpeed);
  Serial.print(" L:"); Serial.print(lposFinal);
  Serial.print(" R:"); Serial.println(rposFinal);

  servoL.write(lposFinal); //writes to servo (0 full back, 90 stop, 180 full forward)
  servoR.write(rposFinal);

  if ((sensors[0] > 600) && (sensors[1] > 600) && (sensors[2] > 600))
  {
    lineCrossed++;
    if (lineCrossed == 0) {return;}
    if (lineCrossed >= requirement) {start = false;} //robot should stop after 2 laps
    // Turn around: spin left in place for a short period while monitoring sensors
    servoL.write(0); // full forward on left
    servoR.write(180); // full reverse on right (adjust if your servos/motor controller are inverted)
    // wait until the line is no longer detected under center sensor (simple approach)
    for (unsigned long t = millis(); millis() - t < 1000;) {
      qtr.readLineBlack(sensors);
      // if center sensor is clear (below threshold), break early
      if (sensors[1] < 600) break;
      delay(10);
    }
    // stop
    servoL.write(90);
    servoR.write(90);
    return;
  }

  // us sensor code \/
  
    digitalWrite(trigpin, LOW);
  if (pulse) {
    if (TaskTimer >= 2) {
      digitalWrite(trigpin, HIGH);
      TaskTimer = 0;
    }
    pulse = false;
  }
  
  if (!pulse) {
    if (TaskTimer >= 10) {
      digitalWrite(trigpin, LOW);
      TaskTimer = 0;
    }
    pulse = true;
  }

  timing = pulseIn(echopin, HIGH);
  distance = (timing * 0.034) / 2;
  
  //Serial.println("Distance: " + distance);
  if (distance <= 10) {
    //just cut and pasted code from motor section
    requirement = 2; //Should stop after a lap to the obstacle and back due to the logic in motor section
    lineCrossed++;
    if (lineCrossed == 0) {return;}
    // Turn around on obstacle detection
    servoL.write(0);
    servoR.write(180);
    for (unsigned long t = millis(); millis() - t < 1000;) {
      qtr.readLineBlack(sensors);
      if (sensors[1] < 600) break;
      delay(10);
    }
    servoL.write(90);
    servoR.write(90);
    return;
    // maybe add buzzer noise for fun
  }
}
