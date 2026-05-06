//non blocking delays !!!
#include <elapsedMillis.h>
elapsedMillis TaskTimer;
// motors
#include <Servo.h>
Servo servoL;
Servo servoR;
int lposFinal, rposFinal;
//some stuff required for logic \/
elapsedMillis crossingTimer;
bool crossingActive = false;
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
uint16_t sensors[3];
int16_t position;
const int sensorL = 28; //pre 26
const int sensorM = 27; // pre 25
const int sensorR = 26; //pre 24
bool calibratedCheck = false;

elapsedMillis calibrateTimer;

void calibrate() {
  qtr.resetCalibration();
  calibrateTimer = 0;

  while (calibrateTimer < 10000) {  // 10 seconds for black
    qtr.calibrate();
    if (calibrateTimer % 500 < 250) {
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  calibrateTimer = 0;
  while (calibrateTimer < 10000) {  // 10 seconds for white
    qtr.calibrate();
    if (calibrateTimer % 250 < 125) {
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  digitalWrite(LED_BUILTIN, LOW);
  calibratedCheck = true;  // mark calibration as done
}
//PID stuffs
float Kp = 0.1;
//float Ki = 0.0;
float Kd = 0.5;
float P, I, D, lastError, servoSpeed;
void pidController() {
  //Read the position using sensors/library objects
  int16_t error = position - 1000; //current position (0 - 3000) - ideal position
  P = error;
  //I = I + error;
  D = error - lastError;
  servoSpeed = Kp * P + Kd * D; //previously P*Kp + I*Ki + D*Kd; Calculates the correction value
  lastError = error;
}
// us sensor
const int trigpin = 2;
const int echopin = 3;
volatile unsigned long echoStart = 0;
volatile unsigned long echoEnd = 0;
volatile bool echoReady = false;
float distance = 0.0;
elapsedMillis trigTimer;
void echoISR() {
  if (gpio_get(echopin)) {
    echoStart = micros();
  } else {
    echoEnd = micros();
    echoReady = true;
  }
}
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
  attachInterrupt(digitalPinToInterrupt(echopin), echoISR, CHANGE);

  servoL.attach(15); 
  servoR.attach(14);
  servoL.write(90); 
  servoR.write(90);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){sensorL, sensorM, sensorR}, 3);

  Serial.begin(115200);
}

void loop() {
  /*
  Serial.println("Sensor 1");
  Serial.println(sensors[0]);
  Serial.println("Sensor 2");
  Serial.println(sensors[1]);
  Serial.println("Sensor 3");
  Serial.println(sensors[2]);
  */
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
    if (TaskTimer >= 3000) {
      if (digitalRead(calibrateButton) == LOW) {
        detachInterrupt(digitalPinToInterrupt(echopin));
        calibrate();
        attachInterrupt(digitalPinToInterrupt(echopin), echoISR, CHANGE);
        TaskTimer = 0;
      } else {TaskTimer = 0;}
    }
  }
  if (digitalRead(startButton) == LOW) {
  start = true;
  }
  if (digitalRead(resetButton) == LOW) {
    servoL.write(90); 
    servoR.write(90);
    lineCrossed = -1;
    requirement = 4;
    start = false;
    calibratedCheck = false;
  }
  if (start == false) {
    return;
  }
  //ir sensor detection \/
  position = qtr.readLineBlack(sensors);
  Serial.println(position);
  pidController();
  //motor code (All detection logic will go here) \/
  /*
  finalServoL = servoSpeed + baseServoSpeed
  finalServoR = servoSpeed + baseServoSpeed
  If servo exceeds maxspeed, limit servo to maxspeed
  Apply new values to servo, preferably with a function
  */
  //2500 = right base speeds while going straight (about 80% max, change in testing to visualize pid)
  //500 = left  base speed
  float lMotorSpeed = 1000.0 + servoSpeed; //Base 500
  float rMotorSpeed = 2000.0 - servoSpeed; //Base 2500

  lMotorSpeed = constrain(lMotorSpeed, 0, 2000.0);
  rMotorSpeed = constrain(rMotorSpeed, 1000.0, 3000.0);
  
  lposFinal = map((int)lMotorSpeed, 0, 3000, 0, 180); //motorspeed, min (0), max (3000), 0, 180,
  rposFinal = map((int)rMotorSpeed, 0, 3000, 0, 180); //values are 0 and 3000 to match ir sensor values (any plausible range should work theoretically)
  Serial.println(lposFinal);
  Serial.println(rposFinal);
  
  servoL.write(lposFinal); //writes to servo (0 full back, 90 stop, 180 full forward)
  servoR.write(rposFinal);

  if (crossingActive && crossingTimer >= 500) {
  crossingActive = false;
  }

  if ((sensors[0] > 950) && (sensors[1] > 950) && (sensors[2] > 950))
  {
    turn180();
  }

  // us sensor code \/
    // Trigger a pulse every 60ms
  if (trigTimer >= 60) {
    digitalWrite(trigpin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigpin, LOW);
    trigTimer = 0;
  }
  // Read result whenever ISR signals it's ready
  if (echoReady) {
    noInterrupts(); // briefly pause ISR to safely read shared vars
    unsigned long duration = echoEnd - echoStart;
    echoReady = false;
    interrupts();

    distance = (duration * 0.034) / 2.0;

    if (distance > 0 && distance <= 10) {
      requirement = 2;
      turn180();
    }
  }
}

void turn180() {
  if (crossingActive) { return; }
  
  crossingActive = true;
  crossingTimer = 0;
  lineCrossed++;

  if (lineCrossed == 0) { return; }
  if (lineCrossed >= requirement) {
    start = false;
    servoL.write(90);
    servoR.write(90);
    return;
  }
    //Turn around
    servoL.write(0); //full
    servoR.write(0); //reverse
    while (true) {
    int16_t pos = qtr.readLineBlack(sensors);
    if (pos >= 900 && pos <= 1100) { break; }
    } 
    servoL.write(30);  // nudge forward briefly
    servoR.write(180);
    delay(50);
    return;
}
