//non blocking delays !!!
#include <elapsedMillis.h>
bool pulse = true;
// motors and arm
#include <Servo.h>
Servo servoL;
Servo servoR;
Servo armServo;
int lpos = 0;
int rpos = 0;
// ir sensor
#include <QTRSensors.h>
QTRSensors qtr;
const int sensorL = ; //any analog pins
const int sensorM = ; //any analog pins
const int sensorR = ; //any analog pins

void calibrate() {
  resetCalibration();
  for (uint8_t i = 0; i < 1000; i++) //10 seconds on black, 10 on white
  {
    qtr.calibrate();
    delay(20);
  }
}
//PID stuffs
float Kp = 0.0; // to two or more decimal places
float Ki = 0.0; // to 4 or more decimal places
float Kd = 0.0; // to one decimal place
int P, I, D, lastError;
void pidController() {
  //Read the position using sensors/library objects
  int16_t error = position - 1000; //current position (0 - 3000) - ideal position
  P = error;
  I = I + error;
  D = error + lastError;
  int servoSpeed = P*Kp + I*Ki + D*Kd; //Calculates the correction value
  lastError = error;
  /* Do this in motor section of loop
  finalServoL = servoSpeed + baseServoSpeed
  finalServoR = servoSpeed + baseServoSpeed
  If servo exceeds maxspeed, limit servo to maxspeed
  Apply new values to servo, preferably with a function
  */
}
// us sensor
const int buzzer = ; 
const int trigpin = ; 
const int echopin = ; 
float timing = 0.0;
float distance = 0.0;
/*
issues and to do
calibration buttons and wireless calibration
figure out how to track amnt of times track has been run
turn around AS SOON as you hit the line
start button
literally all the code
*/
void setup() {
  /*
  initialize all pins
  set up ultrasonic sensor
  set up ir sensors
  set up servo motors
  basic logic for start up
    Drive forward (until ir sensor detection, set up in loop)
    Reset arm to starting position
  */
  elapsedMillis TaskTimer;
  //potential defined delay value
  
  pinMode(echopin, INPUT);
  pinMode(trigpin, OUTPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(trigpin, LOW);
  digitalWrite(buzzer, LOW);

  servoL.attach(1); //arbitrary pin numbers, adjust later
  servoR.attach(2);
  armServo.attach(3);
  lpos = map(); //left motorspeed, min, max, 0, 180,
  rpos = map(); //right motorspeed, min, max, 0, 180,

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){sensorL, sensorM, sensorR}, 3);

  Serial.begin(9600);
}

void loop() {
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
  //ir sensor detection \/
  uint16_t sensors[3];
  int16_t position = qtr.readLineBlack(sensors);

  pidController();
  //motor code (All detection logic will go here) \/
  //nothing yet

  // us sensor code \/
  digitalWrite(trigpin, LOW);
  if (pulse) {
    if (TaskTimer >= 2) {
      digitalWrite(trigpin, HIGH);
    }
    pulse = false;
  }
  
  if (!pulse) {
    if (TaskTimer >= 10) {
      digitalWrite(trigpin, LOW);
    }
    pulse = true;
  }

  timing = pulseIn(echopin, HIGH);
  distance = (timing * 0.034) / 2;
  //Serial.println("Distance: " + distance);
  if (distance <= 10) {
    //turn 180
    // maybe add buzzer noise for fun
  } /* else {
    stop buzzer noise
  } */
}
