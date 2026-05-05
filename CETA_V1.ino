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
//PID stuffs
float Kp = 0.0; // to two or more decimal places
float Ki = 0.0; // to 4 or more decimal places
float Kd = 0.3; // to one decimal place
int P, I, D, lastError, servoSpeed;
void pidController() {
  //Read the position using sensors/library objects
  int16_t error = position - 1000; //current position (0 - 3000) - ideal position
  P = error;
  I = I + error;
  D = error - lastError;
  servoSpeed = P*Kp + I*Ki + D*Kd; //Calculates the correction value
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
  //2500 = right base speeds while going straight (about 80% max, change in testing to visualize pid)
  //500 = left  base speed
  int16_t lMotorSpeed = 500 + servoSpeed; //may need to change signs based on motor directionality (i'm dumb)
  int16_t rMotorSpeed = 2500 - servoSpeed; 

  if (lMotorSpeed > 3000) {
    lMotorSpeed = 3000;
  } //may need to change logic based on motor directionality
  if (rMotorSpeed > 3000) {
    rMotorSpeed = 3000;
  }
  if (lMotorSpeed < 1000) {
    lMotorSpeed = 1000;
  } 
  if (rMotorSpeed < 1000) {
    rMotorSpeed = 1000;
  }
  
  lposFinal = map(lMotorSpeed, 0, 3000, 0, 180); //motorspeed, min (0), max (3000), 0, 180,
  rposFinal = map(rMotorSpeed, 0, 3000, 0, 180); //values are 0 and 3000 to match ir sensor values (any plausible range should work theoretically)
  
  servoL.write(lposFinal); //writes to servo (0 full back, 90 stop, 180 full forward)
  servoR.write(rposFinal);

  if ((sensors[0] > 600) && (sensors[1] > 600) && (sensors[2] > 600))
  {
    lineCrossed++;
    if (lineCrossed == 0) {return;}
    if (lineCrossed >= requirement) {start = false;} //robot should stop after 2 laps
    //Turn around
    //may need to add a slight delay for sensors to cross the "T" fully
    servoL.write(0); //full
    servoR.write(90); //barely any movement
    //wait however many seconds for a full turn or use the below while
    while (qtr.readLineBlack(sensors) >= 950 && qtr.readLineBlack(sensors) <= 1050) {
      //empty to stall
    } // if this while somehow blocks .readlineblack function research "std::thread" instead
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
    //Turn around
    //may need to add a slight delay for sensors to cross the "T" fully
    servoL.write(180); //full
    servoR.write(100); //barely any movement
    //wait however many seconds for a full turn or use the below while
    while (qtr.readLineBlack(sensors) >= 950 && qtr.readLineBlack(sensors) <= 1050) {
      //empty to stall
    } // if this while somehow blocks .readlineblack function research "std::thread" instead
    return;
    // maybe add buzzer noise for fun
  }
}
