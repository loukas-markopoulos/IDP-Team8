
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MotorShield.h>

//caution: i use hella doubles because i am cautious, lazy and do not believe in such a thing as 'memory management'
//at speed 200, the unloaded motor completes 8 revolutions in 9.85 seconds. let's use 200 as our "standard speed"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// set up the motors with 1 being left and 2 being right
Adafruit_DCMotor *MotorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *MotorRight = AFMS.getMotor(2);

int8_t LEFT_LINE_SENSOR_PIN = 7;
int8_t RIGHT_LINE_SENSOR_PIN = 6;
int8_t FRONT_LINE_SENSOR_PIN = 5;

float WHEEL_DIAMETER = 64; //mm
float LINEAR_SPEED_200 =  PI * (8/9.85) * WHEEL_DIAMETER; // in mms-1 at speed 200

int16_t STATE = 1; // 0: idle, 1: moving

enum LINE_FOLLOW_STATE {
  CENTRAL_LINE_AHEAD, // the side sensors both read black and the front sensor reads white i.e we're on track and there's road ahead
  CENTRAL_NO_LINE_AHEAD, // the side sensors both read black and the front sensor reads black i.e we're on track but out of road
  LINE_TO_LEFT_AHEAD, //the left sensor reads white and the right sensor reads black and the front sensor reads white
  LINE_TO_RIGHT_AHEAD, //options to the right and ahead like above
  LINE_TO_LEFT_NO_AHEAD, //as above but front black
  LINE_TO_RIGHT_NO_AHEAD,
  UNSURE // catch-all
};

void setup() {
  Serial.begin(9600);  
  Serial.println("initialising the absolute bear minimum...");

  pinMode(8, OUTPUT); //using pin 8 as a 5v supply
  pinMode(9, OUTPUT); //using pin 9 as a 5v supply //TODO get rid of this and put a proper shield on the Arduino with connectors for the sensors
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);
  

  // motor initialisation

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  pinMode(7, INPUT); //line sensors
  pinMode(6, INPUT);
  pinMode(5, INPUT);

}

LINE_FOLLOW_STATE ReadLineFollowSensors () {
  bool leftSensor = !digitalRead(LEFT_LINE_SENSOR_PIN);
  bool rightSensor = !digitalRead(RIGHT_LINE_SENSOR_PIN);
  bool frontSensor = !digitalRead(FRONT_LINE_SENSOR_PIN);
  if (leftSensor and rightSensor and (not frontSensor)) {
    return CENTRAL_LINE_AHEAD;
  } else if (leftSensor and rightSensor and frontSensor) {
    return CENTRAL_NO_LINE_AHEAD;
  } else if (leftSensor and (not rightSensor)) {
    if (not frontSensor) {
      return LINE_TO_RIGHT_AHEAD;
    } else {
      return LINE_TO_RIGHT_NO_AHEAD;
    }
  } else if (rightSensor and (not leftSensor)) {
    if (not frontSensor) {
      return LINE_TO_LEFT_AHEAD;
    } else {
      return LINE_TO_LEFT_NO_AHEAD;
    }
  } else {
    return UNSURE;
  }
}

void Drive(bool forward, int16_t speed, double differential) { // speed is an int from 0-255, differential is a percentage difference between the two motors with +ve turning left and -ve turning right.
  if (forward) {
    double leftPower = (differential > 0) ? speed * (1 - (differential/100)) : speed;
    double rightPower = (differential < 0) ? speed * (1 + (differential/100)) : speed;
    MotorLeft->run(BACKWARD);
    MotorRight->run(BACKWARD); //backwards is forwards in our hardware configuration
    Serial.println(leftPower);
    Serial.println(rightPower);
    MotorLeft->setSpeed(leftPower);
    MotorRight->setSpeed(rightPower);
  }
  else {
    double leftPower = (differential > 0) ? speed * (1 - (differential/100)) : speed;
    double rightPower = (differential < 0) ? speed * (1 + (differential/100)) : speed;
    MotorLeft->run(FORWARD);
    MotorRight->run(FORWARD);
    MotorLeft->setSpeed(leftPower);
    MotorRight->setSpeed(rightPower);
  }
}

void TankTurn(bool clockwise, int16_t speed) { //TODO: change in terms of degrees
if (clockwise) {
  MotorLeft->run(FORWARD);
  MotorRight->run(BACKWARD);
  } else {
  MotorLeft->run(BACKWARD);
  MotorRight->run(FORWARD);
  };
  MotorLeft->setSpeed(speed);
  MotorRight->setSpeed(speed);
}

void StopDriving() {
  MotorLeft->run(RELEASE);
  MotorRight->run(RELEASE);
}

void DriveDistanceStraight(bool forward_loc, double distance) { //distance in mm
  float timeToDrive = (distance / LINEAR_SPEED_200) * 1000;
  Drive(forward_loc, 200, 0);
  delay(timeToDrive);
  StopDriving();
}

void SimpleLineFollow () {
  LINE_FOLLOW_STATE followState = ReadLineFollowSensors(); //simplest possible line follower
  Serial.println(followState);
  switch (followState)
  {
  case CENTRAL_LINE_AHEAD:
    Drive(true, 200, 0);
    break;

  case CENTRAL_NO_LINE_AHEAD:
    StopDriving();
    // STATE = 0;
    break;

  case LINE_TO_LEFT_AHEAD:
    Drive(true, 200, 15);
    break;
  
  case LINE_TO_RIGHT_AHEAD:
    Drive(true, 200, -15);
    break;

  case LINE_TO_LEFT_NO_AHEAD:
    Drive(true, 200, 15);
    break;
  
  case LINE_TO_RIGHT_NO_AHEAD:
    Drive(true, 200, -15);
    break;

  default:
    Drive(true, 200, 0);
    break;
  }
}

void loop() {
  if (STATE == 1) {
    SimpleLineFollow();
  }
}