
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

int8_t WIDE_RIGHT_LINE_SENSOR_PIN = 13;
int8_t WIDE_LEFT_LINE_SENSOR_PIN = 12;
int8_t FRONT_RIGHT_LINE_SENSOR_PIN = 11;
int8_t FRONT_LEFT_LINE_SENSOR_PIN = 10;

float WHEEL_DIAMETER = 64; //mm
float LINEAR_SPEED_200 =  PI * (8/9.85) * WHEEL_DIAMETER; // in mms-1 at speed 200

int16_t STATE = 1; // 0: idle, 1: moving

enum LINE_FOLLOW_STATE {
  CENTRAL, //front two white, side two black
  LINE_TO_RIGHT, //front right white, front left black, side two black
  LINE_TO_LEFT, //front left white, front right black, side two black
  UNSURE // catch-all
};

struct JUNCTION {
  bool AHEAD;
  bool LEFT;
  bool RIGHT;
};

void setup() {
  Serial.begin(9600);  
  Serial.println("initialising the absolute bear minimum...");
  

  // motor initialisation

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }

  Serial.println("Motor Shield found.");

  delay(5000); //so we can get some stuff done before the thing do

}

LINE_FOLLOW_STATE ReadLineFollowSensors () {
  bool wideLeftSensor = digitalRead(WIDE_LEFT_LINE_SENSOR_PIN); //typecast to bool where true is white and black is false
  bool wideRightSensor = digitalRead(WIDE_RIGHT_LINE_SENSOR_PIN);
  bool frontLeftSensor = digitalRead(FRONT_LEFT_LINE_SENSOR_PIN);
  bool frontRightSensor = digitalRead(FRONT_RIGHT_LINE_SENSOR_PIN);

  if (!wideLeftSensor and !wideRightSensor) {
  
    if (frontLeftSensor and frontRightSensor) {
      return CENTRAL;
    }

    else if (frontLeftSensor and !frontRightSensor) {
      return LINE_TO_LEFT;
    }

    else if (!frontLeftSensor and frontRightSensor) {
      return LINE_TO_RIGHT;
    }

  }

  return UNSURE;

}

JUNCTION AssessJunction () {
  bool wideLeftSensor = digitalRead(WIDE_LEFT_LINE_SENSOR_PIN); //typecast to bool where true is white and black is false
  bool wideRightSensor = digitalRead(WIDE_RIGHT_LINE_SENSOR_PIN);
  bool frontLeftSensor = digitalRead(FRONT_LEFT_LINE_SENSOR_PIN);
  bool frontRightSensor = digitalRead(FRONT_RIGHT_LINE_SENSOR_PIN);

  bool ahead_loc = (frontRightSensor and frontLeftSensor);
  bool left_loc = wideLeftSensor;
  bool right_loc = wideRightSensor;

  return JUNCTION{ahead_loc, left_loc, right_loc};
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

void StopDriving() {
  MotorLeft->run(RELEASE);
  MotorRight->run(RELEASE);
}

JUNCTION LineFollowToJunction () {
  while (true) {
    LINE_FOLLOW_STATE followState = ReadLineFollowSensors();
    switch (followState)
    {
    case CENTRAL:
      Drive(true, 200, 0);
      break;

    case LINE_TO_LEFT:
      Drive(true, 200, 22);;
      break;
    
    case LINE_TO_RIGHT:
      Drive(true, 200, -22);;
      break;

    case UNSURE:
      StopDriving();
      JUNCTION NewJunction = AssessJunction();  
      return NewJunction;
    }   
  }
}

void Turn90Degrees(bool right) {

  int16_t differential = right ? -90 : 90;
  bool turning = true;
  Drive(true, 200, differential);
  delay(1500); //let's let it start the turn before we start checking the sensors to see if we're done

  while (turning) {
    bool frontLeftSensor = digitalRead(FRONT_LEFT_LINE_SENSOR_PIN);
    bool frontRightSensor = digitalRead(FRONT_RIGHT_LINE_SENSOR_PIN);


    Drive(true, 200, differential);
    turning = !(frontLeftSensor and frontRightSensor);
  };

  StopDriving();
  Serial.println("Turn Complete!");
}

void loop() {
  if (STATE == 1) {
    JUNCTION nextJunction = LineFollowToJunction();
    if (nextJunction.LEFT) {
      Turn90Degrees(false);
    } else if (nextJunction.RIGHT) {
      Turn90Degrees(true);
    } else {
      STATE = 0;
    }
  }
}