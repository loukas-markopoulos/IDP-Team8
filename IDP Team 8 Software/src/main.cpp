
// #include <Arduino.h>
// #include <SPI.h>
// #include <Adafruit_MotorShield.h>

// //caution: i use hella doubles because i am cautious, lazy and do not believe in such a thing as 'memory management'
// //at speed 200, the unloaded motor completes 8 revolutions in 9.85 seconds. let's use 200 as our "standard speed"

// // Create the motor shield object with the default I2C address
// Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// // set up the motors with 1 being left and 2 being right
// Adafruit_DCMotor *MotorLeft = AFMS.getMotor(1);
// Adafruit_DCMotor *MotorRight = AFMS.getMotor(2);

// int8_t WIDE_RIGHT_LINE_SENSOR_PIN = 13;
// int8_t WIDE_LEFT_LINE_SENSOR_PIN = 12;
// int8_t FRONT_RIGHT_LINE_SENSOR_PIN = 11;
// int8_t FRONT_LEFT_LINE_SENSOR_PIN = 10;

// float WHEEL_DIAMETER = 64; //mm
// float LINEAR_SPEED_200 =  PI * (8/9.85) * WHEEL_DIAMETER; // in mms-1 at speed 200

// int16_t STATE = 1; // 0: idle, 1: moving

// enum LINE_FOLLOW_STATE {
//   CENTRAL, //front two white, side two black
//   LINE_TO_RIGHT, //front right white, front left black, side two black
//   LINE_TO_LEFT, //front left white, front right black, side two black
//   UNSURE // catch-all
// };

// struct JUNCTION {
//   bool AHEAD;
//   bool LEFT;
//   bool RIGHT;
// };

// void setup() {
//   Serial.begin(9600);  
//   Serial.println("initialising the absolute bear minimum...");
  

//   // motor initialisation

//   if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
//   // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
//     Serial.println("Could not find Motor Shield. Check wiring.");
//     while (1);
//   }

//   Serial.println("Motor Shield found.");

//   delay(5000); //so we can get some stuff done before the thing do

// }

// LINE_FOLLOW_STATE ReadLineFollowSensors () {
//   bool wideLeftSensor = digitalRead(WIDE_LEFT_LINE_SENSOR_PIN); //typecast to bool where true is white and black is false
//   bool wideRightSensor = digitalRead(WIDE_RIGHT_LINE_SENSOR_PIN);
//   bool frontLeftSensor = digitalRead(FRONT_LEFT_LINE_SENSOR_PIN);
//   bool frontRightSensor = digitalRead(FRONT_RIGHT_LINE_SENSOR_PIN);

//   if (!wideLeftSensor and !wideRightSensor) {
  
//     if (frontLeftSensor and frontRightSensor) {
//       return CENTRAL;
//     }

//     else if (frontLeftSensor and !frontRightSensor) {
//       return LINE_TO_LEFT;
//     }

//     else if (!frontLeftSensor and frontRightSensor) {
//       return LINE_TO_RIGHT;
//     }

//   }

//   return UNSURE;

// }

// JUNCTION AssessJunction () {
//   bool wideLeftSensor = digitalRead(WIDE_LEFT_LINE_SENSOR_PIN); //typecast to bool where true is white and black is false
//   bool wideRightSensor = digitalRead(WIDE_RIGHT_LINE_SENSOR_PIN);
//   bool frontLeftSensor = digitalRead(FRONT_LEFT_LINE_SENSOR_PIN);
//   bool frontRightSensor = digitalRead(FRONT_RIGHT_LINE_SENSOR_PIN);

//   bool ahead_loc = (frontRightSensor and frontLeftSensor);
//   bool left_loc = wideLeftSensor;
//   bool right_loc = wideRightSensor;

//   return JUNCTION{ahead_loc, left_loc, right_loc};
// }

// void Drive(bool forward, int16_t speed, double differential) { // speed is an int from 0-255, differential is a percentage difference between the two motors with +ve turning left and -ve turning right.
//   if (forward) {
//     double leftPower = (differential > 0) ? speed * (1 - (differential/100)) : speed;
//     double rightPower = (differential < 0) ? speed * (1 + (differential/100)) : speed;
//     MotorLeft->run(BACKWARD);
//     MotorRight->run(BACKWARD); //backwards is forwards in our hardware configuration
//     Serial.println(leftPower);
//     Serial.println(rightPower);
//     MotorLeft->setSpeed(leftPower);
//     MotorRight->setSpeed(rightPower);
//   }
//   else {
//     double leftPower = (differential > 0) ? speed * (1 - (differential/100)) : speed;
//     double rightPower = (differential < 0) ? speed * (1 + (differential/100)) : speed;
//     MotorLeft->run(FORWARD);
//     MotorRight->run(FORWARD);
//     MotorLeft->setSpeed(leftPower);
//     MotorRight->setSpeed(rightPower);
//   }
// }

// void StopDriving() {
//   MotorLeft->run(RELEASE);
//   MotorRight->run(RELEASE);
// }

// JUNCTION LineFollowToJunction () {
//   while (true) {
//     LINE_FOLLOW_STATE followState = ReadLineFollowSensors();
//     switch (followState)
//     {
//     case CENTRAL:
//       Drive(true, 200, 0);
//       break;

//     case LINE_TO_LEFT:
//       Drive(true, 200, 22);
//       break;
    
//     case LINE_TO_RIGHT:
//       Drive(true, 200, -22);
//       break;

//     case UNSURE:
//       StopDriving();
//       JUNCTION NewJunction = AssessJunction();  
//       return NewJunction;
//     }   
//   }
// }

// void TurnUntilLine(bool right) {

//   bool turning = true;

//   int16_t differential = right ? -90 : 90;

//   int16_t TURN_STATE = 0;
//     // 0: STARTING
//     // 1: CENTRAL, trips when both central sensors cross
//     // 2: OVERSHOOT, trips when far wide sensor crosses ie we've overshot

//   while (turning) {
//     bool frontLeftSensor = digitalRead(FRONT_LEFT_LINE_SENSOR_PIN);
//     bool frontRightSensor = digitalRead(FRONT_RIGHT_LINE_SENSOR_PIN);
//     bool wideLeftSensor = digitalRead(WIDE_LEFT_LINE_SENSOR_PIN); //typecast to bool where true is white and black is false
//     bool wideRightSensor = digitalRead(WIDE_RIGHT_LINE_SENSOR_PIN);

//     bool firstSensor = right ? wideRightSensor : wideLeftSensor; // which direction?
//     bool lastSensor = right ? wideLeftSensor : wideRightSensor;
//     bool central = frontLeftSensor and frontRightSensor;

//     switch (TURN_STATE) {
//       case 0:
//         Drive(true, 200, differential);
//       case 1:
//         StopDriving();
//         turning = false;
//       case 2:
//         Drive(true, 200, -differential);
//     };

//     if (central) {
//       TURN_STATE = 1;
//     }

//     if (lastSensor) {
//       TURN_STATE = 2;
//     }

//     if (firstSensor and TURN_STATE = 2) {
//       TURN_STATE = 0;
//     }

//   }
// };

//   StopDriving();
//   Serial.println("Turn Complete!");
// }

// void loop() {
//   if (STATE == 1) {
//     JUNCTION nextJunction = LineFollowToJunction();
//     if (nextJunction.LEFT) {
//       TurnUntilLine(false);
//     } else if (nextJunction.RIGHT) {
//       TurnUntilLine(true);
//     } else {
//       STATE = 0;
//     }
//   }
// }


// testing the two servos for claw mechanisms

int verticalServo = 9;
int clawServo = 10;

void setup() {
    pinMode(verticalServo, OUTPUT);
    pinMode(clawServo, OUTPUT);
    delay(100);
}

// angle position is fixed 

void servoPulse(int angle, int servo) {
    double pulsewidth = angle * (2000 / 270) + 500;
    digitalWrite(servo,HIGH);
    delayMicroseconds(int(pulsewidth));
    digitalWrite(servo,LOW);
    delayMicroseconds(int(20000-pulsewidth));
}

// void pickup() {
//     servoPulse(45, clawServo);
//     delay(1000);
//     servoPulse(45, verticalServo);
//     delay(1000);
//     servoPulse(0, clawServo);
//     delay(1000);
//     servoPulse(0, verticalServo);
// }

void clawTest() {
    servoPulse(0, clawServo);
    delay(1000);
}

// void verticalTest() {

// }


clawTest();
