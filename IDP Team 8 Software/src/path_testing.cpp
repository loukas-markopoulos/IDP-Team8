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

void TurnUntilLine(bool right) {
  int16_t differential = right ? -80 : 80; //80 seems perfect for the right turn but the left turn is a bit crap
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
};

struct PATHSTEP {
    int nodeId;                  // Unique ID for each node
    String exitDirection;        // Direction the robot should take when leaving ("straight","left","right","turnaround")
    String conditionType;        // Allows for if statements
};

PATHSTEP main_path[] = {
    // BOX1 NODE 2
    {0, 'straight', ''},
    // PICKUP BOX FUNCTION
    {2, 'left', ''},
    {1, 'right', ''},
    {4, 'straight', ''},
    {12, 'right', 'checkBox1Type'},
    // Magnetic (9) from 12
    // Non Magnetic (10) from 12
    // Drop Box


    // BOX2 NODE 5
    // PICKUP BOX FUNCTION
    {5, 'turnaround', 'checkBox2Type'},
    // Magnetic (9) from 5
    // Non Magnetic (10) from 5
    // Drop Box


    // BOX3 NODE 6-5
    {13, 'right', ''},
    {6, 'right', ''},
    // PICKUP BOX FUNCTION  
    {5, 'right', 'checkBox3Type'},
    // Magnetic (9) from 5
    // Non Magnetic (10) from 5
    // Drop Box


    // BOX4 NODE 2-3
    {13, 'right', ''},
    {6, 'straight', ''},
    {3, 'right', ''},
    // PICKUP BOX FUNCTION
    {2, 'straight', ''}
    {1, 'right', ''},
    {4, 'straight', ''},
    {12, 'right', 'checkBox4Type'}
    // Magnetic (9) from 12
    // Non Magnetic (10) from 12
    // Drop Box


    // BOX5 OFF NODE 6-5 LINE
    {13, 'right', ''},
    {6, 'right', ''},
    // PICKUP BOX OFF LINE FUNCTION 
    {5, 'right', 'checkBox5Type'},
    // Magnetic (9) from 5
    // Non Magnetic (10) from 5
    // Drop Box


    // BOX6 OFF NODE 1-2 LINE
    {12, 'left', ''},
    {4, 'straight', ''},
    {1, 'left', ''},
    // PICKUP BOX FUNCTION
    {2, 'straight', ''},
    {3, 'left', 'checkBox6Type'},
    // Magnetic (9) from 3 and back to start
    // Non Magnetic (9) from 3 and back to start
    // Drop Box
};

PATHSTEP box1_magnetic[] = {
    {7, 'right', ''},
    {11, 'right', ''},
    {9, 'turn around', ''},
    {11, 'right', ''},
};

PATHSTEP box1_nonMagnetic[] = {
    {7, 'straight', ''},
    {8, 'right', ''},
    {10, 'turn around', ''},
    {8, 'left', ''},
    {7, 'left', ''},
    {11, 'straight', ''}
};

PATHSTEP box2_magnetic[] = {
    {11, 'left', ''},
    {9, 'turn around', ''},
    {11, 'left', ''},
    {7, 'right', ''},
    {8, 'straight', ''},
};

PATHSTEP box2_nonMagnetic[] = {
    {11, 'straight', ''},
    {7, 'right', ''},
    {8, 'right', ''},
    {10, 'turn around', ''},
    {8, 'right', ''},
};

PATHSTEP box3_magnetic[] = {
    {11, 'left', ''},
    {9, 'turn around', ''},
    {11, 'left', ''},
    {7, 'right', ''},
    {8, 'straight', ''},
    {13, 'right', ''},
};

PATHSTEP box3_nonMagnetic[] = {
    {11, 'straight', ''},
    {7, 'right', ''},
    {8, 'right', ''},
    {10, 'turn around', ''},
    {8, 'right', ''},
};

PATHSTEP box4_magnetic[] = {
    {7, 'right', ''},
    {11, 'right', ''},
    {9, 'turn around', ''},
    {11, 'left', ''},
    {7, 'right', ''},
    {8, 'straight', ''},
};

PATHSTEP box4_nonMagnetic[] = {
    {7, 'straight', ''},
    {8, 'right', ''},
    {10, 'turn around', ''},
    {8, 'right', ''},
};

PATHSTEP box5_magnetic[] = {
    {11, 'left', ''},
    {9, 'turn around', ''},
    {11, 'left', ''},
    {7, 'left', ''},
};

PATHSTEP box5_nonMagnetic[] = {
    {11, 'straight', ''},
    {7, 'right', ''},
    {8, 'right', ''},
    {10, 'turn around', ''},
    {8, 'left', ''},
    {7, 'straight', ''},
};

PATHSTEP box6_magnetic[] = {
    {6, 'left', ''},
    {5, 'right', ''},
    {11, 'left', ''},
    {9, 'turn around', ''},
    {11, 'right', ''},
    {5, 'right', ''},
    {4, 'left', ''},
    {1, 'left', ''},
    {2, 'right', ''},
};

PATHSTEP box6_nonMagnetic[] = {
    {6, 'straight', ''},
    {13, 'left', ''},
    {8, 'left', ''},
    {10, 'turn around', ''},
    {8, 'right', ''},
    {13, 'right', ''},
    {6, 'straight', ''},
    {3, 'right', ''},
    {2, 'left', ''},
};


int currentNode = 0;

void loop() {
    bool magnetic = false;
    if (STATE == 1) {
      JUNCTION junction = LineFollowToJunction();
      delay(1000);
      String pathDecider = main_path[currentNode].conditionType;
      if (pathDecider == 'checkBox1Type') {
          if (magnetic) {
                  JUNCTION junction = LineFollowToJunction();
                  delay(1000);
                  String nextAction = box1_magnetic[currentNode].exitDirection;
                  if (nextAction == "straight") {
                      Drive(true, 200, 0);
                      Serial.println("goin' straight");
                      delay(500);
                      } else if (nextAction == "left") {
                      Serial.println("turnin' left");
                      TurnUntilLine(false);
                      } else if (nextAction == "right") {
                      Serial.println("turnin' right");
                      TurnUntilLine(true);
                      } else {
                      Serial.printIn("turnin' 180");
                      TurnUntilLine(???);
                      }
                      currentNode++;
                      delay(1000);
          }
          else {
              JUNCTION junction = LineFollowToJunction();
              delay(1000);
              String nextAction = box1_nonMagnetic[currentNode].exitDirection;
              if (nextAction == "straight") {
                Drive(true, 200, 0);
                Serial.println("goin' straight");
                delay(500);
              } else if (nextAction == "left") {
                Serial.println("turnin' left");
                TurnUntilLine(false);
              } else if (nextAction == "right") {
                Serial.println("turnin' right");
                TurnUntilLine(true);
              } else {
                Serial.println("turnin' 180");
                TurnUntilLine(???);
              }
              currentNode++;
              delay(1000);
          }
      }
      if (pathDecider == 'checkBox2Type') {
          if (magnetic) {
                  JUNCTION junction = LineFollowToJunction();
                  delay(1000);
                  String nextAction = box2_magnetic[currentNode].exitDirection;
                  if (nextAction == "straight") {
                      Drive(true, 200, 0);
                      Serial.println("goin' straight");
                      delay(500);
                      } else if (nextAction == "left") {
                      Serial.println("turnin' left");
                      TurnUntilLine(false);
                      } else if (nextAction == "right") {
                      Serial.println("turnin' right");
                      TurnUntilLine(true);
                      } else {
                      Serial.printIn("turnin' 180");
                      TurnUntilLine(???);
                      }
                      currentNode++;
                      delay(1000);
          }
          else {
              JUNCTION junction = LineFollowToJunction();
              delay(1000);
              String nextAction = box2_nonMagnetic[currentNode].exitDirection;
              if (nextAction == "straight") {
                Drive(true, 200, 0);
                Serial.println("goin' straight");
                delay(500);
              } else if (nextAction == "left") {
                Serial.println("turnin' left");
                TurnUntilLine(false);
              } else if (nextAction == "right") {
                Serial.println("turnin' right");
                TurnUntilLine(true);
              } else {
                Serial.println("turnin' 180");
                TurnUntilLine(???);
              }
              currentNode++;
              delay(1000);
          }
      }
      if (pathDecider == 'checkBox3Type') {
          if (magnetic) {
                  JUNCTION junction = LineFollowToJunction();
                  delay(1000);
                  String nextAction = box3_magnetic[currentNode].exitDirection;
                  if (nextAction == "straight") {
                      Drive(true, 200, 0);
                      Serial.println("goin' straight");
                      delay(500);
                      } else if (nextAction == "left") {
                      Serial.println("turnin' left");
                      TurnUntilLine(false);
                      } else if (nextAction == "right") {
                      Serial.println("turnin' right");
                      TurnUntilLine(true);
                      } else {
                      Serial.printIn("turnin' 180");
                      TurnUntilLine(???);
                      }
                      currentNode++;
                      delay(1000);
          }
          else {
              JUNCTION junction = LineFollowToJunction();
              delay(1000);
              String nextAction = box3_nonMagnetic[currentNode].exitDirection;
              if (nextAction == "straight") {
                Drive(true, 200, 0);
                Serial.println("goin' straight");
                delay(500);
              } else if (nextAction == "left") {
                Serial.println("turnin' left");
                TurnUntilLine(false);
              } else if (nextAction == "right") {
                Serial.println("turnin' right");
                TurnUntilLine(true);
              } else {
                Serial.println("turnin' 180");
                TurnUntilLine(???);
              }
              currentNode++;
              delay(1000);
          }
      }
      if (pathDecider == 'checkBox4Type') {
          if (magnetic) {
                  JUNCTION junction = LineFollowToJunction();
                  delay(1000);
                  String nextAction = box4_magnetic[currentNode].exitDirection;
                  if (nextAction == "straight") {
                      Drive(true, 200, 0);
                      Serial.println("goin' straight");
                      delay(500);
                      } else if (nextAction == "left") {
                      Serial.println("turnin' left");
                      TurnUntilLine(false);
                      } else if (nextAction == "right") {
                      Serial.println("turnin' right");
                      TurnUntilLine(true);
                      } else {
                      Serial.printIn("turnin' 180");
                      TurnUntilLine(???);
                      }
                      currentNode++;
                      delay(1000);
          }
          else {
              JUNCTION junction = LineFollowToJunction();
              delay(1000);
              String nextAction = box4_nonMagnetic[currentNode].exitDirection;
              if (nextAction == "straight") {
                Drive(true, 200, 0);
                Serial.println("goin' straight");
                delay(500);
              } else if (nextAction == "left") {
                Serial.println("turnin' left");
                TurnUntilLine(false);
              } else if (nextAction == "right") {
                Serial.println("turnin' right");
                TurnUntilLine(true);
              } else {
                Serial.println("turnin' 180");
                TurnUntilLine(???);
              }
              currentNode++;
              delay(1000);
          }
      }
      if (pathDecider == 'checkBox5Type') {
          if (magnetic) {
                  JUNCTION junction = LineFollowToJunction();
                  delay(1000);
                  String nextAction = box5_magnetic[currentNode].exitDirection;
                  if (nextAction == "straight") {
                      Drive(true, 200, 0);
                      Serial.println("goin' straight");
                      delay(500);
                      } else if (nextAction == "left") {
                      Serial.println("turnin' left");
                      TurnUntilLine(false);
                      } else if (nextAction == "right") {
                      Serial.println("turnin' right");
                      TurnUntilLine(true);
                      } else {
                      Serial.printIn("turnin' 180");
                      TurnUntilLine(???);
                      }
                      currentNode++;
                      delay(1000);
          }
          else {
              JUNCTION junction = LineFollowToJunction();
              delay(1000);
              String nextAction = box5_nonMagnetic[currentNode].exitDirection;
              if (nextAction == "straight") {
                Drive(true, 200, 0);
                Serial.println("goin' straight");
                delay(500);
              } else if (nextAction == "left") {
                Serial.println("turnin' left");
                TurnUntilLine(false);
              } else if (nextAction == "right") {
                Serial.println("turnin' right");
                TurnUntilLine(true);
              } else {
                Serial.println("turnin' 180");
                TurnUntilLine(???);
              }
              currentNode++;
              delay(1000);
          }
      }
      if (pathDecider == 'checkBox6Type') {
          if (magnetic) {
                  JUNCTION junction = LineFollowToJunction();
                  delay(1000);
                  String nextAction = box6_magnetic[currentNode].exitDirection;
                  if (nextAction == "straight") {
                      Drive(true, 200, 0);
                      Serial.println("goin' straight");
                      delay(500);
                      } else if (nextAction == "left") {
                      Serial.println("turnin' left");
                      TurnUntilLine(false);
                      } else if (nextAction == "right") {
                      Serial.println("turnin' right");
                      TurnUntilLine(true);
                      } else {
                      Serial.printIn("turnin' 180");
                      TurnUntilLine(???);
                      }
                      currentNode++;
                      delay(1000);
                      STATE = 0;
          }
          else {
              JUNCTION junction = LineFollowToJunction();
              delay(1000);
              String nextAction = box6_nonMagnetic[currentNode].exitDirection;
              if (nextAction == "straight") {
                Drive(true, 200, 0);
                Serial.println("goin' straight");
                delay(500);
              } else if (nextAction == "left") {
                Serial.println("turnin' left");
                TurnUntilLine(false);
              } else if (nextAction == "right") {
                Serial.println("turnin' right");
                TurnUntilLine(true);
              } else {
                Serial.println("turnin' 180");
                TurnUntilLine(???);
              }
              currentNode++;
              delay(1000);
              STATE = 0;
          }
      }
      

      String nextAction = main_path[currentNode].exitDirection;
              if (nextAction == "straight") {
                Drive(true, 200, 0);
                Serial.println("goin' straight");
                delay(500);
              } else if (nextAction == "left") {
                Serial.println("turnin' left");
                TurnUntilLine(false);
              } else if (nextAction == "right") {
                Serial.println("turnin' right");
                TurnUntilLine(true);
              } else {
                Serial.println("turnin' 180");
                TurnUntilLine(???);
              }
              currentNode++;
              delay(1000);
    }
    else {
      // STOP THE ROBOT!!!
    }
}
