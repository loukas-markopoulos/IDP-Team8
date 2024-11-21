
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// set up the motors with 1 being left and 2 being right
Adafruit_DCMotor *MotorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *MotorRight = AFMS.getMotor(2);

int8_t WIDE_RIGHT_LINE_SENSOR_PIN = 13;
int8_t WIDE_LEFT_LINE_SENSOR_PIN = 12;
int8_t FRONT_RIGHT_LINE_SENSOR_PIN = 11;
int8_t FRONT_LEFT_LINE_SENSOR_PIN = 10;

int STATE = 1; // 0: idle, 1: moving
int MAIN_PATH_INDEX = 0;
int CURRENT_BOX = 1;

enum DIRECTION {
  STRAIGHT,
  LEFT,
  RIGHT,
  TURNAROUND
};

enum ACTION {
  NO_ACTION,
  DEPOSIT,
  PICKUP_ALONG_PATH, //for boxes on the line
  SEARCH,
};

enum LINE_FOLLOW_STATE {
  CENTRAL, //front two white, side two black
  LINE_TO_RIGHT, //front right white, front left black, side two black
  LINE_TO_LEFT, //front left white, front right black, side two black
  UNSURE // catch-all
};

enum NAVIGATION_STATE {
  FOLLOWING_NODAL_PATH,
  SEEKING_OBJECT,
  LOST
};

struct JUNCTION {
  bool AHEAD;
  bool LEFT;
  bool RIGHT;
};

struct PATHSTEP {
    int nodeId;                  // Unique ID for each node
    DIRECTION exitDirection;        // Direction the robot should take when leaving ("straight","left","right","TurnAroundUntilLine")
    ACTION action;
    bool boxVertical; // is the next box vertical?
};

struct RESULT {
  bool USED;
  bool MAGNETIC;
};

static PATHSTEP main_path[] = {
    // BOX1 NODE 2
    {0, STRAIGHT, NO_ACTION, false},
    // PICKUP BOX FUNCTION
    {2, LEFT, NO_ACTION, false},
    {1, RIGHT, NO_ACTION, false},
    {4, STRAIGHT, NO_ACTION, false},
    {12, RIGHT, PICKUP_ALONG_PATH, false},
    // Magnetic (9) from 12
    // Non Magnetic (10) from 12
    // Drop Box


    // BOX2 NODE 5
    // PICKUP BOX FUNCTION
    {5, TURNAROUND, PICKUP_ALONG_PATH, false},
    // Magnetic (9) from 5
    // Non Magnetic (10) from 5
    // Drop Box


    // BOX3 NODE 6-5
    {13, RIGHT, NO_ACTION, false},
    {6, RIGHT, NO_ACTION, false},
    // PICKUP BOX FUNCTION  
    {5, RIGHT, PICKUP_ALONG_PATH, false},
    // Magnetic (9) from 5
    // Non Magnetic (10) from 5
    // Drop Box


    // BOX4 NODE 2-3
    {13, RIGHT, NO_ACTION, false},
    {6, STRAIGHT, NO_ACTION, false},
    {3, RIGHT, NO_ACTION, false},
    // PICKUP BOX FUNCTION
    {2, STRAIGHT, NO_ACTION, false},
    {1, RIGHT, NO_ACTION, false},
    {4, STRAIGHT, NO_ACTION, false},
    {12, RIGHT, PICKUP_ALONG_PATH, false},
    // Magnetic (9) from 12
    // Non Magnetic (10) from 12
    // Drop Box


    // BOX5 OFF NODE 6-5 LINE
    {13, RIGHT, NO_ACTION, false},
    {6, RIGHT, NO_ACTION, false},
    // PICKUP BOX OFF LINE FUNCTION 
    {5, RIGHT, SEARCH, false},
    // Magnetic (9) from 5
    // Non Magnetic (10) from 5
    // Drop Box


    // BOX6 OFF NODE 1-2 LINE
    {12, LEFT, NO_ACTION, false},
    {4, STRAIGHT, NO_ACTION, false},
    {1, LEFT, NO_ACTION, false},
    // PICKUP BOX OFF LINE FUNCTION
    {2, STRAIGHT, NO_ACTION, false},
    {3, LEFT, SEARCH, false},
    // Magnetic (9) from 3 and back to start
    // Non Magnetic (9) from 3 and back to start
    // Drop Box
};

static PATHSTEP box1_magnetic[] = {
    {7, RIGHT, NO_ACTION, false},
    {11, RIGHT, NO_ACTION, false},
    {9, TURNAROUND, NO_ACTION, false},
    {11, RIGHT, NO_ACTION, false},
};

static PATHSTEP box1_nonMagnetic[] = {
    {7, STRAIGHT, NO_ACTION, false},
    {8, RIGHT, NO_ACTION, false},
    {10, TURNAROUND, NO_ACTION, false},
    {8, LEFT, NO_ACTION, false},
    {7, LEFT, NO_ACTION, false},
    {11, STRAIGHT, NO_ACTION, false}
};

static PATHSTEP box2_magnetic[] = {
    {11, LEFT, NO_ACTION, false},
    {9, TURNAROUND, NO_ACTION, false},
    {11, LEFT, NO_ACTION, false},
    {7, RIGHT, NO_ACTION, false},
    {8, STRAIGHT, NO_ACTION, false},
};

static PATHSTEP box2_nonMagnetic[] = {
    {11, STRAIGHT, NO_ACTION, false},
    {7, RIGHT, NO_ACTION, false},
    {8, RIGHT, NO_ACTION, false},
    {10, TURNAROUND, NO_ACTION, false},
    {8, RIGHT, NO_ACTION, false},
};

static PATHSTEP box3_magnetic[] = {
    {11, LEFT, NO_ACTION, false},
    {9, TURNAROUND, NO_ACTION, false},
    {11, LEFT, NO_ACTION, false},
    {7, RIGHT, NO_ACTION, false},
    {8, STRAIGHT, NO_ACTION, false},
    {13, RIGHT, NO_ACTION, false},
};

static PATHSTEP box3_nonMagnetic[] = {
    {11, STRAIGHT, NO_ACTION, false},
    {7, RIGHT, NO_ACTION, false},
    {8, RIGHT, NO_ACTION, false},
    {10, TURNAROUND, NO_ACTION, false},
    {8, RIGHT, NO_ACTION, false},
};

static PATHSTEP box4_magnetic[] = {
    {7, RIGHT, NO_ACTION, false},
    {11, RIGHT, NO_ACTION, false},
    {9, TURNAROUND, NO_ACTION, false},
    {11, LEFT, NO_ACTION, false},
    {7, RIGHT, NO_ACTION, false},
    {8, STRAIGHT, NO_ACTION, false},
};

static PATHSTEP box4_nonMagnetic[] = {
    {7, STRAIGHT, NO_ACTION, false},
    {8, RIGHT, NO_ACTION, false},
    {10, TURNAROUND, NO_ACTION, false},
    {8, RIGHT, NO_ACTION, false},
};

static PATHSTEP box5_magnetic[] = {
    {11, LEFT, NO_ACTION, false},
    {9, TURNAROUND, NO_ACTION, false},
    {11, LEFT, NO_ACTION, false},
    {7, LEFT, NO_ACTION, false},
};

static PATHSTEP box5_nonMagnetic[] = {
    {11, STRAIGHT, NO_ACTION, false},
    {7, RIGHT, NO_ACTION, false},
    {8, RIGHT, NO_ACTION, false},
    {10, TURNAROUND, NO_ACTION, false},
    {8, LEFT, NO_ACTION, false},
    {7, STRAIGHT, NO_ACTION, false},
};

static PATHSTEP box6_magnetic[] = {
    {6, LEFT, NO_ACTION, false},
    {5, RIGHT, NO_ACTION, false},
    {11, LEFT, NO_ACTION, false},
    {9, TURNAROUND, NO_ACTION, false},
    {11, RIGHT, NO_ACTION, false},
    {5, RIGHT, NO_ACTION, false},
    {4, LEFT, NO_ACTION, false},
    {1, LEFT, NO_ACTION, false},
    {2, RIGHT, NO_ACTION, false},
};

static PATHSTEP box6_nonMagnetic[] = {

    {6, STRAIGHT, NO_ACTION, false},
    {13, LEFT, NO_ACTION, false},
    {8, LEFT, NO_ACTION, false},
    {10, TURNAROUND, NO_ACTION, false},
    {8, RIGHT, NO_ACTION, false},
    {13, RIGHT, NO_ACTION, false},
    {6, STRAIGHT, NO_ACTION, false},
    {3, RIGHT, NO_ACTION, false},
    {2, LEFT, NO_ACTION, false},
};

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

void TurnAroundUntilLine(bool clockwise) {
  double power = 200;
  if (clockwise) {
    MotorLeft->run(FORWARD);
    MotorRight->run(BACKWARD);
  } else {
    MotorLeft->run(BACKWARD);
    MotorRight->run(FORWARD);
  }
  MotorLeft->setSpeed(power);
  MotorRight->setSpeed(power);
  delay(1800);
  bool turning = true;
  while (turning) {
    bool frontLeftSensor = digitalRead(FRONT_LEFT_LINE_SENSOR_PIN);
    bool frontRightSensor = digitalRead(FRONT_RIGHT_LINE_SENSOR_PIN);
    turning = !(frontLeftSensor and frontRightSensor);
  };
  StopDriving();
};

void TurnUntilLine(bool right) {
  int16_t differential = right ? -100 : 100; //80 seems perfect for the right turn but the left turn is a bit crap
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

bool PickUpBox() { // returns true if the box is magnetic
  CURRENT_BOX++;
  return true;
};

RESULT ExecutePathStepAtJunction(PATHSTEP pathstep, JUNCTION junction) {
  DIRECTION nextDirection = pathstep.exitDirection;
  ACTION nextAction = pathstep.action;
  if (nextDirection == STRAIGHT) {
      Drive(true, 200, 0);
      delay(500);
  } else if (nextDirection == LEFT) {
      TurnUntilLine(false);
  } else if (nextDirection == RIGHT) {
      TurnUntilLine(true);
  } else if (nextDirection == TURNAROUND){
      TurnAroundUntilLine(true);
  } else { // something has gone badly wrong
      StopDriving();
      STATE = 0;
  }
  if ((nextAction == SEARCH) or (nextAction == PICKUP_ALONG_PATH)) {
    bool boxMagnetic = PickUpBox();
    return {true, boxMagnetic};
  };
  return {false, false};
};

void ExecuteSubPath (PATHSTEP* SubPath) {
  for (int SUB_PATH_INDEX = 0; SUB_PATH_INDEX < sizeof(SubPath); SUB_PATH_INDEX++) {
    JUNCTION NewJunction = LineFollowToJunction();
    PATHSTEP NextPathStep = *(SubPath + SUB_PATH_INDEX);
    RESULT result = ExecutePathStepAtJunction(NextPathStep, NewJunction);
  }
}

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

void loop() {
  if (STATE == 1) {
    JUNCTION newJunction = LineFollowToJunction();
    PATHSTEP nextPathstep = main_path[MAIN_PATH_INDEX];
    RESULT Result = ExecutePathStepAtJunction(nextPathstep, newJunction);

    if (Result.USED) {
      switch (CURRENT_BOX) {
        case 1:
          Result.MAGNETIC ? ExecuteSubPath(box1_magnetic) : ExecuteSubPath(box1_nonMagnetic);
          break;
        case 2:
          Result.MAGNETIC ? ExecuteSubPath(box2_magnetic) : ExecuteSubPath(box2_nonMagnetic);
          break;
        case 3:
          Result.MAGNETIC ? ExecuteSubPath(box3_magnetic) : ExecuteSubPath(box3_nonMagnetic);
          break;
        case 4:
          Result.MAGNETIC ? ExecuteSubPath(box4_magnetic) : ExecuteSubPath(box4_nonMagnetic);
          break;
        case 5:
          Result.MAGNETIC ? ExecuteSubPath(box5_magnetic) : ExecuteSubPath(box5_nonMagnetic);
          break;
        case 6:
          Result.MAGNETIC ? ExecuteSubPath(box6_magnetic) : ExecuteSubPath(box6_nonMagnetic);
          break;
      }
    };

    delay(1000);

    MAIN_PATH_INDEX++;
  }
}