
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

int8_t NAVIGATION_STATE = 0;

bool LAST_BOX_MAGNETIC = true;

enum DIRECTION {
  STRAIGHT,
  LEFT,
  RIGHT,
  TURNAROUND
};

enum ACTION {
  NO_ACTION,
  //along path actions:
  PICKUP_ALONG_PATH, //for boxes on the line
  SEARCH,
  //end of path actions:
  DEPOSIT,
};

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

struct PATHSTEP {
    int nodeId;                  // Unique ID for each node
    DIRECTION EXIT_DIRECTION;
    ACTION DURING_PATH_ACTION;
    ACTION END_PATH_ACTION;
    int EXPECTED_TIME; //in seconds i guess
};

static PATHSTEP main_path_0[5] = {
    // BOX1 NODE 2
  {0, STRAIGHT, PICKUP_ALONG_PATH, NO_ACTION, 0},
  // PICKUP BOX FUNCTION
  {2, LEFT, NO_ACTION, NO_ACTION, 0},
  {1, RIGHT, NO_ACTION, NO_ACTION, 0},
  {4, STRAIGHT, NO_ACTION, NO_ACTION, 0},
  {12, RIGHT, PICKUP_ALONG_PATH, NO_ACTION, 0},
  // Magnetic (9) from 12
  // Non Magnetic (10) from 12
  // Drop Box
};

static PATHSTEP main_path_1[1] = {
 // BOX2 NODE 5
    // PICKUP BOX FUNCTION
    {5, TURNAROUND, PICKUP_ALONG_PATH, NO_ACTION, 0},
    // Magnetic (9) from 5
    // Non Magnetic (10) from 5
    // Drop Box

};

static PATHSTEP main_path_2[3] = {
 // BOX3 NODE 6-5
    {13, RIGHT, NO_ACTION, NO_ACTION, 0},
    {6, RIGHT, NO_ACTION, NO_ACTION, 0},
    // PICKUP BOX FUNCTION  
    {5, RIGHT, PICKUP_ALONG_PATH, NO_ACTION, 0},
    // Magnetic (9) from 5
    // Non Magnetic (10) from 5
    // Drop Box


};

static PATHSTEP main_path_3[7] = {
     // BOX4 NODE 2-3
    {13, RIGHT, NO_ACTION, NO_ACTION, 0},
    {6, STRAIGHT, NO_ACTION, NO_ACTION, 0},
    {3, RIGHT, NO_ACTION, NO_ACTION, 0},
    // PICKUP BOX FUNCTION
    {2, STRAIGHT, NO_ACTION, NO_ACTION, 0},
    {1, RIGHT, NO_ACTION, NO_ACTION, 0},
    {4, STRAIGHT, NO_ACTION, NO_ACTION, 0},
    {12, RIGHT, PICKUP_ALONG_PATH, NO_ACTION, 0},
    // Magnetic (9) from 12
    // Non Magnetic (10) from 12
    // Drop Box
};

static PATHSTEP main_path_4[3] = {

    // BOX5 OFF NODE 6-5 LINE
    {13, RIGHT, NO_ACTION, NO_ACTION, 0},
    {6, RIGHT, NO_ACTION, NO_ACTION, 0},
    // PICKUP BOX OFF LINE FUNCTION 
    {5, RIGHT, SEARCH, NO_ACTION, 0},
    // Magnetic (9) from 5
    // Non Magnetic (10) from 5
    // Drop Box
 
};

static PATHSTEP main_path_5[5] = {
      // BOX6 OFF NODE 1-2 LINE
    {12, LEFT, NO_ACTION, NO_ACTION, 0},
    {4, STRAIGHT, NO_ACTION, NO_ACTION, 0},
    {1, LEFT, NO_ACTION, NO_ACTION, 0},
    // PICKUP BOX OFF LINE FUNCTION
    {2, STRAIGHT, NO_ACTION, NO_ACTION, 0},
    {3, LEFT, SEARCH, NO_ACTION, 0},
    // Magnetic (9) from 3 and back to start
    // Non Magnetic (9) from 3 and back to start
    // Drop Box
};

static PATHSTEP box1_magnetic[4] = {
    {7, RIGHT, NO_ACTION, NO_ACTION, 0},
    {11, RIGHT, NO_ACTION, NO_ACTION, 0},
    {9, TURNAROUND, NO_ACTION, NO_ACTION, 0},
    {11, RIGHT, NO_ACTION, NO_ACTION, 0},
};

static PATHSTEP box1_nonMagnetic[6] = {
    {7, STRAIGHT, NO_ACTION, NO_ACTION, 0},
    {8, RIGHT, NO_ACTION, NO_ACTION, 0},
    {10, TURNAROUND, NO_ACTION, NO_ACTION, 0},
    {8, LEFT, NO_ACTION, NO_ACTION, 0},
    {7, LEFT, NO_ACTION, NO_ACTION, 0},
    {11, STRAIGHT, NO_ACTION, NO_ACTION, 0}
};

static PATHSTEP box2_magnetic[5] = {
    {11, LEFT, NO_ACTION, NO_ACTION, 0},
    {9, TURNAROUND, NO_ACTION, NO_ACTION, 0},
    {11, LEFT, NO_ACTION, NO_ACTION, 0},
    {7, RIGHT, NO_ACTION, NO_ACTION, 0},
    {8, STRAIGHT, NO_ACTION, NO_ACTION, 0},
};

static PATHSTEP box2_nonMagnetic[5] = {
    {11, STRAIGHT, NO_ACTION, NO_ACTION, 0},
    {7, RIGHT, NO_ACTION, NO_ACTION, 0},
    {8, RIGHT, NO_ACTION, NO_ACTION, 0},
    {10, TURNAROUND, NO_ACTION, NO_ACTION, 0},
    {8, RIGHT, NO_ACTION, NO_ACTION, 0},
};

static PATHSTEP box3_magnetic[6] = {
    {11, LEFT, NO_ACTION, NO_ACTION, 0},
    {9, TURNAROUND, NO_ACTION, NO_ACTION, 0},
    {11, LEFT, NO_ACTION, NO_ACTION, 0},
    {7, RIGHT, NO_ACTION, NO_ACTION, 0},
    {8, STRAIGHT, NO_ACTION, NO_ACTION, 0},
    {13, RIGHT, NO_ACTION, NO_ACTION, 0},
};

static PATHSTEP box3_nonMagnetic[5] = {
    {11, STRAIGHT, NO_ACTION, NO_ACTION, 0},
    {7, RIGHT, NO_ACTION, NO_ACTION, 0},
    {8, RIGHT, NO_ACTION, NO_ACTION, 0},
    {10, TURNAROUND, NO_ACTION, NO_ACTION, 0},
    {8, RIGHT, NO_ACTION, NO_ACTION, 0},
};

static PATHSTEP box4_magnetic[6] = {
    {7, RIGHT, NO_ACTION, NO_ACTION, 0},
    {11, RIGHT, NO_ACTION, NO_ACTION, 0},
    {9, TURNAROUND, NO_ACTION, NO_ACTION, 0},
    {11, LEFT, NO_ACTION, NO_ACTION, 0},
    {7, RIGHT, NO_ACTION, NO_ACTION, 0},
    {8, STRAIGHT, NO_ACTION, NO_ACTION, 0},
};

static PATHSTEP box4_nonMagnetic[4] = {
    {7, STRAIGHT, NO_ACTION, NO_ACTION, 0},
    {8, RIGHT, NO_ACTION, NO_ACTION, 0},
    {10, TURNAROUND, NO_ACTION, NO_ACTION, 0},
    {8, RIGHT, NO_ACTION, NO_ACTION, 0},
};

static PATHSTEP box5_magnetic[3] = {
    {9, TURNAROUND, NO_ACTION, NO_ACTION, 0},
    {11, LEFT, NO_ACTION, NO_ACTION, 0},
    {7, LEFT, NO_ACTION, NO_ACTION, 0},
};

static PATHSTEP box5_nonMagnetic[6] = {
    {11, STRAIGHT, NO_ACTION, NO_ACTION, 0},
    {7, RIGHT, NO_ACTION, NO_ACTION, 0},
    {8, RIGHT, NO_ACTION, NO_ACTION, 0},
    {10, TURNAROUND, NO_ACTION, NO_ACTION, 0},
    {8, LEFT, NO_ACTION, NO_ACTION, 0},
    {7, STRAIGHT, NO_ACTION, NO_ACTION, 0},
};

static PATHSTEP box6_magnetic[10] = {
    {6, LEFT, NO_ACTION, NO_ACTION, 0},
    {5, RIGHT, NO_ACTION, NO_ACTION, 0},
    {11, LEFT, NO_ACTION, NO_ACTION, 0},
    {9, TURNAROUND, NO_ACTION, NO_ACTION, 0},
    {11, RIGHT, NO_ACTION, NO_ACTION, 0},
    {5, RIGHT, NO_ACTION, NO_ACTION, 0},
    {4, LEFT, NO_ACTION, NO_ACTION, 0},
    {1, LEFT, NO_ACTION, NO_ACTION, 0},
    {2, RIGHT, NO_ACTION, NO_ACTION, 0},
};

static PATHSTEP box6_nonMagnetic[9] = {

    {6, STRAIGHT, NO_ACTION, NO_ACTION, 0},
    {13, LEFT, NO_ACTION, NO_ACTION, 0},
    {8, LEFT, NO_ACTION, NO_ACTION, 0},
    {10, TURNAROUND, NO_ACTION, NO_ACTION, 0},
    {8, RIGHT, NO_ACTION, NO_ACTION, 0},
    {13, RIGHT, NO_ACTION, NO_ACTION, 0},
    {6, STRAIGHT, NO_ACTION, NO_ACTION, 0},
    {3, RIGHT, NO_ACTION, NO_ACTION, 0},
    {2, LEFT, NO_ACTION, NO_ACTION, 0},
};

//AWARENESS FUNCTIONS

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

//BASE MOVEMENT FUNCTIONS

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

//ALONG-PATH FUNCTIONS

void LineFollowToJunction (int time) {
  bool following = true;
  while (following) {
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

    case UNSURE: //ok we're either at a junction or have fallen off the line. TODO write the rest of the fn code haha
      following = false;
      StopDriving();
      //more code here to check for false junctioning
      break;
    }   
  }
}

void GetOntoCourse () {
  Serial.println("i'm driving onto the course");
  bool following = true;
  while (following) {
    Drive(true, 200, 0);
    LINE_FOLLOW_STATE OnCourse = ReadLineFollowSensors();
    if (OnCourse == 0) {
      StopDriving();
      following = false;
    }
  }
  StopDriving();
  delay(1000);
  Serial.println("i'm on the course and getting to the first junction");
  LineFollowToJunction(0);
  Serial.println("i've detected node zero and i'm ready to start the nav loop!");
}

bool PickUpBoxAlongLine(int time) { //true if magnetic
  LineFollowToJunction(time);
  return true; //oh wow it was magnetic, could never have guessed...
};

//TODO: END-PATH FUNCTIONS

//TURN AT END OF JUNCTION

void TurnAtJunction(DIRECTION direction, JUNCTION junction) {
  if (direction == STRAIGHT) {
      Serial.println("I've found a STRAIGHT, been told to take it, and i'm taking it!");
      Drive(true, 200, 0);
      delay(500);
  } else if (direction == LEFT) {
      Serial.println("I've found a RIGHT turn, been told to take it, and i'm taking it!");
      TurnUntilLine(false);
  } else if (direction == RIGHT) {
      Serial.println("I've found a RIGHT turn, been told to take it, and i'm taking it!");
      TurnUntilLine(true);
  } else if (direction == TURNAROUND){
      Serial.println("i'm backing that ass up and turning around");
      TurnAroundUntilLine(true);
  } else { // something has gone badly wrong aaaaaahhhhhhhhhhhh
      while (true) {
        StopDriving();
        Serial.println("I am freaking out!!!!");
      }
      //screamIntoVoid();
  }
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

  delay(1000); //so we can get some stuff done before the thing do
  GetOntoCourse();
}

void ExecutePathSection(PATHSTEP NextSection[], int PathLength) {
  for (int step = 0; step < PathLength; step++) {
    //LIFECYCLE START
    PATHSTEP NextPathStep = NextSection[step];
    Serial.print("i'm at node ");
    Serial.print(NextPathStep.nodeId);
    Serial.print(" turning ");
    Serial.println(NextPathStep.EXIT_DIRECTION);

    // CHECK JUNCTION AND TURN
    JUNCTION NextJunction = AssessJunction();
    TurnAtJunction(NextPathStep.EXIT_DIRECTION, NextJunction);

    //DURING LINE ACTIONS
    switch (NextPathStep.DURING_PATH_ACTION) {
      case NO_ACTION:
        Serial.println("i'm line following to junction");
        LineFollowToJunction(0);
        break;
      case PICKUP_ALONG_PATH:
        Serial.println("i'm picking up a box along path!");
        LAST_BOX_MAGNETIC = PickUpBoxAlongLine(0);
        LAST_BOX_MAGNETIC ? Serial.println("The box was magnetic!") : Serial.println("The box wasn't magnetic!");
        break;
    };

    //END OF LINE ACTIONS

    // switch (NextPathStep.END_PATH_ACTION) {
    //   case NO_ACTION:
    //     break;
    //   case default:
    //   //more stuff will eventually go here
    //   break;
    // };

    //LIFECYCLE END
  };
}

void loop() {

  //CHOOSE PATH SECTION

  //the nav state code starts at node zero so we need to get to the first "junction" before this can start

  switch (NAVIGATION_STATE) {
    case 0:
      Serial.println("i'm executing path segment 0");
      ExecutePathSection(main_path_0, 5);
      break;
    case 1:
      Serial.println("i'm executing path segment 1");
      LAST_BOX_MAGNETIC ? ExecutePathSection(box1_magnetic, 4) : ExecutePathSection(box1_nonMagnetic, 6);
      break;
    case 2:
      Serial.println("i'm executing path segment 2");
      ExecutePathSection(main_path_1, 1);
      break;
    default:
      StopDriving();
      Serial.println("i'm done with this shit, i'm getting a marmalade sandwich");
      //youreDone();
      break;
  };

  NAVIGATION_STATE++;
}