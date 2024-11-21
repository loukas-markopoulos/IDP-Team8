// Picking up boxes:

// For boxes on the line:
// Detect box is there
// Approach box until within lift range
// Close pickup mechanism
// Lift box 
// Detect magnetic or non magnetic

// For boxes off the line:
// Go to point on line opposite box
// Turn  90deg to box 
// Go straight
// Detect box is there
// Approach box until within lift range
// Close pickup mechanism 
// Lift box 
// Detect magnetic or non magnetic
// 180deg turn 
// Straight until line is detected
// Turn 90deg same direction as before (2nd line in this section)

// --------------------------------------------------------------------

// Dropping off boxes:
// Lower pickup mechanism to previous height
// Open pickup mechanism


#include <Arduino.h>
#include <Servo.h>

Servo verticalServo;            // controls the height of the claw (0deg in vertical position)
Servo clawServo;                // controls opening and closing of the claw (0deg in closed position)

const int trigPin = ;
const int echoPin= ;

void setup() {
    verticalServo.attach(9);
    clawServo.attach(10);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

void pickup() {
    // clawServo open whilst in vertical position
    clawServo.write(90);        // CHANGE ANGLE FOR GEAR RATIO
    delay(1000);
    // verticalServo lowers claw
    verticalServo.write(90);    // should be exactly 90
    delay(1000);
    // clawServo grabs box 
    clawServo.write(0);
    delay(1000);
    // vertical servo raises claw
    verticalServo.write(0);
    delay(1000);                // then take current reading (for weight)
}

void loop() {

    // Detect distance
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    long distance = duration * 0.034 / 2;   // distance of object in cm


    // stop when box is at desired distance
    if (distance < 2){                      // DISTANCE TO BE SET PROPERLY LATER
        
        
    }                      
    // average next 15 values (max US f 30Hz)
    // if average <= desired distance:
    // run pickup
    // else carry on 
}