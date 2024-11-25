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

Servo verticalServo;            // initialise servo that controls the height of the claw (0deg in vertical position)
Servo clawServo;                // initialise servo that controls opening and closing of the claw (0deg in closed position)

const int trigPin = ;           // ENTER PIN FOR TRIGGER
const int echoPin = ;           // ENTER PIN FOR ECHO

void setup() {
    verticalServo.attach(9);
    clawServo.attach(10);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

void pickup() {
    // NOTE: careful with absolute values of angles, need to first know reference angles
    // alternatively, can use .read() to get current angle then add 90, but this may introduce (cumulative) error
    
    // open clawServo whilst in vertical position
    clawServo.write(90);        // CHANGE ANGLE FOR GEAR RATIO
    delay(1000);
    // verticalServo lowers claw
    verticalServo.write(90);    // should be exactly 90
    delay(1000);
    // clawServo closes to grab box 
    clawServo.write(0);
    delay(1000);
    // vertical servo raises claw
    verticalServo.write(0);
    delay(1000);                // then implement code to take current reading (for weight)
}

double returnDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    double duration = pulseIn(echoPin, HIGH);
    double distance = duration * 0.034 / 2;   // distance of object in cm

    return distance;
}

double getAverageReadings(int numReadings, double minDistance) {
    float readingsSum = 0;
    for(int i = 0; i < numReadings; i++) {
        readingsSum += returnDistance();
    }
    double averageReading = readingsSum / numReadings;
    return averageReading;
}

void loop() {
    // Move forward

    // Detect distance
    long distance = returnDistance();

    // stop when box is at desired distance
    if (distance < 2) {
        if (getAverageReadings(10, 2) < 2.3) {  // this loop gets more readings and averages to confirm object indeed within range (i.e. not random fluctuation)
            pickup();
        }
    }
    // average next 15 values (max US f 30Hz)
    // if average <= desired distance:
    // run pickup
    // else carry on 
}

