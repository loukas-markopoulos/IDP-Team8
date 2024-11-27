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

#define MAX_RANG    (520)
#define ADC_SOLUTION    (1023.0)
int sensityPin = A0;
int magPin = ;      //magnetic sensor pin

void setup() {
    verticalServo.attach(9);
    clawServo.attach(10);
    pinMode(magPin, INPUT);
}

float dist_t, sensity_t;
float critical_dist;

void loop() {
    // Detect distance
    // Drive
    float distance = returnDistance();

    // stop when box is at desired distance
    if (distance < critical_dist) {
        // STOP
        if (getAverageReadings(10) < (critical_dist + 0.2)) {  // this loop gets more readings and averages to confirm object indeed within range (i.e. not random fluctuation)
            pickup(distance);
        }
    }
}

void pickup(float distance) {
    // NOTE: careful with absolute values of angles, need to first know reference angles
    // alternatively, can use .read() to get current angle then add 90, but this may introduce (cumulative) error
    magReading(distance);

    // go forward a little bit

    clawServo.write(0);
    delay(1000);
    verticalServo.write(0);
    delay(1000);
}

bool magnetic = false;
float magSensorDistance;
// magSensorDistance < critical distance
int val = 0;

bool magReading(float distance) {
    while (distance > (magSensorDistance + 0.2) ||  distance < (magSensorDistance - 0.2)) {
        if (distance > magSensorDistance) {
            // forward and redefine distance 
        }
        else {
            // reverse and redefine distance
        }
    }

    clawServo.write(90);
    delay(1000);
    verticalServo.write(90);
    delay(1000);
    clawServo.write(0);
    delay(1000);

    //magnetic reading
    val = digitalRead(magPin);
    if (val == HIGH) {
        magnetic = true;
    } else {
        magnetic = false;
    }

    clawServo.write(90);
    delay(1000);
    return magnetic;
}

double returnDistance() {
    sensity_t = analogRead(sensityPin);
    dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;
    return dist_t;
}

float getAverageReadings(int numReadings) {
    float readingsSum = 0;
    for(int i = 0; i < numReadings; i++) {
        readingsSum += returnDistance();
    }
    float averageReading = readingsSum / numReadings;
    return averageReading;
}

