// new main file to test pick up functions

#include <Arduino.h>

int verticalServo = 9;
int clawServo = 10;

#define MAX_RANG    (520)
#define ADC_SOLUTION    (1023.0)
int sensityPin = A0;
int magPin = ;                  // FIND OUT magnetic sensor pin
float dist_t, sensity_t;


void setup() {
    pinMode(verticalServo, OUTPUT);
    pinMode(clawServo, OUTPUT);
    pinMode(magPin, INPUT);
}

void servoPulse(int angle, int servo) {
    double pulsewidth = angle * (2000 / 270) + 500;
    digitalWrite(servo,HIGH);
    delayMicroseconds(int(pulsewidth));
    digitalWrite(servo,LOW);
    delayMicroseconds(int(20000-pulsewidth));
}

float returnDistance() {    
    sensity_t = analogRead(sensityPin);
    dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;
    return dist_t;
}

float returnAverageDistance(int numReadings) {
    float readingsSum = 0;
    for(int i = 0; i < numReadings; i++) {
        readingsSum += returnDistance();
    }
    float averageReading = readingsSum / numReadings;
    return averageReading;
}

void getDistanceAway(float desiredDistance, float tol) {
    stopDriving();
    while (returnAverageDistance(10) > (desiredDistance + tol)) {
        // forward and redefine distance
        Drive(true, 50, 0);
        delay(200);
        stopDriving()
    }
}

bool magReading(float magSensorDistance) {
    // while (distance > (magSensorDistance + 0.2) ||  distance < (magSensorDistance - 0.2)) {
    // while (distance > (magSensorDistance + 1)) {
    //     // forward and redefine distance
    //     Drive(true, 50, 0);
    //     distance = returnDistance();
    // }

    getDistanceAway(magSensorDistance, 1.0);     // figure out tolerance after testing

    servoPulse(90, clawServo);
    delay(1000);
    servoPulse(90, verticalServo);
    delay(1000);
    servoPulse(0, clawServo);

    //magnetic reading
    val = digitalRead(magPin);
    if (val == HIGH) {
        magnetic = true;
    } else {
        magnetic = false;
    }

    servoPulse(90, clawServo);
    delay(1000);
    return magnetic;
}

void pickup() {
    Drive(true, 20, 0);     // CALIBRATE EITHER SPEED OR DELAY TIME (NEXT LINE) 
    delay(1000);
    stopDriving();

    servoPulse(0, clawServo);
    delay(1000);
    servoPulse(0, verticalServo);
    delay(1000);
}


