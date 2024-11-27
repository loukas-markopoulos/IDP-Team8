#include <Arduino.h>

int verticalServo = 9;
int clawServo = 10;

#define MAX_RANG    (520)
#define ADC_SOLUTION    (1023.0)
int sensityPin = A0;
int magPin = ;      //magnetic sensor pin

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
    
}

void getDistanceAway(float distance) {
    
}
