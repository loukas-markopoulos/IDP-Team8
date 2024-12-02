// new main file just to test angles of servos

#include <Arduino.h>

int verticalServo = 9;
int clawServo = 10;

void setup() {
    pinMode(verticalServo, OUTPUT);
    pinMode(clawServo, OUTPUT);
}

void servoPulse(int angle, int servo) {
    double pulsewidth = angle * (2000 / 270) + 500;
    digitalWrite(servo,HIGH);
    delayMicroseconds(int(pulsewidth));
    digitalWrite(servo,LOW);
    delayMicroseconds(int(20000-pulsewidth));
}

void testServo(int servo) {
    servoPulse(20, servo);
}

void loop() {
    testServo(verticalServo);
    break;
}
