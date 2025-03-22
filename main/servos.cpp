#include "servos.h"
#include <Arduino.h> // for constrain() and map()

Servos::Servos(int servoMin, int servoMax)
    : pwm(Adafruit_PWMServoDriver()), servoMin(servoMin), servoMax(servoMax) {}

void Servos::initialize() {
    pwm.begin();
    pwm.setPWMFreq(60);
}

void Servos::move(int pin, float angle) {
    int pulseLength = map(constrain(90 + angle, 0, 180), 0, 180, servoMin, servoMax);
    pwm.setPWM(pin, 0, pulseLength);
}