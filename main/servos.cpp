#include "servos.h"
#include <Arduino.h> // for constrain() and map()

Servos::Servos(int servoMin, int servoMax)
    : pwm(Adafruit_PWMServoDriver()), servoMin(servoMin), servoMax(servoMax) {
        this->initializeOnceOnStartup();
    }

void Servos::initializeOnceOnStartup() {
    pwm.begin();
    pwm.setPWMFreq(50);
}

void Servos::initialize() {}

void Servos::uninitialize() {
    // this should release the servos
    // src: https://forums.adafruit.com/viewtopic.php?t=60404
    pwm.setPWM(0, 0, 0);
}

void Servos::move(int pin, float angle) {
    int pulseLength = map(constrain(angle, 0, 180), 0, 180, servoMin, servoMax);
    pwm.setPWM(pin, 0, pulseLength);
}