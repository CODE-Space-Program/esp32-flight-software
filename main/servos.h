#pragma once

#include <Adafruit_PWMServoDriver.h>

/**
 * @brief wrapper for servo driver
 */
class Servos {
public:
    Servos(int servoMin, int servoMax);

    void initialize();

    void move(int channel, float angle);

private:
    Adafruit_PWMServoDriver pwm;
    int servoMin;
    int servoMax;
};