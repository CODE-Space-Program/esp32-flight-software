#pragma once

#include <Adafruit_PWMServoDriver.h>

#define SERVO_PITCH_CHANNEL 0
#define SERVO_YAW_CHANNEL 1
#define SERVOMIN 150
#define SERVOMAX 600
#define ANGLE_LIMIT 5
#define STEP_SIZE 1

extern Adafruit_PWMServoDriver pwm;

void initializeTVC();
void controlTVC(float pitch, float yaw);
void moveServos(float pitch, float yaw);
