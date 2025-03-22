#include "tvc.h"
#include <Wire.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// PID constants
float Kp = 0.55, Ki = 0.65, Kd = 0.09;

float prevErrorPitch = 0, prevErrorYaw = 0;
float integralPitch = 0, integralYaw = 0;

void initializeTVC()
{
    pwm.begin();
    pwm.setPWMFreq(60);
}

void controlTVC(float pitch, float yaw)
{
    float errorPitch = 0 - pitch;
    integralPitch += errorPitch;
    float derivativePitch = errorPitch - prevErrorPitch;
    float controlSignalPitch = Kp * errorPitch + Ki * integralPitch + Kd * derivativePitch;
    prevErrorPitch = errorPitch;

    float errorYaw = 0 - yaw;
    integralYaw += errorYaw;
    float derivativeYaw = errorYaw - prevErrorYaw;
    float controlSignalYaw = Kp * errorYaw + Ki * integralYaw + Kd * derivativeYaw;
    prevErrorYaw = errorYaw;

    int pitchPulseLength = map(constrain(90 + controlSignalPitch, 0, 180), 0, 180, SERVOMIN, SERVOMAX);
    int yawPulseLength = map(constrain(90 + controlSignalYaw, 0, 180), 0, 180, SERVOMIN, SERVOMAX);

    pwm.setPWM(SERVO_PITCH_CHANNEL, 0, pitchPulseLength);
    pwm.setPWM(SERVO_YAW_CHANNEL, 0, yawPulseLength);
}

void moveServos(float pitch, float yaw)
{
    int pitchPulseLength = map(constrain(90 + pitch, 0, 180), 0, 180, SERVOMIN, SERVOMAX);
    int yawPulseLength = map(constrain(90 + yaw, 0, 180), 0, 180, SERVOMIN, SERVOMAX);

    pwm.setPWM(SERVO_PITCH_CHANNEL, 0, pitchPulseLength);
    pwm.setPWM(SERVO_YAW_CHANNEL, 0, yawPulseLength);
}
