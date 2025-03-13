#pragma once

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create servo objects
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


// Define servo channels
#define SERVO_PITCH_CHANNEL 0
#define SERVO_YAW_CHANNEL 1

// Define min and max pulse length for servos
#define SERVOMIN 150
#define SERVOMAX 600 

float Kp = 0.55, Ki = 0.65, Kd = 0.09;
float prevErrorPitch = 0, prevErrorYaw = 0;
float integralPitch = 0, integralYaw = 0;

// function to initialize the servos with the pins
void initializeTVC() {

    pwm.begin();
    pwm.setPWMFreq(60);
}

// function to control the TVC
void controlTVC(float pitch, float yaw) {

    // PID control for pitch
    float errorPitch = 0 - pitch;
    integralPitch += errorPitch;
    float derivativePitch = errorPitch - prevErrorPitch;
    float controlSignalPitch = Kp * errorPitch + Ki * integralPitch + Kd * derivativePitch;
    prevErrorPitch = errorPitch;

    // PID control for yaw
    float errorYaw = 0 - yaw;
    integralYaw += errorYaw;
    float derivativeYaw = errorYaw - prevErrorYaw;
    float controlSignalYaw = Kp * errorYaw + Ki * integralYaw + Kd * derivativeYaw;
    prevErrorYaw = errorYaw;

    // Control the servos
    int pitchPulseLength = map(constrain(90 + controlSignalPitch, 0 ,180), 0, 180, SERVOMIN, SERVOMAX);
    int yawPulseLength = map(constrain(90 + controlSignalYaw, 0, 180), 0, 180, SERVOMIN, SERVOMAX);

    pwm.setPWM(SERVO_PITCH_CHANNEL, 0, pitchPulseLength);
    pwm.setPWM(SERVO_YAW_CHANNEL, 0, yawPulseLength);
}
