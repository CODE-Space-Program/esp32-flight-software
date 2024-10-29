#pragma once

#include <ESP32Servo.h>

// Create servo objects
Servo servoPitch;
Servo servoYaw;

float Kp = 1.0, Ki = 0.0, Kd = 0.0;
float prevErrorPitch = 0, prevErrorYaw = 0;
float integralPitch = 0, integralYaw = 0;

// function to initialize the servos with the pins
void initializeTVC() {

    servoPitch.attach(16); // change 0 with the pin number for the pitch servo
    servoYaw.attach(0); // change 0 with the pin number for the yaw servo 
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
    int pitchServoPos = constrain(90 + controlSignalPitch, 0, 180);
    int yawServoPos = constrain(90 + controlSignalYaw, 0, 180);
    servoPitch.write(pitchServoPos);
    servoYaw.write(yawServoPos);
}

