#include "tvc.h"

Tvc::Tvc(Servos servos, int pitchServoChannel, int yawServoChannel, float servoReduction)
    : servos(servos), pitchServoChannel(pitchServoChannel), yawServoChannel(yawServoChannel), servoReduction(servoReduction) {}

void Tvc::initialize() {
    if (servosLocked) {
        servos.initialize();

        servosLocked = false;

        Serial.println("TVC initialized");
    }
}

void Tvc::uninitialize() {
    if (!servosLocked) {
        servos.uninitialize();

        servosLocked = true;

        Serial.println("TVC uninitialized");
    }
}

void Tvc::moveRaw(float pitch, float yaw) {
    this->pitch = pitch;
    this->yaw = yaw;

    servos.move(pitchServoChannel, pitch);
    servos.move(yawServoChannel, yaw);
}

float scaleControlSignal(float controlSignal, float minOutput, float maxOutput) {
    // Saturate the control signal within the desired range
    if (controlSignal < minOutput) controlSignal = minOutput;
    if (controlSignal > maxOutput) controlSignal = maxOutput;

    return controlSignal;
}

void Tvc::move(float pitch, float yaw) {

    static unsigned long lastTime = micros(); // Initialize lastTime with the current time
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0; // Convert microseconds to seconds
    lastTime = currentTime;

    float errorPitch = -pitch;
    integralPitch += errorPitch * dt;
    integralPitch = constrain(integralPitch, -10, 10);
    float derivativePitch = (errorPitch - prevErrorPitch) / dt;
    float controlSignalPitch = Kp * errorPitch + Ki * integralPitch + Kd * derivativePitch;
    prevErrorPitch = errorPitch;

    float errorYaw = -yaw;
    integralYaw += errorYaw * dt;
    integralYaw = constrain(integralYaw, -10, 10);
    float derivativeYaw = (errorYaw - prevErrorYaw) / dt ;
    float controlSignalYaw = Kp * errorYaw + Ki * integralYaw + Kd * derivativeYaw;
    prevErrorYaw = errorYaw;

    controlSignalPitch = scaleControlSignal(controlSignalPitch, -25, 25);
    controlSignalYaw = scaleControlSignal(controlSignalYaw, -25, 25);

    //Serial.print("delta time ");
    //Serial.println(dt);
    // test comment
    // Serial.println("Control signal pitch and control signal yaw:");
    // Serial.println(controlSignalPitch);
    // Serial.println(controlSignalYaw);

    controlSignalPitch *= servoReduction;
    controlSignalYaw *= servoReduction;

    moveRaw(controlSignalPitch + 90, controlSignalYaw + 90);
}
