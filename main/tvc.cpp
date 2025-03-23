#include "tvc.h"

Tvc::Tvc(Servos servos, int pitchServoChannel, int yawServoChannel)
    : servos(servos), pitchServoChannel(pitchServoChannel), yawServoChannel(yawServoChannel) {}

void Tvc::initialize() {
    if (!isActive) {
        servos.initialize();

        isActive = true;

        Serial.println("TVC initialized");
    }
}

void Tvc::uninitialize() {
    if (isActive) {
        servos.uninitialize();

        isActive = false;

        Serial.println("TVC uninitialized");
    }
}

void Tvc::moveRaw(float pitch, float yaw) {
    this->pitch = pitch;
    this->yaw = yaw;

    if (!isActive) {
        Serial.println("WARNING Tvc.moveRaw called while servos are inactive, ignoring");
        return;
    }
    servos.move(pitchServoChannel, pitch);
    servos.move(yawServoChannel, yaw);
}

void Tvc::move(float pitch, float yaw) {
    float errorPitch = -pitch;
    integralPitch += errorPitch;
    float derivativePitch = errorPitch - prevErrorPitch;
    float controlSignalPitch = Kp * errorPitch + Ki * integralPitch + Kd * derivativePitch;
    prevErrorPitch = errorPitch;

    float errorYaw = -yaw;
    integralYaw += errorYaw;
    float derivativeYaw = errorYaw - prevErrorYaw;
    float controlSignalYaw = Kp * errorYaw + Ki * integralYaw + Kd * derivativeYaw;
    prevErrorYaw = errorYaw;

    moveRaw(controlSignalPitch, controlSignalYaw);
}
