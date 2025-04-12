#include "tvc.h"

Tvc::Tvc(Servos servos, int pitchServoChannel, int yawServoChannel)
    : servos(servos), pitchServoChannel(pitchServoChannel), yawServoChannel(yawServoChannel) {}

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

    float actualPitch = datapoint.pitch_inverted ? -pitch : pitch;
    float actualYaw = datapoint.yaw_inverted ? -yaw : yaw;

    float finalPitch = datapoint.pitch_and_yaw_swiched ? actualYaw : actualPitch;
    float finalYaw = datapoint.pitch_and_yaw_swiched ? actualPitch : actualYaw;

    static unsigned long lastTime = micros(); // Initialize lastTime with the current time
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0; // Convert microseconds to seconds
    lastTime = currentTime;

    float errorPitch = -finalPitch;
    integralPitch += errorPitch * dt;
    integralPitch = constrain(integralPitch, -10, 10);
    float derivativePitch = (errorPitch - prevErrorPitch) / dt;
    float controlSignalPitch = datapoint.kp * errorPitch + datapoint.ki * integralPitch + datapoint.kd * derivativePitch;
    prevErrorPitch = errorPitch;

    float errorYaw = -finalYaw;
    integralYaw += errorYaw * dt;
    integralYaw = constrain(integralYaw, -10, 10);
    float derivativeYaw = (errorYaw - prevErrorYaw) / dt ;
    float controlSignalYaw = datapoint.kp * errorYaw + datapoint.ki * integralYaw + datapoint.kd * derivativeYaw;
    prevErrorYaw = errorYaw;

    controlSignalPitch = scaleControlSignal(controlSignalPitch, -25, 25);
    controlSignalYaw = scaleControlSignal(controlSignalYaw, -25, 25);

    //Serial.print("delta time ");
    //Serial.println(dt);

    // Serial.println("Control signal pitch and control signal yaw:");
    // Serial.println(controlSignalPitch);
    // Serial.println(controlSignalYaw);
    moveRaw(controlSignalPitch + 90, controlSignalYaw + 90);
}
