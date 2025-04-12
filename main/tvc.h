#pragma once

#include "servos.h"

/**
 * @brief Thrust Vector Control (TVC) class.
 */
class Tvc {
public:
    int pitchServoChannel;
    int yawServoChannel;
    float pitch;
    float yaw;

    Tvc(Servos servos, int pitchServoChannel, int yawServoChannel);

    void initialize();
    void uninitialize();

    /**
     * @brief Move the TVC to the specified angles.
     * @param pitch Angle in degrees [-90, 90].
     * @param yaw Angle in degrees [-90, 90].
     */
    void moveRaw(float pitch, float yaw);

    /**
     * @brief Move the TVC using PID control to stabilize around 0°.
     * @param pitch Current pitch angle.
     * @param yaw Current yaw angle.
     */
    void move(float pitch, float yaw);

    bool servosLocked = true;

private:
    Servos servos;

    // PID constants
    float Kp = 1;
    float Ki = 0.9; // for the V1 TVC 0.4 for the Ki works good
    float Kd = 0.10;

    float prevErrorPitch = 0, prevErrorYaw = 0;
    float integralPitch = 0, integralYaw = 0;
};
