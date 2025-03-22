#pragma once

#include "tvc.h"

class TvcTest
{
public:
    TvcTest()
    {
    }

    bool isInProgress()
    {
        return millis() - startedMs < durationMs;
    }

    void start(const double maxDegrees, const float stepSize, const int duration = 5000)
    {
        startedMs = millis();
        durationMs = duration;

        maxPitch = maxDegrees;
        minPitch = -maxDegrees;

        maxYaw = maxDegrees;
        minYaw = -maxDegrees;

        stepSizeYaw = stepSize;
        stepSizePitch = stepSize;
    }

    float getNewPitch()
    {
        float newPitch;

        if (pitchDirection)
        {
            newPitch = lastPitch + stepSizePitch;
        }
        else
        {
            newPitch = lastPitch - stepSizePitch;
        }

        if (newPitch >= maxPitch)
        {
            pitchDirection = false;
        }
        if (newPitch <= minPitch)
        {
            pitchDirection = true;
        }
        lastPitch = newPitch;

        return newPitch;
    }

    float getNewYaw()
    {
        float newYaw;

        if (yawDirection)
        {
            newYaw = lastYaw + stepSizeYaw;
        }
        else
        {
            newYaw = lastYaw - stepSizeYaw;
        }

        if (newYaw >= maxYaw)
        {
            yawDirection = false;
        }
        if (newYaw <= minYaw)
        {
            yawDirection = true;
        }
        lastYaw = newYaw;

        return newYaw;
    }
    void tick()
    {
        if (isInProgress())
        {
            float newPitch = getNewPitch();
            float newYaw = getNewYaw();

            moveServos(newPitch, newYaw);
        }
    }

private:
    int startedMs = 0;
    int durationMs = 0;

    float lastYaw = 0;
    float lastPitch = 0;

    float maxYaw = 5;
    float minYaw = -5;
    float maxPitch = 5;
    float minPitch = -5;

    float stepSizeYaw = 0;
    float stepSizePitch = 0;

    bool yawDirection = true;
    bool pitchDirection = true;
};