#pragma once

bool DO_CIRCLE = true;

float angle = 0.0f;
float centerPitch = 90.0f;
float centerYaw = 90.0f;

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
        if (DO_CIRCLE) {
            float newPitch = centerPitch + maxPitch * sin(angle);
            lastPitch = newPitch;
            return newPitch;
        }
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
        if (DO_CIRCLE) {
            float newYaw = centerYaw + maxPitch * cos(angle);
            lastYaw = newYaw;
            angle += stepSizeYaw; // Advance the shared angle
            return newYaw;
        }
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