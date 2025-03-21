#pragma once

class TvcTest
{
public:
    TvcTest(const double maxDegrees) : maxDegrees(maxDegrees)
    {
        maxPitch = maxDegrees;
        minPitch = -maxDegrees;

        maxYaw = maxDegrees;
        minYaw = -maxDegrees;
    }

    int getNewPitch()
    {
        float newPitch;

        if (pitchDirection)
        {
            newPitch = lastPitch + stepSize;
        }
        else
        {
            newPitch = lastPitch - stepSize;
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
    int getNewYaw()
    {
        float newYaw;

        if (yawDirection)
        {
            newYaw = lastYaw + stepSize;
        }
        else
        {
            newYaw = lastYaw - stepSize;
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
    float maxDegrees;

    float lastYaw;
    float lastPitch;

    float maxYaw = 5;
    float minYaw = -5;
    float maxPitch = 5;
    float minPitch = -5;

    float stepSize = 0.1;

    bool yawDirection = true;
    bool pitchDirection = true;
};