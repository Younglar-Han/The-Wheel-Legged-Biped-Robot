#ifndef MOTORFEEDBACK_HPP
#define MOTORFEEDBACK_HPP

class MotorFeedback
{
public:
    bool hasSpeedFdb;
    float speedFdb;
    float lastSpeedFdb;

    bool hasPositionFdb;
    float positionFdb;
    float lastPositionFdb;

    bool hasAccelerationFdb;
    float accelerationFdb;
    float lastAccelerationFdb;

    bool hasTemperatureFdb;
    float temperatureFdb;

    MotorFeedback():
        hasSpeedFdb(false),
        speedFdb(0.0f),
        lastSpeedFdb(0.0f),

        hasPositionFdb(false),
        positionFdb(0.0f),
        lastPositionFdb(0.0f),

        hasAccelerationFdb(false),
        accelerationFdb(0.0f),
        lastAccelerationFdb(0.0f),

        hasTemperatureFdb(false),
        temperatureFdb(0.0f)
    {
    }
};

#endif
