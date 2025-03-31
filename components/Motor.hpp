#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "RobotEngine.hpp"
#include "MotorFeedback.hpp"

class Motor : public ActuatorEntity
{
public:
    Motor(EntityClassType _classType):
          ActuatorEntity(_classType),
          controlMode{RELAX_MODE},
          pFeedback{nullptr}
    {}

    enum MotorControlModeType
    {
        RELAX_MODE,
        SPD_MODE,
        POS_MODE, 
        CUR_MODE       
    };

    MotorControlModeType controlMode;
    MotorFeedback* pFeedback;
    float speedSet;
    float positionSet;
    float accelerationSet;

    virtual void Update() = 0;
};

#endif
