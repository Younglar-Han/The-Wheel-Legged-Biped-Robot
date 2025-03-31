#ifndef LK9025_HPP
#define LK9025_HPP

#include "Motor.hpp"
#include "LK9025SensorHandler.hpp"
#include "Pid.hpp"
#include "CanManager.hpp"
#include "LKCanMotorCommander.hpp"

#ifndef PI
#define PI 3.14159265358979f
#endif

class LK9025 : public Motor
{
public:
    LK9025SensorHandler sensorFeedBack;
    Pid pidSpeed;
    Pid pidPosition;
    float currentSet;
    float torqueSet;
    CAN_TypeDef* can;
    uint32_t canId;

    LK9025();

    void RegistMotor(CAN_TypeDef* _can, int _canId)
    {
        can = _can;
        canId = _canId;
        sensorFeedBack.RegistSensor(_can, _canId);
    }

    virtual void Update();
};

#endif
