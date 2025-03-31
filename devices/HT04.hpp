#ifndef HT04_HPP
#define HT04_HPP

#include "Motor.hpp"
#include "HT04SensorHandler.hpp"
#include "Pid.hpp"
#include "CanManager.hpp"
#include "common_define.h"

class HT04 : public Motor
{
public:
    HT04SensorHandler sensorFeedBack;
    Pid pidSpeed;
    Pid pidPosition;
    float currentSet;
    float torqueSet;
    CAN_TypeDef *can;
    uint32_t canId;
    bool lastRelax;
	uint32_t lastUpdateTick;

    HT04();

    void RegistMotor(CAN_TypeDef *_can, int _canId)
    {
        can = _can;
        canId = _canId;
        sensorFeedBack.RegistSensor(_can, _canId);
    }

    void Motivate();

    void SendControlCommand(float current);

    void EnterMotorMode();

    void ExitMotorMode();

    void SetZeroPos();

    uint16_t Float2Uint(float x, float x_min, float x_max, uint8_t bits)
    {
        float span = x_max - x_min;
        float offset = x_min;

        return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
    }

    virtual void Update();
};

#endif
