#ifndef HT04SENSORHANDLER_HPP
#define HT04SENSORHANDLER_HPP

#include "RobotEngine.hpp"
#include "CanMsgDispatcher.hpp"
#include "CanMsgHandler.hpp"
#include "MotorFeedback.hpp"
#include "Math.hpp"

class HT04SensorHandler : public SensorEntity, public CanMsgHandler, public MotorFeedback
{
private:
    uint16_t ecd;
    int16_t speed_raw;
    int16_t given_current;
    uint32_t canId;
    uint8_t motorId;
    CAN_TypeDef *can;
    bool newDataReady;
    const static float Pmin;
    const static float Pmax;
    const static float Vmin;
    const static float Vmax;
    const static float Tmin;
    const static float Tmax;

public:
    uint16_t offset;
    float torqueFdb;

    HT04SensorHandler();

    void RegistSensor(CAN_TypeDef *_can, int _canId)
    {
        can = _can;
        canId = _canId;
        CanMsgDispatcher::Instance()->RegisterHandler(can, canId, this);
    }

    float Uint2Float(int x_int, float x_min, float x_max, int bits)
    {
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }

    virtual void HandleNewCanRxMsg(CanRxMsg *_msg)
    {
        CanMsgHandler::HandleNewCanRxMsg(_msg);
        newDataReady = true;
    }

    virtual bool HasNewData()
    {
        bool _dataReadyShadow = newDataReady;
        newDataReady = false;
        return _dataReadyShadow;
    }

    virtual void Init();
    virtual void Update();
};

#endif
