#include "HT04SensorHandler.hpp"

const float HT04SensorHandler::Pmin = -95.5f;
const float HT04SensorHandler::Pmax = 95.5f;
const float HT04SensorHandler::Vmin = -45.0f; // rps
const float HT04SensorHandler::Vmax = 45.0f;
const float HT04SensorHandler::Tmin = -29.7f;
const float HT04SensorHandler::Tmax = 29.7f;

HT04SensorHandler::HT04SensorHandler() : SensorEntity(ECT_HT04Sensor)
{
    hasAccelerationFdb = false;
    hasPositionFdb = true;
    hasSpeedFdb = true;
    hasTemperatureFdb = true;
    newDataReady = false;
}

void HT04SensorHandler::Init()
{
    SetCanTimeout(true);
}

void HT04SensorHandler::Update()
{
    motorId = lastCanMsg.Data[0];
    ecd = (uint16_t)(lastCanMsg.Data[1] << 8 | lastCanMsg.Data[2]);           
    speed_raw = (uint16_t)(lastCanMsg.Data[3] << 4 | lastCanMsg.Data[4] >> 4);     
    given_current = (uint16_t)((lastCanMsg.Data[4] & 0x0F)<< 8 | lastCanMsg.Data[5]);

    lastSpeedFdb = speedFdb;

    lastPositionFdb = positionFdb;

    // positionFdb = Uint2Float((int)(Math::LoopFloatConstrain((float)(ecd - offset + 32767), 0, 65535.0f)), Pmin, Pmax, 16);
	positionFdb = Uint2Float(ecd, Pmin, Pmax, 16);
    speedFdb = Uint2Float(speed_raw, Vmin, Vmax, 12);
    torqueFdb = Uint2Float(given_current, Tmin, Tmax, 12);
}
