#include "LK9025SensorHandler.hpp"

const float LK9025SensorHandler::RawPos2Rad = 0.00009587379924285f;  /* 2Pi / 65536*/
const float LK9025SensorHandler::RawDps2Rps = 0.0174532925199433f;    /* 2Pi / 360 */

LK9025SensorHandler::LK9025SensorHandler() : SensorEntity(ECT_LK9025Sensor)
{
    hasAccelerationFdb = false;
    hasPositionFdb = true;
    hasSpeedFdb = true;
    hasTemperatureFdb = true;
    newDataReady = false;
}

void LK9025SensorHandler::Init()
{
    SetCanTimeout(true);
    SetTimeoutTick(300);
}

void LK9025SensorHandler::Update()
{
    ecd = (uint16_t)(lastCanMsg.Data[7] << 8 | lastCanMsg.Data[6]);           
    speed_dps = (uint16_t)(lastCanMsg.Data[5] << 8 | lastCanMsg.Data[4]);     
    given_current = (uint16_t)(lastCanMsg.Data[3] << 8 | lastCanMsg.Data[2]); 
    temperate = lastCanMsg.Data[1];

    lastSpeedFdb = speedFdb;

    lastPositionFdb = positionFdb;

    positionFdb = Math::LoopFloatConstrain((float)(ecd - offset) * RawPos2Rad, - Math::Pi, Math::Pi);
    speedFdb = speed_dps * RawDps2Rps;

    torqueFdb = (float)given_current / 392.78f;

    temperatureFdb = (float)temperate;
}
