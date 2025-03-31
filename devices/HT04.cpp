#include "HT04.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

HT04::HT04() : Motor(ECT_HT04)
{
    pFeedback = &sensorFeedBack;
    controlMode = Motor::CUR_MODE;
    positionSet = 0.0f;
    speedSet = 0.0f;
    accelerationSet = 0.0f;
    currentSet = 0.0f;
    torqueSet = 0.0f;
    EnterMotorMode();
    lastRelax = true;
    SetDefaultTicksToUpdate(2);
}

void HT04::Update()
{
	lastUpdateTick = Time::GetTick();
    if (controlMode == RELAX_MODE)
    {
        if (!lastRelax)
        {
            lastRelax = true;
        }
        ExitMotorMode();
        return;
    }

    if (lastRelax)
    {
        EnterMotorMode();
        lastRelax = false;
		return;
    }

    if (controlMode == CUR_MODE)
    {
        currentSet = Float2Uint(torqueSet, -29.7f, 29.7f, 12);
        currentSet = Math::FloatConstrain(currentSet, 0, 4095); // max: 4095
        SendControlCommand(currentSet);
        return;
    }

    float _spdFdb = pFeedback->speedFdb;

    if (controlMode == POS_MODE)
    {
        pidPosition.ref = positionSet;

        pidPosition.fdb = sensorFeedBack.positionFdb;

        if (positionSet - pidPosition.fdb > PI)
        {
            pidPosition.ref -= PIX2;
        }
        else if (positionSet - pidPosition.fdb < -PI)
        {
            pidPosition.ref += PIX2;
        }

        pidPosition.UpdateResult();

        speedSet = pidPosition.result;
    }
    pidSpeed.ref = speedSet;
    pidSpeed.fdb = _spdFdb;

    pidSpeed.UpdateResult();

    currentSet = pidSpeed.result;
    
    currentSet = Math::FloatConstrain(currentSet, -2000.0f, 2000.0f);

    SendControlCommand(currentSet + 2047.0f);
}

void HT04::SendControlCommand(float current)
{
    CAN_TypeDef *_can = can;
    uint32_t _id = canId;

    uint16_t _current = (uint16_t)(current);

    CanTxMsg _txMessage;
    _txMessage.Header.StdId = _id;
    _txMessage.Header.IDE = CAN_ID_STD;
    _txMessage.Header.RTR = CAN_RTR_DATA;
    _txMessage.Header.DLC = 0x08;
    _txMessage.Data[0] = 0;
    _txMessage.Data[1] = 0;
    _txMessage.Data[2] = 0;
    _txMessage.Data[3] = 0;
    _txMessage.Data[4] = 0;
    _txMessage.Data[5] = 0;
    _txMessage.Data[6] = _current >> 8;
    _txMessage.Data[7] = _current & 0xFF;

    CAN_HandleTypeDef *_canHandle = (_can == CAN1) ? &hcan1 : &hcan2;
    HAL_CAN_AddTxMessage(_canHandle, &_txMessage.Header, _txMessage.Data, NULL);
}

void HT04::EnterMotorMode()
{
    CAN_TypeDef *_can = can;
    uint32_t _id = canId;

    CanTxMsg _txMessage;
    _txMessage.Header.StdId = _id;
    _txMessage.Header.IDE = CAN_ID_STD;
    _txMessage.Header.RTR = CAN_RTR_DATA;
    _txMessage.Header.DLC = 0x08;
    _txMessage.Data[0] = 0xFF;
    _txMessage.Data[1] = 0xFF;
    _txMessage.Data[2] = 0xFF;
    _txMessage.Data[3] = 0xFF;
    _txMessage.Data[4] = 0xFF;
    _txMessage.Data[5] = 0xFF;
    _txMessage.Data[6] = 0xFF;
    _txMessage.Data[7] = 0xFC;

    CAN_HandleTypeDef *_canHandle = (_can == CAN1) ? &hcan1 : &hcan2;
    HAL_CAN_AddTxMessage(_canHandle, &_txMessage.Header, _txMessage.Data, NULL);
}

void HT04::ExitMotorMode()
{
    CAN_TypeDef *_can = can;
    uint32_t _id = canId;

    CanTxMsg _txMessage;
    _txMessage.Header.StdId = _id;
    _txMessage.Header.IDE = CAN_ID_STD;
    _txMessage.Header.RTR = CAN_RTR_DATA;
    _txMessage.Header.DLC = 0x08;
    _txMessage.Data[0] = 0xFF;
    _txMessage.Data[1] = 0xFF;
    _txMessage.Data[2] = 0xFF;
    _txMessage.Data[3] = 0xFF;
    _txMessage.Data[4] = 0xFF;
    _txMessage.Data[5] = 0xFF;
    _txMessage.Data[6] = 0xFF;
    _txMessage.Data[7] = 0xFD;

    CAN_HandleTypeDef *_canHandle = (_can == CAN1) ? &hcan1 : &hcan2;
    HAL_CAN_AddTxMessage(_canHandle, &_txMessage.Header, _txMessage.Data, NULL);
}

void HT04::SetZeroPos()
{
    CAN_TypeDef *_can = can;
    uint32_t _id = canId;

    CanTxMsg _txMessage;
    _txMessage.Header.StdId = _id;
    _txMessage.Header.IDE = CAN_ID_STD;
    _txMessage.Header.RTR = CAN_RTR_DATA;
    _txMessage.Header.DLC = 0x08;
    _txMessage.Data[0] = 0xFF;
    _txMessage.Data[1] = 0xFF;
    _txMessage.Data[2] = 0xFF;
    _txMessage.Data[3] = 0xFF;
    _txMessage.Data[4] = 0xFF;
    _txMessage.Data[5] = 0xFF;
    _txMessage.Data[6] = 0xFF;
    _txMessage.Data[7] = 0xFE;

    CAN_HandleTypeDef *_canHandle = (_can == CAN1) ? &hcan1 : &hcan2;
    HAL_CAN_AddTxMessage(_canHandle, &_txMessage.Header, _txMessage.Data, NULL);
}

void HT04::Motivate()
{
    controlMode = CUR_MODE;
    lastRelax = true;
}
