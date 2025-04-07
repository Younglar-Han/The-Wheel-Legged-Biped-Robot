#include "LKCanMotorCommander.hpp"
#include "Dr16.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void LKCanMotorCommander::Update()
{
    for (int i = 0; i < 2; ++i)
    {
        CAN_HandleTypeDef *_can = (i == 0) ? &hcan1 : &hcan2;
        if (dirtyFlag[i])
        {
            uint32_t _id = 0x280;

            CanTxMsg _txMessage;
            _txMessage.Header.StdId = _id;
            _txMessage.Header.IDE = CAN_ID_STD;
            _txMessage.Header.RTR = CAN_RTR_DATA;
            _txMessage.Header.DLC = 0x08;
            _txMessage.Data[0] = msgBuffer[i][0];
            _txMessage.Data[1] = msgBuffer[i][0] >> 8;
            _txMessage.Data[2] = msgBuffer[i][1];
            _txMessage.Data[3] = msgBuffer[i][1] >> 8;
            _txMessage.Data[4] = msgBuffer[i][2];
            _txMessage.Data[5] = msgBuffer[i][2] >> 8;
            _txMessage.Data[6] = msgBuffer[i][3];
            _txMessage.Data[7] = msgBuffer[i][3] >> 8;

            uint32_t TxMailbox;
            HAL_CAN_AddTxMessage(_can, &_txMessage.Header, _txMessage.Data, &TxMailbox);

            dirtyFlag[i] = false;
        }
    }
}

void LKCanMotorCommander::Add2Buffer(CAN_TypeDef *_can, int32_t _canId, int16_t _current)
{
    int _i = (_can == CAN1) ? 0 : 1;
    int _j = _canId - 0x141;
    msgBuffer[_i][_j] = _current;
    dirtyFlag[_i] = true;
}
