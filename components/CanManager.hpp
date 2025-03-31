#ifndef CANMANAGER_H
#define CANMANAGER_H

#include "main.h"
#include "can.h"
#include <string.h>

#define CAN_QUEUE_LENGTH 32
#define CAN1_NVIC 4
#define CAN2_NVIC 4

struct CanRxMsg
{
    CAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
};

struct CanTxMsg
{
    CAN_TxHeaderTypeDef Header;
    uint8_t Data[8];
};

class CanMsgQueue
{
private:
    CanRxMsg m_Data[CAN_QUEUE_LENGTH];
    uint8_t m_Front;
    uint8_t m_Rear;
public:
    CanMsgQueue():m_Front(0),
                  m_Rear(0)
    {}
    
    bool IsEmpty()
    {
        return m_Front == m_Rear;
    }

    // NOT TESTED!!
    uint8_t Count()
    {
        uint8_t _count = m_Rear >= m_Front ? m_Rear - m_Front : CAN_QUEUE_LENGTH - (m_Front - m_Rear);
        return _count;
    }

    void Empty()
    {
        m_Front = 0;
        m_Rear = 0;
    }

    CanRxMsg* Peek()
    {
        return &(m_Data[m_Front]);
    }
    
    void Enqueue(CanRxMsg* _msg)
    {
        memcpy(&m_Data[m_Rear], _msg, sizeof(CanRxMsg));
        m_Rear = (m_Rear + 1) % CAN_QUEUE_LENGTH;
    }

    CanRxMsg* Dequeue()
    {
        CanRxMsg* _msg = &m_Data[m_Front];
        m_Front = (m_Front + 1) % CAN_QUEUE_LENGTH;
        return _msg;
    }
};

struct CanMsgStream
{
    CAN_HandleTypeDef* m_pCan;
    uint32_t m_StreamId;
    uint8_t m_MsgBuffer[256];
    uint8_t m_MsgLen;
    uint8_t m_LenSent;
};


class CanManager
{
private:
    CanManager(){}
    CanMsgQueue m_Queue[2];
    CanMsgStream m_SendBuffer[8];
    uint8_t m_SendFront;
    uint8_t m_SendRear;
    int8_t m_PacketPerTick;
    bool SendQueueEmpty();
    void SendOnePacket();
public:

    static CanManager* Instance()
    {
        static CanManager instance;
        return &instance;
    }

    CanMsgQueue* MsgQueue(uint32_t _index)
    {
        return &m_Queue[_index & 0x0001];
    }

    void Init();
    void Update();
    void CanTransmit(CAN_HandleTypeDef* _can, uint32_t _id, uint8_t* _pData, uint32_t _len);

    int8_t GetPacketPerTick(){ return m_PacketPerTick; }

    void SetPacketPerTick(int8_t _packetPerTick)
    {
        if( _packetPerTick < 1)
        {
            m_PacketPerTick = 1;
            return;
        }

        if( _packetPerTick > 3)
        {
            m_PacketPerTick = 3;
            return;
        }

        m_PacketPerTick = _packetPerTick;
    }
};

#endif
