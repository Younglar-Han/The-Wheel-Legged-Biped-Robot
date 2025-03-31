#ifndef CANMSGHANDLER_H
#define CANMSGHANDLER_H

#include "main.h"
#include "CanManager.hpp"
#include "Time.hpp"
#include <string.h>

class CanMsgHandler
{
protected:
    CanRxMsg lastCanMsg;

    uint32_t m_LastReceivedTick;
    uint32_t m_TimeoutTick;
public:
    virtual void HandleNewCanRxMsg(CanRxMsg* _msg)
    {
        memcpy(&lastCanMsg, _msg, sizeof(CanRxMsg));
        m_LastReceivedTick = Time::GetTick();
    }

    CanMsgHandler() : m_LastReceivedTick(0), m_TimeoutTick(100)
    {
        ;
    }

    uint32_t GetLastReceiveTick() const { return m_LastReceivedTick; }
    uint32_t GetTimeoutTick() const { return m_TimeoutTick; }
    bool IsMsgTimeout() const { return m_LastReceivedTick + m_TimeoutTick < Time::GetTick(); }
};

#endif
