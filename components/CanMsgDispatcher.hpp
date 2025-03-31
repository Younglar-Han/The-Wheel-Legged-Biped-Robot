#ifndef CANMSGDISPATCHER_H
#define CANMSGDISPATCHER_H

#include "stm32f4xx.h"
#include "CanManager.hpp"
#include "HashTable.hpp"
#include "CanMsgHandler.hpp"

class CanMsgDispatcher
{
private:
    CanMsgDispatcher(){}
    HashTable<CanMsgHandler*> m_CanIdHandlerTable[2];
public:
    static CanMsgDispatcher* Instance()
    {
        static CanMsgDispatcher instance;
        return &instance;
    }

    virtual void Update();
    
    void RegisterHandler(CAN_TypeDef* _canX, int32_t _canId, CanMsgHandler* _handler);
};

#endif
