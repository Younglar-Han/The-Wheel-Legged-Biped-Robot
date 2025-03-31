#include "CanMsgDispatcher.hpp"

void CanMsgDispatcher::RegisterHandler(CAN_TypeDef* _canX, int32_t _canId, CanMsgHandler* _handler)
{
    int _canIndex = 0;
    if(_canX == CAN2)
    {
        _canIndex = 1;
    }

    m_CanIdHandlerTable[_canIndex].Insert(_canId, _handler);
}

void CanMsgDispatcher::Update()
{
    CanManager& _canManager = *CanManager::Instance();
    for(int i = 0; i < 2; ++i)
    {
        while(!_canManager.MsgQueue(i)->IsEmpty())
        {
            CanRxMsg& _rxMsg = *_canManager.MsgQueue(i)->Dequeue();
            uint32_t _Id = _rxMsg.Header.StdId == 0 ? _rxMsg.Data[0] : _rxMsg.Header.StdId;
            CanMsgHandler** _handler = m_CanIdHandlerTable[i].Search(_Id);

            if(_handler != nullptr)
            {
                (*_handler)->HandleNewCanRxMsg(&_rxMsg);
            }
        }
    }
}
