#ifndef WHEELSTATEBALANCE_HPP
#define WHEELSTATEBALANCE_HPP

#include "StateMachine.hpp"
#include "WheelController.hpp"
#include "BoardPacket.hpp"
#include "Dr16.hpp"

class WheelStateBalance : public State<WheelController>
{
private:
    BoardPacketManager* m_pBoardManager;
    float last_deltayaw;
    
public:
    virtual void Init(WheelController* pOwner);
    virtual void Enter(WheelController* pOwner);
    virtual void Execute(WheelController* pOwner);
    virtual void Exit(WheelController* pOwner);

    static WheelStateBalance* Instance()
    {
        static WheelStateBalance instance;
        return &instance;
    }
};

#endif
