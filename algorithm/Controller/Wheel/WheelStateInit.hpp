#ifndef WHEELSTATEINIT_HPP
#define WHEELSTATEINIT_HPP

#include "StateMachine.hpp"
#include "WheelController.hpp"

class WheelStateInit : public State<WheelController>
{
private:
    BoardPacketManager* m_pBoardManager;
    
public:
    virtual void Init(WheelController* pOwner);
    virtual void Enter(WheelController* pOwner);
    virtual void Execute(WheelController* pOwner);
    virtual void Exit(WheelController* pOwner);

    static WheelStateInit* Instance()
    {
        static WheelStateInit instance;
        return &instance;
    }
};

#endif
