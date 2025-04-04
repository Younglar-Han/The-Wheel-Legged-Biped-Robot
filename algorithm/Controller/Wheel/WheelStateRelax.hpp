#ifndef WHEELSTATERELAX_HPP
#define WHEELSTATERELAX_HPP

#include "StateMachine.hpp"
#include "WheelController.hpp"

class WheelStateRelax : public State<WheelController>
{
public:
    virtual void Init(WheelController* pOwner);
    virtual void Enter(WheelController* pOwner);
    virtual void Execute(WheelController* pOwner);
    virtual void Exit(WheelController* pOwner);

    static WheelStateRelax* Instance()
    {
        static WheelStateRelax instance;
        return &instance;
    }
};

#endif
