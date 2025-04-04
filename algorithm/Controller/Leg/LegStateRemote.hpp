#ifndef LEGSTATEREMOTE_HPP
#define LEGSTATEREMOTE_HPP

#include "StateMachine.hpp"
#include "LegController.hpp"

class LegStateRemote : public State<LegController>
{
public:
    virtual void Init(LegController* pOwner);
    virtual void Enter(LegController* pOwner);
    virtual void Execute(LegController* pOwner);
    virtual void Exit(LegController* pOwner);

    static LegStateRemote* Instance()
    {
        static LegStateRemote instance;
        return &instance;
    }
};

#endif
