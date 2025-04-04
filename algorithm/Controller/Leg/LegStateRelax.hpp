#ifndef LEGSTATERELAX_HPP
#define LEGSTATERELAX_HPP

#include "StateMachine.hpp"
#include "LegController.hpp"

class LegStateRelax : public State<LegController>
{
public:
    virtual void Init(LegController* pOwner);
    virtual void Enter(LegController* pOwner);
    virtual void Execute(LegController* pOwner);
    virtual void Exit(LegController* pOwner);

    static LegStateRelax* Instance()
    {
        static LegStateRelax instance;
        return &instance;
    }
};

#endif
