#ifndef LEGSTATEFIXEDPOS_HPP
#define LEGSTATEFIXEDPOS_HPP

#include "StateMachine.hpp"
#include "LegController.hpp"

class LegStateFixedPos : public State<LegController>
{
public:
    virtual void Init(LegController* pOwner);
    virtual void Enter(LegController* pOwner);
    virtual void Execute(LegController* pOwner);
    virtual void Exit(LegController* pOwner);

    static LegStateFixedPos* Instance()
    {
        static LegStateFixedPos instance;
        return &instance;
    }
};

#endif
