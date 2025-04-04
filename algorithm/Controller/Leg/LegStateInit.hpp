#ifndef LEGSTATEINIT_HPP
#define LEGSTATEINIT_HPP

#include "StateMachine.hpp"
#include "LegController.hpp"

class LegStateInit : public State<LegController>
{
private:
    HT04* m_LegMotor[4];
    uint32_t m_initTick;

public:
    virtual void Init(LegController* pOwner);
    virtual void Enter(LegController* pOwner);
    virtual void Execute(LegController* pOwner);
    virtual void Exit(LegController* pOwner);

    static LegStateInit* Instance()
    {
        static LegStateInit instance;
        return &instance;
    }
};

#endif
