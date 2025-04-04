#ifndef LEGSTATEBALANCE_HPP
#define LEGSTATEBALANCE_HPP

#include "StateMachine.hpp"
#include "LegController.hpp"

class LegStateBalance : public State<LegController>
{
private:
    BoardPacketManager* m_pBoardManager;
    uint32_t m_ticks;
    bool last_jump;
    
public:
    virtual void Init(LegController* pOwner);
    virtual void Enter(LegController* pOwner);
    virtual void Execute(LegController* pOwner);
    virtual void Exit(LegController* pOwner);

    static LegStateBalance* Instance()
    {
        static LegStateBalance instance;
        return &instance;
    }
};

#endif
