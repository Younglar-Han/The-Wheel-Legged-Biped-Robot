#include "LegStateRemote.hpp"

void LegStateRemote::Init(LegController* pOwner)
{
    ;
}

void LegStateRemote::Enter(LegController* pOwner)
{
    ;
}

void LegStateRemote::Execute(LegController* pOwner)
{
    Dr16* pDr16 = Dr16::Instance();
    if(pDr16->QuerySwState(Dr16::RC_SW_R, Dr16::RC_SW_M2U))
    {
        pOwner->SetHeight(LegController::LEG_MID_HEIGHT);
    }
    
    if(pDr16->QuerySwState(Dr16::RC_SW_R, Dr16::RC_SW_M2D))
    {
        pOwner->SetHeight(LegController::LEG_MIN_HEIGHT + 0.05f);
    }
    pOwner->SetRoll(0.0f);
}

void LegStateRemote::Exit(LegController* pOwner)
{
    ;
}
