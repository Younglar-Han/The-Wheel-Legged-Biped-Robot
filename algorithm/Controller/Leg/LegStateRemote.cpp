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
    I6X* pI6X = I6X::Instance();
    if(pI6X->QuerySwState(I6X::RC_SW_R1, I6X::RC_SW_M2U))
    {
        pOwner->SetHeight(LegController::LEG_MID_HEIGHT);
    }
    
    if(pI6X->QuerySwState(I6X::RC_SW_R1, I6X::RC_SW_M2D))
    {
        pOwner->SetHeight(LegController::LEG_MIN_HEIGHT + 0.05f);
    }
    pOwner->SetRoll(0.0f);
}

void LegStateRemote::Exit(LegController* pOwner)
{
    ;
}
