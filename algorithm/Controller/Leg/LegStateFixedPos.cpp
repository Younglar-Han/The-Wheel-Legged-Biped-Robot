#include "LegStateFixedPos.hpp"

void LegStateFixedPos::Init(LegController* pOwner)
{
    ;
}

void LegStateFixedPos::Enter(LegController* pOwner)
{
    pOwner->LegPosMode();
    pOwner->SetPos(0.0f);
}

void LegStateFixedPos::Execute(LegController* pOwner)
{
    pOwner->LegPosMode();
    pOwner->SetPos(0.0f);
}

void LegStateFixedPos::Exit(LegController* pOwner)
{
    ;
}
