#include "LegStateRelax.hpp"
#include "Buzzer.hpp"

void LegStateRelax::Init(LegController* pOwner)
{
    ;
}

void LegStateRelax::Enter(LegController* pOwner)
{
    pOwner->SetCur(0.0f);
    pOwner->ResetCurrentState();
}

void LegStateRelax::Execute(LegController* pOwner)
{
    pOwner->SetCur(0.0f);
    pOwner->ResetCurrentState();
}

void LegStateRelax::Exit(LegController* pOwner)
{
    ;
}
