#include "WheelStateRelax.hpp"

void WheelStateRelax::Init(WheelController* pOwner)
{
    ;
}

void WheelStateRelax::Enter(WheelController* pOwner)
{
    pOwner->RelaxWheel();
}

void WheelStateRelax::Execute(WheelController* pOwner)
{
    pOwner->RelaxWheel();
}

void WheelStateRelax::Exit(WheelController* pOwner)
{
    ;
}
