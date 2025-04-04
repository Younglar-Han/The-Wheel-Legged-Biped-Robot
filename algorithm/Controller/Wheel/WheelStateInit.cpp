#include "WheelStateInit.hpp"

void WheelStateInit::Init(WheelController* pOwner)
{
    m_pBoardManager = BoardPacketManager::Instance();
}

void WheelStateInit::Enter(WheelController* pOwner)
{
    pOwner->SetRelaxMode(false);
    pOwner->SetSpdMode(true);
}

void WheelStateInit::Execute(WheelController* pOwner)
{
    pOwner->WheelSpdMode();
    pOwner->SetWheelSpd(0.0f);
}

void WheelStateInit::Exit(WheelController* pOwner)
{
    pOwner->SetSpdMode(false);
	pOwner->ResetCurrentState();
}
