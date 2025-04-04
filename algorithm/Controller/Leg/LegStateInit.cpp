#include "LegStateInit.hpp"

void LegStateInit::Init(LegController *pOwner)
{
    m_LegMotor[0] = pOwner->GetLegMotor(LegController::LMT_LeftFront);
    m_LegMotor[1] = pOwner->GetLegMotor(LegController::LMT_LeftRear);
    m_LegMotor[2] = pOwner->GetLegMotor(LegController::LMT_RightRear);
    m_LegMotor[3] = pOwner->GetLegMotor(LegController::LMT_RightFront);
}

void LegStateInit::Enter(LegController *pOwner)
{
    pOwner->LegSpdMode();
    m_initTick = Time::GetTick();
}

void LegStateInit::Execute(LegController *pOwner)
{
    pOwner->LegSpdMode();
    pOwner->SetSpd(-6.0f);
    bool legReady = true;
    for (int i = 0; i < 4; i++)
    {
        if (fabs(m_LegMotor[i]->pFeedback->speedFdb) > 0.05f || Time::GetTick() - m_initTick < 500)
        {
            legReady = false;
            break;
        }
    }
    if (legReady)
    {
        pOwner->InitFinished();
    }
}

void LegStateInit::Exit(LegController *pOwner)
{
    ;
}
