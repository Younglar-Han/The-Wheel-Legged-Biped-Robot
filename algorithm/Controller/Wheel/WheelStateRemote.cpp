#include "WheelStateRemote.hpp"

void WheelStateRemote::Init(WheelController* pOwner)
{
    m_pDr16 = Dr16::Instance()->Instance();
    m_IMU = AHRSEstimator::Instance();
    target_yaw = 0.0f;
}

void WheelStateRemote::Enter(WheelController* pOwner)
{
    pOwner->SetRelaxMode(false);
    target_yaw = m_IMU->GetYaw();
}

void WheelStateRemote::Execute(WheelController* pOwner)
{
    pOwner->SetVmcMode(m_pDr16->QuerySwState(Dr16::RC_SW_L, Dr16::RC_SW_UP));
    pOwner->SetSdot(m_pDr16->GetLVAxis() * 2.5f);
    target_yaw -= m_pDr16->GetRHAxis() * PIX2;
    float yaw = m_IMU->GetYaw();
    float delta_yaw = yaw - target_yaw;
    delta_yaw = Math::LoopFloatConstrain(delta_yaw, -PI, PI);
    pOwner->SetDeltaYaw(delta_yaw);
    pOwner->SetYawDot(-m_pDr16->GetRHAxis() * PIX2);
}

void WheelStateRemote::Exit(WheelController* pOwner)
{
    ;
}
