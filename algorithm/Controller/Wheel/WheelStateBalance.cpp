#include "WheelStateBalance.hpp"

void WheelStateBalance::Init(WheelController *pOwner)
{
    m_pBoardManager = BoardPacketManager::Instance();
}

void WheelStateBalance::Enter(WheelController *pOwner)
{
    pOwner->MotivateWheel();
    pOwner->SetSdot(0.0f);
    pOwner->SetDeltaYaw(0.0f);
    pOwner->SetYawDot(0.0f);
    pOwner->SetVmcMode(false);
    pOwner->SetRelaxMode(true);
}

void WheelStateBalance::Execute(WheelController *pOwner)
{
    pOwner->MotivateWheel();
    
    I6X* pI6X = I6X::Instance();

    pOwner->SetSdot(pI6X->GetLVAxis());
    pOwner->SetDeltaYaw(0.0f);
    pOwner->SetYawDot(pI6X->GetRHAxis());

    pOwner->SetVmcMode(true);
    pOwner->SetRelaxMode(false);

    float yaw = 0.0f;
    float pitch = 0.0f;
    float target_pitch = acos(Math::FloatConstrain(cos(pitch) / (sqrt(sin(yaw) * sin(yaw) * cos(pitch) * cos(pitch) + cos(yaw) * cos(yaw))), -1.0f, 1.0f));
    target_pitch = ((fabs(yaw) < PI / 2 && pitch < 0) || (fabs(yaw) > PI / 2 && pitch > 0)) ? target_pitch : -target_pitch;
    // target_pitch = 0.0f;
    pOwner->SetPitch(0.0f);
    pOwner->SetCosPitch(cos(pitch) / (sqrt(sin(yaw) * sin(yaw) * cos(pitch) * cos(pitch) + cos(yaw) * cos(yaw))));
}

void WheelStateBalance::Exit(WheelController *pOwner)
{
    ;
}
