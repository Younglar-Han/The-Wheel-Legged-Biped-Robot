#include "LegStateBalance.hpp"
#include "STP23L.hpp"

void LegStateBalance::Init(LegController *pOwner)
{
    m_pBoardManager = BoardPacketManager::Instance();
}

void LegStateBalance::Enter(LegController *pOwner)
{
    pOwner->SetInitLengthPID();
    m_ticks = Time::GetTick();
}

void LegStateBalance::Execute(LegController *pOwner)
{
    if (Time::GetTick() - m_ticks > 500) // 1s
    {
        pOwner->SetStandardLengthPID();
    }

    // float target_height = Math::FloatConstrain(ctrlMsg.GetLegLength(), LegController::LEG_MIN_HEIGHT, LegController::LEG_MAX_HEIGHT); // 可变腿高度时，限制腿高度在合理范围内
    float target_height = LegController::LEG_MID_HEIGHT;
    pOwner->SetHeight(target_height);
    pOwner->CalcF();           // 计算腿部的支持力
    pOwner->CalcMotorTorque(); // 计算腿部电机的扭矩
    pOwner->SetMotorTorque();  // 设置腿部电机的扭矩
    // jump related
    // if (ctrlMsg2.IsJump() && !last_jump && !STP23L::Instance()->IsTimeout())
    // {
    //     pOwner->SetJump(true);
    //     pOwner->SetJumpTime(Time::GetTick());
    // }
    // pOwner->SetCDown(ctrlMsg2.IsJump());
    // last_jump = ctrlMsg2.IsJump();

    float yaw = 0.0f;
    float pitch = 0.0f;
    float target_roll = acos(Math::FloatConstrain(cos(pitch) / (sqrt(cos(yaw) * cos(yaw) * cos(pitch) * cos(pitch) + sin(yaw) * sin(yaw))), -1.0f, 1.0f));
    target_roll = yaw * pitch < 0.0f ? -target_roll : target_roll;
    // target_roll = 0.0f;
    pOwner->SetRoll(target_roll);
    pOwner->SetCosRoll(cos(pitch) / (sqrt(cos(yaw) * cos(yaw) * cos(pitch) * cos(pitch) + sin(yaw) * sin(yaw))));
}

void LegStateBalance::Exit(LegController *pOwner)
{
    ;
}
