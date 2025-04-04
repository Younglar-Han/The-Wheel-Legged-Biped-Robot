#include "WheelController.hpp"
#include "WheelStateRelax.hpp"
#include "WheelStateBalance.hpp"
#include "WheelStateRemote.hpp"
#include "WheelStateInit.hpp"
#include "arm_math.h"

#include "Math.hpp"
#include "Buzzer.hpp"

const float WheelController::LK9025StandardSpeedKp = 200.0f; // origin 200f   0203 14:40
const float WheelController::LK9025StandardSpeedKi = 1.0f;
const float WheelController::LK9025StandardSpeedKd = 50.0f; // origin 50f    0204 11:17
const float WheelController::LK9025StandardSpeedMaxout = 2000.0f;
const float WheelController::LK9025StandardSpeedIMaxout = 0.0f;
const float WheelController::LK9025MaxSpeed = 60.0f;

WheelController *pWheelDebugHandler;

WheelController::WheelController() : ControllerEntity(ECT_WheelController),
                                     m_IMU(AHRSEstimator::Instance()),
                                     torque_leg_l(0.0f),
                                     torque_leg_r(0.0f),
                                     state_vmc(false),
                                     state_relax(true),
                                     s_dot_acc(0.0f),
                                     wheelFsm(this)
{
}

void WheelController::Init()
{
    SetDefaultTicksToUpdate(2);
    pWheelDebugHandler = this;

    // 瓴控电机注册 以车前进方向（目前为电池安装方向）为正，左边注册为2，右边为1（瓴控没有0号id），即0x141和0x142
    for (int i = 0; i < WMT_LENGTH; ++i)
    {
        m_WheelMotor[i].RegistMotor(CAN2, 0x142 - i);
        m_WheelMotor[i].controlMode = Motor::CUR_MODE;
    }

    m_RampFilter_s.SetMargin(0.0024f); // 为了防止速度突变，设置了一个斜坡滤波器
    m_RampFilter_yaw.SetMargin(0.008f);
    m_s_dot_filter.SetTau(0.05f);
    m_s_dot_filter.SetUpdatePeriod(2);
    m_YawDotFilter.SetTau(0.01f);
    m_YawDotFilter.SetUpdatePeriod(2);

    for (int i = 0; i < 3;i++)
    {
        m_gyro_filter[i].SetTau(1.0f);
        m_gyro_filter[i].SetUpdatePeriod(2);
    }

    m_pLegController = (LegController *)this->GetOwner()->GetEntity(ECT_LegController);

    chassisFollowPid.SetParams(2.0f, 0.0f, 25.0f, 50.0f, 30.0f);
    sFollowPid.SetParams(10.0f, 0.0f, 100.0f, 3.0f, 0.0f);

    for (int i = 0; i < WMT_LENGTH; i++)
    {
        m_WheelMotor[i].pidSpeed.SetParams(LK9025StandardSpeedKp, LK9025StandardSpeedKi, LK9025StandardSpeedKd, LK9025StandardSpeedMaxout, LK9025StandardSpeedIMaxout);
    }

    wheelFsm.Init();

    yaw = m_IMU->GetYaw();
    ResetCurrentState(); // 重置当前状态
}

void WheelController::Update()
{
    wheelFsm.HandleInput();
    wheelFsm.Update();

    if ((m_WheelMotor[0].sensorFeedBack.IsTimeout() || m_WheelMotor[1].sensorFeedBack.IsTimeout()))
    {
        // Buzzer::Instance()->On();
        buzzer_tick = Time::GetTick();
    }
    else
    {
        if (Time::GetTick() - buzzer_tick > 1000)
        {
            Buzzer::Instance()->Off();
        }
    }
    wheel1_update_period = Time::GetTick() - m_WheelMotor[0].sensorFeedBack.GetLastUpdateTick();
    wheel2_update_period = Time::GetTick() - m_WheelMotor[1].sensorFeedBack.GetLastUpdateTick();

    CalcLQR();
    UpdateStateVariables(); // 更新状态变量
    CalcWheelTorque();      // 计算轮子力矩
    PredictOutput();        // 防打滑思路，通过预测模型计算出轮子的速度，然后根据速度差计算出轮子的力矩，然后通过力矩控制轮子的速度

    // if (m_ctrlMsg2.IsForceRelax())
    // {
    //     WheelSpdMode();
    //     SetWheelSpd(0.0f);
    //     return;
    // }
    // else
    // {
    //     MotivateWheel();
    // }

    if (!state_relax)
    {
        if (!state_spd)
        {
            SetWheelTorque(); // 设置轮子力矩
        }
        else
        {
            WheelSpdMode(); // 设置轮子速度模式
        }
    }
    else
    {
        RelaxWheel();        // 轮子松开
        ResetCurrentState(); // 重置当前状态
    }
}

void WheelController::CalcLQR()
{
    float l_l = m_pLegController->GetLengthL();
    float l_r = m_pLegController->GetLengthR();
    float l_l2 = l_l * l_l;
    float l_r2 = l_r * l_r;
    float l_lr = l_l * l_r;

    // 通过双边腿长拟合LQR的K矩阵及A、B矩阵
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            LQR_K[i][j] = constSet[0][i][j] + constSet[1][i][j] * l_l + constSet[2][i][j] * l_l2 + constSet[3][i][j] * l_r + constSet[4][i][j] * l_r2 + constSet[5][i][j] * l_lr;
            A_matrix[i][j] = AconstSet[0][i][j] + AconstSet[1][i][j] * l_l + AconstSet[2][i][j] * l_l2 + AconstSet[3][i][j] * l_r + AconstSet[4][i][j] * l_r2 + AconstSet[5][i][j] * l_lr;
            if (j < 4)
                B_matrix[i][j] = BconstSet[0][i][j] + BconstSet[1][i][j] * l_l + BconstSet[2][i][j] * l_l2 + BconstSet[3][i][j] * l_r + BconstSet[4][i][j] * l_r2 + BconstSet[5][i][j] * l_lr;
        }
    }

    // 离地时对应的K矩阵置零
    if (m_pLegController->IsLeftLeaveGround())
    {
        for (int i = 0; i < 10; i++)
        {
            LQR_K[0][i] = 0.0f;
            if (i != 4 && i != 5)
                LQR_K[2][i] = 0.0f;
        }
        LQR_K[3][4] = 0.0f;
        LQR_K[3][5] = 0.0f;
    }
    if (m_pLegController->IsRightLeaveGround())
    {
        for (int i = 0; i < 10; i++)
        {
            LQR_K[1][i] = 0.0f;
            if (i != 6 && i != 7)
                LQR_K[3][i] = 0.0f;
        }
        LQR_K[2][6] = 0.0f;
        LQR_K[2][7] = 0.0f;
    }
}

void WheelController::UpdateStateVariables()
{
    bool isLeftLeaveGround = m_pLegController->IsLeftLeaveGround() && state_vmc;
    bool isRightLeaveGround = m_pLegController->IsRightLeaveGround() && state_vmc;

    theta_l = m_pLegController->GetThetaL();        // left leg angle fdb
    theta_dot_l = m_pLegController->GetThetaLDot(); // left leg angle spd fdb

    theta_r = m_pLegController->GetThetaR();        // right leg angle fdb
    theta_dot_r = m_pLegController->GetThetaRDot(); // right leg angle spd fdb

    theta_b = m_IMU->GetPitch();        // body angle fdb
    theta_dot_b = m_IMU->tmp_m_gyro[0]; // body angle spd fdb

    length_l = m_pLegController->GetLengthL();
    length_r = m_pLegController->GetLengthR();

    float theta_joint[4];
    float min_err = 1.0f;
    for (int i = 0; i < 4;i++)
    {
        theta_joint[i] = m_pLegController->GetLegMotor(LegController::LegMotorType(i))->pFeedback->positionFdb;
        float err = fabs(theta_joint[i]) > fabs(theta_joint[i] - 1.7f) ? fabs(theta_joint[i] - 1.7f) : fabs(theta_joint[i]);
        min_err = err < min_err ? err : min_err;
    }
    theta_coe = Math::FloatConstrain(min_err * 4.0f, 0.0f, 1.0f);
    yaw_coe = Math::FloatConstrain((0.43f - fabs(delta_yaw)) * 3.0f, 0.0f, 1.0f);
    if (fabs(real_target_s_dot) < 0.05f)
        yaw_coe = 1.0f;
    s_coe = theta_coe * yaw_coe;

    last_s_dot = s_dot;
    s_dot_l = m_WheelMotor[0].sensorFeedBack.speedFdb * 0.1f;  // wheel radius * (left wheel spd fdb)
    s_dot_r = -m_WheelMotor[1].sensorFeedBack.speedFdb * 0.1f; // wheel radius * (right wheel spd fdb)
    s_dot = 0.5f * (s_dot_l + s_dot_r);                        // 1/2 * (left wheel spd + right wheel spd)

    calc_yaw_dot = (- s_dot_l + s_dot_r) / 0.502f - length_l * cos(theta_l) * theta_dot_l / 0.502f + length_r * cos(theta_r) * theta_dot_r / 0.502f;
    m_YawDotFilter.SetInput(calc_yaw_dot);
    m_YawDotFilter.Update();
    calc_yaw_dot = m_YawDotFilter.GetResult();

    float estimate_coe = Math::FloatConstrain(2.0f - 2.0f * fabs(calc_yaw_dot - m_IMU->tmp_m_gyro[2]), 0.0f, 1.0f);

    SpeedEstimate();

    m_s_dot_filter.SetInput(s_dot);
    m_s_dot_filter.Update();
    filtered_s_dot = m_s_dot_filter.GetResult();
    original_s_dot = s_dot;
    vel_est = vel - body_wheel_spd;
    s_dot = filtered_s_dot * estimate_coe + vel_est * (1.0f - estimate_coe);
	// s_dot = filtered_s_dot;

    m_pLegController->SetSDot(s_dot);

    float margin = fabs(s_dot) > 0.7f ? 0.006f : (0.05f * fabs(s_dot) * fabs(s_dot) - 0.07f * fabs(s_dot) + 0.025f);
    m_RampFilter_yaw.SetMargin(margin);

    if (fabs(real_target_s_dot) > 0.01f)
    {
        move_flag = true;
    }

    target_s_dot = SdotFilter(real_target_s_dot); // wheel spd set

    if (move_flag && fabs(target_s_dot) < 0.01f)
    {
        target_s = s;
        if (fabs(s_dot) < 0.03f)
            move_flag = false;
    }

    if (fabs(s - target_s) > 2.0f)
    {
        target_s = s;
        m_pLegController->SetLoseJudge(true);
    }
    else
    {
        m_pLegController->SetLoseJudge(false);
    }

    last_target_s_dot = real_target_s_dot;

    if (isLeftLeaveGround)
        s_dot = s_dot_r;
    if (isRightLeaveGround)
        s_dot = s_dot_l;
    if (isLeftLeaveGround && isRightLeaveGround)
        ResetCurrentState();

    s += s_dot * 0.002f;               // wheel fdb
    target_s += target_s_dot * 0.002f; // wheel fdb

    delta_yaw = YawFilter(real_delta_yaw);
    yaw = m_IMU->GetYaw();
    target_yaw = yaw - delta_yaw;

    chassisFollowPid.ref = 0.0f;
    chassisFollowPid.fdb = delta_yaw;
    chassisFollowPid.UpdateResult();
    yaw_dot = m_IMU->tmp_m_gyro[2];            // imu yaw spd fdb
    target_yaw_dot += chassisFollowPid.result; // imu yaw spd set

    float s_dot_coe = Math::FloatConstrain(1.05f - 0.3f * fabs(s_dot), 0.5f, 1.0f);
    target_yaw_dot *= s_dot_coe;

    delta_yaw *= theta_coe;
    target_yaw_dot *= theta_coe;

    float compensation_coe = Math::FloatConstrain(yaw_dot - 0.8f, 0.0f, 1.0f);

    float compensation = (length_l + length_r + 0.5f) * 10.0f * (s_dot_l * s_dot_l - s_dot_r * s_dot_r);
    compensation *= compensation_coe;
    float compensation_l = compensation > 0.0f ? compensation : 0.0f;
    float compensation_r = compensation < 0.0f ? -compensation : 0.0f;
    m_pLegController->SetCompensation(compensation_l, compensation_r);
}

void WheelController::SpeedEstimate()
{
    body_spd = s_dot + (length_l * cos(theta_l) * theta_dot_l + length_r * cos(theta_r) * theta_dot_r) / 2.0f;
    body_wheel_spd = body_spd - s_dot;
    for (int i = 0; i < 3;i++)
    {
        body_acc[i] = m_IMU->motion_acc[i] * 9.81f;
        origin_acc[i] = m_IMU->get_m_a(i);
        last_gyro[i] = current_gyro[i];
        current_gyro[i] = m_IMU->tmp_m_gyro[i];
        m_gyro_filter[i].SetInput(current_gyro[i] - last_gyro[i]);
        m_gyro_filter[i].Update();
        delta_gyro[i] = m_gyro_filter[i].GetResult();
    }
    extra_acc_x_yaw = - current_gyro[2] * current_gyro[2] * CENTER_IMU_L + delta_gyro[2] * CENTER_IMU_W;
    extra_acc_x_pitch = - current_gyro[0] * current_gyro[0] * CENTER_IMU_L - delta_gyro[0] * CENTER_IMU_H;
    extra_acc_z_pitch = - current_gyro[0] * current_gyro[0] * CENTER_IMU_H + delta_gyro[0] * CENTER_IMU_L;
    acc_x = origin_acc[0] - extra_acc_x_yaw - extra_acc_x_pitch;
    acc_z = origin_acc[2] - extra_acc_z_pitch;
    last_acc = actual_acc;
    actual_acc = acc_x * cos(m_IMU->GetPitch()) + acc_z * sin(m_IMU->GetPitch());
    u = (last_acc + actual_acc) / 2.0f;
    vel_prior = vel + 0.002f * u;
    vel_predict = vel_prior;
    vel_cov = vel_cov + 0.002f * VEL_PROCESS_NOISE;
    vel_measure = body_spd;
    k = vel_cov / (vel_cov + VEL_MEASURE_NOISE);
    vel = vel_prior + k * (vel_measure - vel_prior);
    vel_cov = (1 - k) * vel_cov;
    vel_cov = Math::FloatConstrain(vel_cov, 0.01f, 100.0f);
}

void WheelController::CalcWheelTorque()
{
    float real_delta_pitch = theta_b - target_theta_b;
    float delta_pitch = Math::FloatConstrain(real_delta_pitch, -0.1f, 0.1f);
    float state[10] = {s - target_s, s_dot - target_s_dot, delta_yaw, yaw_dot - target_yaw_dot, theta_l, theta_dot_l, theta_r, theta_dot_r, real_delta_pitch, theta_dot_b};

    torque_l = 0.0f;
    torque_r = 0.0f;
    torque_leg_l = 0.0f;
    torque_leg_r = 0.0f;

    float pitch_coe = Math::FloatConstrain(4.0f - 20.0f * fabs(real_delta_pitch), 0.0f, 1.0f);

    // LQR控制
    for (int i = 0; i < 10; i++)
    {
        torque_l += LQR_K[0][i] * state[i];
        torque_r += LQR_K[1][i] * state[i];
        if (i == 8)
        {
            torque_leg_l += LQR_K[2][i] * delta_pitch * pitch_coe;
            torque_leg_r += LQR_K[3][i] * delta_pitch * pitch_coe;
        }
        else
        {
            torque_leg_l += LQR_K[2][i] * state[i];
            torque_leg_r += LQR_K[3][i] * state[i];
        }
    }

    m_pLegController->SetTorque(-torque_leg_l, -torque_leg_r);
}

void WheelController::PredictOutput()
{
    pred_s = s;
    pred_s_dot = s_dot;
    pred_yaw = yaw;
    pred_yaw_dot = yaw_dot;

    // 预测模型——x(k+1) += Ax(k) + Bu(k)
    // 因为只用到了yaw_dot和s_dot，所以只用了A和B的前四行
    float state[10] = {s, s_dot, yaw, yaw_dot, theta_l, theta_dot_l, theta_r, theta_dot_r, theta_b, theta_dot_b};

    for (int i = 0; i < 10; i++)
    {
        pred_s += A_matrix[0][i] * 0.002f * state[i];
        pred_s_dot += A_matrix[1][i] * 0.002f * state[i];
        pred_yaw += A_matrix[2][i] * 0.002f * state[i];
        pred_yaw_dot += A_matrix[3][i] * 0.002f * state[i];
    }

    float input[4] = {torque_l, torque_r, torque_leg_l, torque_leg_r};

    for (int i = 0; i < 4; i++)
    {
        pred_s += B_matrix[0][i] * 0.002f * input[i];
        pred_s_dot += B_matrix[1][i] * 0.002f * input[i];
        pred_yaw += B_matrix[2][i] * 0.002f * input[i];
        pred_yaw_dot += B_matrix[3][i] * 0.002f * input[i];
    }

    pred_s_dot_l = (pred_s_dot - 0.251f * pred_yaw_dot);
    pred_s_dot_r = (pred_s_dot + 0.251f * pred_yaw_dot);

    // 调整力矩输出为预测与实际之差乘以一个系数
    T_l_adapt = (pred_s_dot_l - s_dot_l) * 1.0f;
    T_r_adapt = (pred_s_dot_r - s_dot_r) * 1.0f;
}

void WheelController::SetWheelTorque()
{
    // m_WheelMotor[WMT_Left].torqueSet = -torque_l + T_l_adapt; // u = - K (x - x_d) + T_adapt (from prediction model)
    // m_WheelMotor[WMT_Right].torqueSet = torque_r - T_r_adapt;

    m_WheelMotor[WMT_Left].torqueSet = 0.0f; // u = - K (x - x_d) + T_adapt (from prediction model)
    m_WheelMotor[WMT_Right].torqueSet = 0.0f;
}

float WheelController::SdotFilter(float _s_dot)
{
    // 减速和加速时的速度变化率不同
    if ((fabs(_s_dot) < fabs(m_RampFilter_s.GetResult())) && (_s_dot * m_RampFilter_s.GetResult() >= 0.0f))
    {
        m_RampFilter_s.SetMargin(0.05f); // 减速变化率
    }
    else
    {
        m_RampFilter_s.SetMargin(0.02f); // 加速变化率
    }
    m_RampFilter_s.SetInput(_s_dot);
    m_RampFilter_s.Update();
    // return m_RampFilter_s.GetResult();
    float delta_s_dot = m_RampFilter_s.GetResult() - s_dot;
    float delta_s = target_s - s;
    target_s = s + delta_s * s_coe;
    return s_dot + delta_s_dot * s_coe;
}

float WheelController::YawFilter(float _yaw)
{
    m_RampFilter_yaw.SetMargin(m_RampFilter_yaw.GetMargin());
    m_RampFilter_yaw.SetInput(_yaw);
    m_RampFilter_yaw.Update();
    return m_RampFilter_yaw.GetResult();
}

void WheelController::ResetCurrentState()
{
    s_dot = 0.0;
    target_s_dot = 0.0f;
    m_RampFilter_s.SetResult(0.0f);
    m_RampFilter_yaw.SetResult(0.0f);
    s = 0.0f;
    target_s = 0.0f;
    target_yaw = yaw;
    chassisFollowPid.Clear();
    sFollowPid.Clear();
}

void WheelController::RelaxWheel()
{
    for (int i = 0; i < WMT_LENGTH; ++i)
    {
        m_WheelMotor[i].controlMode = Motor::RELAX_MODE;
    }
}

void WheelController::MotivateWheel()
{
    for (int i = 0; i < WMT_LENGTH; ++i)
    {
        m_WheelMotor[i].controlMode = Motor::CUR_MODE;
    }
}

void WheelController::WheelSpdMode()
{
    for (int i = 0; i < WMT_LENGTH; ++i)
    {
        m_WheelMotor[i].controlMode = Motor::SPD_MODE;
    }
}

void WheelController::SetWheelSpd(float _s_dot)
{
    m_WheelMotor[WMT_Left].speedSet = -_s_dot;
    m_WheelMotor[WMT_Right].speedSet = _s_dot;
}

void WheelFsm::Init()
{
    WheelStateBalance::Instance()->Init(m_pOwner);
    WheelStateRelax::Instance()->Init(m_pOwner);
    WheelStateRemote::Instance()->Init(m_pOwner);
    WheelStateInit::Instance()->Init(m_pOwner);
    SetCurrentState(WheelStateBalance::Instance());
}

void WheelFsm::HandleInput()
{
    Dr16 *pDr16 = Dr16::Instance();

    // if (pDr16->QuerySwState(Dr16::RC_SW_L, Dr16::RC_SW_DOWN))
    // {
    //     ChangeState(WheelStateRelax::Instance());
    //     return;
    // }

    // if (pDr16->QuerySwState(Dr16::RC_SW_L, Dr16::RC_SW_D2M))
    // {
    //     ChangeState(WheelStateInit::Instance());
    //     return;
    // }

    ChangeState(WheelStateRelax::Instance());
}
