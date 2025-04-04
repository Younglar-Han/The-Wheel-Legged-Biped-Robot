#include "LegController.hpp"
#include "LegStateRelax.hpp"
#include "LegStateRemote.hpp"
#include "LegStateBalance.hpp"
#include "LegStateFixedPos.hpp"
#include "LegStateInit.hpp"
#include "arm_math.h"
#include "Math.hpp"
#include "Buzzer.hpp"
#include "STP23L.hpp"
// #include "bsp_io.h"


// VMC没必要动HT的速度环和位置环内参
const float LegController::HT04StandardSpeedKp = 300.0f; // origin 200f   0203 14:40
const float LegController::HT04StandardSpeedKi = 1.0f;
const float LegController::HT04StandardSpeedKd = 100.0f; // origin 50f    0204 11:17
const float LegController::HT04StandardSpeedMaxout = 2000.0f;
const float LegController::HT04StandardSpeedIMaxout = 0.0f;
const float LegController::HT04MaxSpeed = 60.0f;

const float LegController::HT04StandardPosKp = 1.0f; // origin 1.0f  0203 14:35
const float LegController::HT04StandardPosKi = 0.2f;
const float LegController::HT04StandardPosKd = 0.0f;      // origin 0.0f  0203 14:45
const float LegController::HT04StandardPosMaxout = 8.0f;  // origin 8.0f  0203 14:28
const float LegController::HT04StandardPosIMaxout = 0.0f; // origin 0.0f  0203 15:01

const float LegController::HEIGHT_KP = 1500.0f; // origin 500f  0204 15:30
const float LegController::HEIGHT_KI = 1.0f;
const float LegController::HEIGHT_KD = 6000.0f;    // origin 6000f  0203 16:30
const float LegController::HEIGHT_MAXOUT = 500.0f; // origin 140f 0204 23:04
const float LegController::HEIGHT_IMAXOUT = 30.0f;
const float LegController::HEIGHT_INIT_KP = 500.0f; // origin 500f  0204 15:30
const float LegController::HEIGHT_INIT_KI = 0.1f;
const float LegController::HEIGHT_INIT_KD = 3000.0f;    // origin 6000f  0203 16:30
const float LegController::HEIGHT_INIT_MAXOUT = 500.0f; // origin 140f 0204 23:04
const float LegController::HEIGHT_INIT_IMAXOUT = 30.0f;
const float LegController::ROLL_KP = 600.0f; // origin 100f   0203 19:53
const float LegController::ROLL_KI = 0.01f;
const float LegController::ROLL_KD = 25.0f;      // origin 150f   0203 14:05
const float LegController::ROLL_MAXOUT = 250.0f; // origin 50f    0203 16:30
const float LegController::ROLL_IMAXOUT = 20.0f; // origin 10.0f  0203 15:27
const float LegController::JUMP_KP = 100.0f;
const float LegController::JUMP_KI = 0.0f;
const float LegController::JUMP_KD = 0.0f;
const float LegController::JUMP_MAXOUT = 100.0f;
const float LegController::JUMP_IMAXOUT = 10.0f;
const float LegController::LEG_LENGTH_1 = 0.27f;
const float LegController::LEG_LENGTH_2 = 0.15f;
const float LegController::LEG_LENGTH_3 = 0.115f;
const float LegController::LEG_MID_HEIGHT = 0.21f; // VMC常态高度        0204 21:30
const float LegController::LEG_MAX_HEIGHT = 0.38f; // VMC最大高度        0204 21:30
const float LegController::LEG_MIN_HEIGHT = 0.12f; // VMC最小高度        0204 21:30
const float LegController::G = 9.8f;

LegController::LegController() : ControllerEntity(ECT_LegController),
                                 m_IMU(AHRSEstimator::Instance()),
                                 F_L(0.0f),
                                 F_R(0.0f),
                                 T_L(0.0f),
                                 T_R(0.0f),
                                 length_l(0.09f),
                                 length_r(0.09f),
                                 theta_l(0.0f),
                                 theta_r(0.0f),
                                 last_theta_l(0.0f),
                                 last_theta_r(0.0f),
                                 jump_finish(0),
                                 jump_state(0),
                                 posSet(0.0f),
                                 relax(false),
                                 jumping(false),
                                 has_inited(false),
                                 power_on(false),
                                 last_power_on(false),
                                 target_height(LEG_MID_HEIGHT),
                                 target_roll(0.0f),
                                 legFsm(this)
{
}

void LegController::Init()
{
    SetDefaultTicksToUpdate(2);

    // 海泰电机注册 以车前进方向（目前为电池安装方向）为正，左前方设置为1，逆时针递增标号，123于CAN1，4于CAN2
    for (int i = 0; i < LMT_LENGTH - 1; ++i)
    {
        m_LegMotor[i].RegistMotor(CAN1, 0x01 + i);
        m_LegMotor[i].controlMode = Motor::CUR_MODE;
        m_LegMotor[i].pidSpeed.SetParams(HT04StandardSpeedKp, HT04StandardSpeedKi, HT04StandardSpeedKd, HT04StandardSpeedMaxout, HT04StandardSpeedIMaxout);
        m_LegMotor[i].pidPosition.SetParams(HT04StandardPosKp, HT04StandardPosKi, HT04StandardPosKd, HT04StandardPosMaxout, HT04StandardPosIMaxout);
    }
    m_LegMotor[3].RegistMotor(CAN2, 0x04);
    m_LegMotor[3].controlMode = Motor::CUR_MODE;
    m_LegMotor[3].pidSpeed.SetParams(HT04StandardSpeedKp, HT04StandardSpeedKi, HT04StandardSpeedKd, HT04StandardSpeedMaxout, HT04StandardSpeedIMaxout);
    m_LegMotor[3].pidPosition.SetParams(HT04StandardPosKp, HT04StandardPosKi, HT04StandardPosKd, HT04StandardPosMaxout, HT04StandardPosIMaxout);

    legLengthPid.SetParams(HEIGHT_KP, HEIGHT_KI, HEIGHT_KD, HEIGHT_MAXOUT, HEIGHT_IMAXOUT);
    leftLegLengthPid.SetParams(HEIGHT_KP, HEIGHT_KI, HEIGHT_KD, HEIGHT_MAXOUT, HEIGHT_IMAXOUT);
    rightLegLengthPid.SetParams(HEIGHT_KP, HEIGHT_KI, HEIGHT_KD, HEIGHT_MAXOUT, HEIGHT_IMAXOUT);
    rollPid.SetParams(ROLL_KP, ROLL_KI, ROLL_KD, ROLL_MAXOUT, ROLL_IMAXOUT);

    thetaDDotL.SetTau(10.0f);
    thetaDDotR.SetTau(10.0f);
    lengthDDotL.SetTau(10.0f);
    lengthDDotR.SetTau(10.0f);

    thetaDDotL.SetUpdatePeriod(2);
    thetaDDotR.SetUpdatePeriod(2);
    lengthDDotL.SetUpdatePeriod(2);
    lengthDDotR.SetUpdatePeriod(2);

    thetaDotL.SetTau(0.01f);
    thetaDotR.SetTau(0.01f);
    lengthDotL.SetTau(0.01f);
    lengthDotR.SetTau(0.01f);

    thetaDotL.SetUpdatePeriod(2);
    thetaDotR.SetUpdatePeriod(2);
    lengthDotL.SetUpdatePeriod(2);
    lengthDotR.SetUpdatePeriod(2);

    lengthDFilter.SetTau(0.01f);
    lengthDFilter.SetUpdatePeriod(2);

    distanceFilter.SetTau(1.0f);
    distanceFilter.SetUpdatePeriod(2);

    // bsp_io_init();

    legFsm.Init();
}

void LegController::Update()
{
    // power_on = m_JudgeSystem->GameRobotStatus.power_management_chassis_output;
    // if (power_on)
    // {
    //     bsp_io_reset();
    // }
    // if (!power_on)
    // {
    //     bsp_io_set();
    // }
    // if ((power_on && !last_power_on && Time::GetTick() > 8000) || m_pChassisCtrlMsg2.IsForceRelax())
    // {
    //     NVIC_SystemReset();
    // }
    // last_power_on = power_on;
    // 初始化，前1.5s重复发送海泰电机开始控制指令，后0.5s重置电机零点
    if (Time::GetTick() < 1500)
    {
        MotivateLeg();
        return;
    }

    for (int i = 0; i < LMT_LENGTH; ++i)
    {
        spdfdb[i] = m_LegMotor[i].sensorFeedBack.speedFdb;
    }

    // if (m_LegMotor[0].sensorFeedBack.IsTimeout() || m_LegMotor[1].sensorFeedBack.IsTimeout() || m_LegMotor[2].sensorFeedBack.IsTimeout() || m_LegMotor[3].sensorFeedBack.IsTimeout())
    // {
    //     Buzzer::Instance()->On();
    //     buzzer_tick = Time::GetTick();
    // }
    // else
    // {
    //     if (Time::GetTick() - buzzer_tick > 500)
    //         Buzzer::Instance()->Off();
    // }

    LegCurMode();           // 腿部电机控制模式
    UpdateStateVariables(); // 更新状态变量
    CalcJacob();            // 计算Jacob矩阵
    JudgeLoseControl();     // 判断是否失控

    if (Time::GetTick() - init_tick < 200)
    {
        SetZeroPos();
        return;
    }

    legFsm.HandleInput();
    legFsm.Update();

    if (!jumping && Time::GetTick() - jump_finish > 50)
        LeaveGround();
}

void LegController::UpdateStateVariables()
{
    // 腿部五连杆模型解算，计算腿部倾角和腿长，及更新各种所需变量

    last_theta_l = theta_l;
    last_theta_r = theta_r;
    last_length_l = length_l;
    last_length_r = length_r;

    float theta4 = -(m_LegMotor[LMT_LeftFront].sensorFeedBack.positionFdb + 1.832596f) + 1.22173f;
    float theta1 = -(m_LegMotor[LMT_LeftRear].sensorFeedBack.positionFdb - 0.691456f - 1.145140f) + 1.919810f; // 哼哼哼啊啊啊啊啊啊啊啊啊♂
    float xd = LEG_LENGTH_2 * cos(theta4) + LEG_LENGTH_3 / 2;
    float yd = LEG_LENGTH_2 * sin(theta4);
    float xb = LEG_LENGTH_2 * cos(theta1) - LEG_LENGTH_3 / 2;
    float yb = LEG_LENGTH_2 * sin(theta1);
    float A0 = 2 * LEG_LENGTH_1 * (xd - xb);
    float B0 = 2 * LEG_LENGTH_1 * (yd - yb);
    float lbd = sqrt((xb - xd) * (xb - xd) + (yb - yd) * (yb - yd));
    float C0 = lbd * lbd;
    float theta2 = 2 * atan2(B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0), A0 + C0);
    float xc = LEG_LENGTH_2 * cos(theta1) + LEG_LENGTH_1 * cos(theta2) - LEG_LENGTH_3 / 2;
    float yc = LEG_LENGTH_2 * sin(theta1) + LEG_LENGTH_1 * sin(theta2);
    length_l = sqrt(xc * xc + yc * yc);
    theta_l = PI / 2 - (atan2(yc, -xc) - m_IMU->GetPitch()); ///////////////////!!!!! 在ATAN2 加了负号
    phi0_l = atan2(yc, xc);

    theta4 = (m_LegMotor[LMT_RightFront].sensorFeedBack.positionFdb - 1.832596f) + 1.22173f;             // 前一个值是总可旋转角度，后一个是俯角（其实无所谓，两个值完全可以合一，具体含义可以看whx的五连杆解算）
    theta1 = (m_LegMotor[LMT_RightRear].sensorFeedBack.positionFdb + 0.691456f + 1.145140f) + 1.919810f; // 前一个值是总可旋转角度，后一个是俯角补角，怎么来的不知道，反正推断的-Rededge 0203  17：00
    xd = LEG_LENGTH_2 * cos(theta4) + LEG_LENGTH_3 / 2;
    yd = LEG_LENGTH_2 * sin(theta4);
    xb = LEG_LENGTH_2 * cos(theta1) - LEG_LENGTH_3 / 2;
    yb = LEG_LENGTH_2 * sin(theta1);
    A0 = 2 * LEG_LENGTH_1 * (xd - xb);
    B0 = 2 * LEG_LENGTH_1 * (yd - yb);
    lbd = sqrt((xb - xd) * (xb - xd) + (yb - yd) * (yb - yd));
    C0 = lbd * lbd;
    theta2 = 2 * atan2(B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0), A0 + C0);
    xc = LEG_LENGTH_2 * cos(theta1) + LEG_LENGTH_1 * cos(theta2) - LEG_LENGTH_3 / 2;
    yc = LEG_LENGTH_2 * sin(theta1) + LEG_LENGTH_1 * sin(theta2);
    length_r = sqrt(xc * xc + yc * yc);
    theta_r = PI / 2 - (atan2(yc, -xc) - m_IMU->GetPitch());
    phi0_r = atan2(yc, xc);

    zm = m_IMU->get_m_a(2) - 9.81f;

    last_length_l_dot = length_l_dot;
    last_length_r_dot = length_r_dot;
    last_theta_l_dot = theta_l_dot;
    last_theta_r_dot = theta_r_dot;

    length_l_dot = (length_l - last_length_l) / 0.002f;
    length_r_dot = (length_r - last_length_r) / 0.002f;
    theta_l_dot = (theta_l - last_theta_l) / 0.002f;
    theta_r_dot = (theta_r - last_theta_r) / 0.002f;

    distance = STP23L::Instance()->GetDistance();
    distance_counter = STP23L::Instance()->GetCounter();

    distanceFilter.SetInput(distance);
    distanceFilter.Update();
    distance_filtered = distanceFilter.GetResult();

    thetaDotL.SetInput(theta_l_dot);
    thetaDotR.SetInput(theta_r_dot);
    lengthDotL.SetInput(length_l_dot);
    lengthDotR.SetInput(length_r_dot);

    thetaDotL.Update();
    thetaDotR.Update();
    lengthDotL.Update();
    lengthDotR.Update();

    theta_l_dot = thetaDotL.GetResult();
    theta_r_dot = thetaDotR.GetResult();
    length_l_dot = lengthDotL.GetResult();
    length_r_dot = lengthDotR.GetResult();

    length_l_ddot = (length_l_dot - last_length_l_dot) / 0.002f;
    length_r_ddot = (length_r_dot - last_length_r_dot) / 0.002f;
    theta_l_ddot = (theta_l_dot - last_theta_l_dot) / 0.002f;
    theta_r_ddot = (theta_r_dot - last_theta_r_dot) / 0.002f;

    thetaDDotL.SetInput(theta_l_ddot);
    thetaDDotR.SetInput(theta_r_ddot);
    lengthDDotL.SetInput(length_l_ddot);
    lengthDDotR.SetInput(length_r_ddot);

    thetaDDotL.Update();
    thetaDDotR.Update();
    lengthDDotL.Update();
    lengthDDotR.Update();

    theta_l_ddot = thetaDDotL.GetResult();
    theta_r_ddot = thetaDDotR.GetResult();
    length_l_ddot = lengthDDotL.GetResult();
    length_r_ddot = lengthDDotR.GetResult();
}

void LegController::CalcJacob()
{
    float leg_theta_l = (PI / 2) - phi0_l;
    float leg_theta_r = (PI / 2) - phi0_r;
    float x_e_left = length_l * sin(leg_theta_l);
    float y_e_left = length_l * cos(leg_theta_l);
    float x_e_right = length_r * sin(leg_theta_r);
    float y_e_right = length_r * cos(leg_theta_r);

    float l_a = LEG_LENGTH_3 / 2;
    float l_u = LEG_LENGTH_2;

    theta1_l = m_LegMotor[LMT_LeftFront].sensorFeedBack.positionFdb + 3.752458f;   // HT02
    theta2_l = -m_LegMotor[LMT_LeftRear].sensorFeedBack.positionFdb + 3.752458f;   // HT01
    theta1_r = -m_LegMotor[LMT_RightFront].sensorFeedBack.positionFdb + 3.752458f; // HT04 2 HT05
    theta2_r = m_LegMotor[LMT_RightRear].sensorFeedBack.positionFdb + 3.752458f;   // HT03        还没更新

    float x_1_left = l_a - l_u * cos(theta1_l);
    float y_1_left = l_u * sin(theta1_l);
    float x_2_left = l_a - l_u * cos(theta2_l);
    float y_2_left = l_u * sin(theta2_l);
    float x_1_right = l_a - l_u * cos(theta1_r);
    float y_1_right = l_u * sin(theta1_r);
    float x_2_right = l_a - l_u * cos(theta2_r);
    float y_2_right = l_u * sin(theta2_r);

    float dtheta1_dl_left = 1 / l_u * ((x_e_left + x_1_left) * sin(leg_theta_l) + (y_e_left - y_1_left) * cos(leg_theta_l)) / (-(x_e_left + x_1_left) * sin(theta1_l) + (y_e_left - y_1_left) * cos(theta1_l));
    float dtheta1_dtheta_left = length_l / l_u * ((x_e_left + x_1_left) * cos(leg_theta_l) - (y_e_left - y_1_left) * sin(leg_theta_l)) / (-(x_e_left + x_1_left) * sin(theta1_l) + (y_e_left - y_1_left) * cos(theta1_l));
    float dtheta2_dl_left = 1 / l_u * ((x_e_left - x_2_left) * sin(leg_theta_l) + (y_e_left - y_2_left) * cos(leg_theta_l)) / ((x_e_left - x_2_left) * sin(theta2_l) + (y_e_left - y_2_left) * cos(theta2_l));
    float dtheta2_dtheta_left = length_l / l_u * ((x_e_left - x_2_left) * cos(leg_theta_l) - (y_e_left - y_2_left) * sin(leg_theta_l)) / ((x_e_left - x_2_left) * sin(theta2_l) + (y_e_left - y_2_left) * cos(theta2_l));

    float dtheta1_dl_right = 1 / l_u * ((x_e_right + x_1_right) * sin(leg_theta_r) + (y_e_right - y_1_right) * cos(leg_theta_r)) / (-(x_e_right + x_1_right) * sin(theta1_r) + (y_e_right - y_1_right) * cos(theta1_r));
    float dtheta1_dtheta_right = length_r / l_u * ((x_e_right + x_1_right) * cos(leg_theta_r) - (y_e_right - y_1_right) * sin(leg_theta_r)) / (-(x_e_right + x_1_right) * sin(theta1_r) + (y_e_right - y_1_right) * cos(theta1_r));
    float dtheta2_dl_right = 1 / l_u * ((x_e_right - x_2_right) * sin(leg_theta_r) + (y_e_right - y_2_right) * cos(leg_theta_r)) / ((x_e_right - x_2_right) * sin(theta2_r) + (y_e_right - y_2_right) * cos(theta2_r));
    float dtheta2_dtheta_right = length_r / l_u * ((x_e_right - x_2_right) * cos(leg_theta_r) - (y_e_right - y_2_right) * sin(leg_theta_r)) / ((x_e_right - x_2_right) * sin(theta2_r) + (y_e_right - y_2_right) * cos(theta2_r));

    inv_Jacob_L[0][0] = dtheta1_dl_left;
    inv_Jacob_L[0][1] = dtheta2_dl_left;
    inv_Jacob_L[1][0] = dtheta1_dtheta_left;
    inv_Jacob_L[1][1] = dtheta2_dtheta_left;

    inv_Jacob_R[0][0] = dtheta1_dl_right;
    inv_Jacob_R[0][1] = dtheta2_dl_right;
    inv_Jacob_R[1][0] = dtheta1_dtheta_right;
    inv_Jacob_R[1][1] = dtheta2_dtheta_right;

    float torque_fdb_lf = m_LegMotor[LMT_LeftFront].sensorFeedBack.torqueFdb;
    float torque_fdb_lr = -m_LegMotor[LMT_LeftRear].sensorFeedBack.torqueFdb;
    float torque_fdb_rf = -m_LegMotor[LMT_RightFront].sensorFeedBack.torqueFdb;
    float torque_fdb_rr = m_LegMotor[LMT_RightRear].sensorFeedBack.torqueFdb;

    calc_F_l = inv_Jacob_L[0][0] * torque_fdb_lf + inv_Jacob_L[0][1] * torque_fdb_lr;
    calc_T_l = inv_Jacob_L[1][0] * torque_fdb_lf + inv_Jacob_L[1][1] * torque_fdb_lr;
    calc_F_r = inv_Jacob_R[0][0] * torque_fdb_rf + inv_Jacob_R[0][1] * torque_fdb_rr;
    calc_T_r = inv_Jacob_R[1][0] * torque_fdb_rf + inv_Jacob_R[1][1] * torque_fdb_rr;

    P_L = calc_F_l * cos(theta_l) + fabs(calc_T_l) * sin(theta_l) / length_l;
    P_R = calc_F_r * cos(theta_r) + fabs(calc_T_r) * sin(theta_r) / length_r;

    float det_left = dtheta1_dl_left * dtheta2_dtheta_left - dtheta1_dtheta_left * dtheta2_dl_left;
    float det_right = dtheta1_dl_right * dtheta2_dtheta_right - dtheta1_dtheta_right * dtheta2_dl_right;

    Jacob_L[0][0] = dtheta2_dtheta_left / det_left;
    Jacob_L[0][1] = -dtheta2_dl_left / det_left;
    Jacob_L[1][0] = -dtheta1_dtheta_left / det_left;
    Jacob_L[1][1] = dtheta1_dl_left / det_left;

    Jacob_R[0][0] = dtheta2_dtheta_right / det_right;
    Jacob_R[0][1] = -dtheta2_dl_right / det_right;
    Jacob_R[1][0] = -dtheta1_dtheta_right / det_right;
    Jacob_R[1][1] = dtheta1_dl_right / det_right;
}

void LegController::JudgeLoseControl()
{
    // 判断是否失控，失控时进入松弛状态
    uint8_t lose_count = 0;
    if (target_height > 0.24f && fabs(m_IMU->GetPitch()) > 0.18f)
        lose_count += 2;
    if (target_height < 0.22f && target_height > 0.2f)
    {
        float max_theta = fabs(theta_l) > fabs(theta_r) ? fabs(theta_l) : fabs(theta_r);
        height_bias = Math::FloatConstrain((max_theta - 0.7f) * 0.4f, 0.0f, 0.08f);
    }
    else
    {
        height_bias = 0.0f;
    }
    if (fabs(m_IMU->GetPitch()) > 0.175f)
        lose_count++;
    if (fabs(theta_l) > 0.9f)
        lose_count++;
    if (fabs(theta_r) > 0.9f)
        lose_count++;
}

void LegController::LeaveGround()
{
    float m_w = 1.5f; // 轮子质量

    // 计算地面对轮的支持力
    FN_L = P_L + m_w * G + m_w * (zm - length_l_ddot * cos(theta_l) + 2.0f * length_l_dot * theta_l_dot * sin(theta_l) + length_l * theta_l_ddot * sin(theta_l) + length_l * theta_l_dot * theta_l_dot * cos(theta_l));
    FN_R = P_R + m_w * G + m_w * (zm - length_r_ddot * cos(theta_r) + 2.0f * length_r_dot * theta_r_dot * sin(theta_r) + length_r * theta_r_ddot * sin(theta_r) + length_r * theta_r_dot * theta_r_dot * cos(theta_r));

    // 支持力小于一定值时，最大静摩擦不足以支持系统稳定，则认为离地
    l_leave_ground = (FN_L < 17.0f);
    r_leave_ground = (FN_R < 17.0f);
}

void LegController::CalcF()
{
    target_height = target_height - height_bias;
    // 通过两侧腿长平均值计算腿长PID，为正常工作模式
    legLengthPid.ref = target_height;
    legLengthPid.fdb = (length_l * cos(theta_l) + length_r * cos(theta_r)) / 2;
    legLengthPid.UpdateResult();
    // float legLengthResult = legLengthPid.result - legLengthPid.dResult - (length_l_dot + length_r_dot) * HEIGHT_KD;
    debug_length_p_result = legLengthPid.pResult;
    debug_length_i_result = legLengthPid.iResult;
    debug_length_d_result = legLengthPid.dResult;
    debug_length_result = legLengthPid.result;

    lengthDFilter.SetInput(legLengthPid.dResult);
    lengthDFilter.Update();
    length_d_result_filtered = lengthDFilter.GetResult();

    float legLengthResult = legLengthPid.result - legLengthPid.dResult + length_d_result_filtered;

    // 两侧腿长单独计算PID，为单侧离地/跳跃模式
    leftLegLengthPid.ref = target_height;
    leftLegLengthPid.fdb = length_l * cos(theta_l);
    leftLegLengthPid.UpdateResult();
    // float leftLegLengthResult = leftLegLengthPid.result - leftLegLengthPid.dResult - length_l_dot * HEIGHT_KD;
    float leftLegLengthResult = leftLegLengthPid.result;

    rightLegLengthPid.ref = target_height;
    rightLegLengthPid.fdb = length_r * cos(theta_r);
    rightLegLengthPid.UpdateResult();
    // float rightLegLengthResult = rightLegLengthPid.result - rightLegLengthPid.dResult - length_r_dot * HEIGHT_KD;
    float rightLegLengthResult = rightLegLengthPid.result;

    // 通过IMU的roll角计算rollPID
    rollPid.ref = target_roll;
    rollPid.fdb = m_IMU->GetRoll();
    rollPid.UpdateResult();
    float rollResult = rollPid.result - rollPid.dResult + m_IMU->tmp_m_gyro[1] * ROLL_KD;

    // 左右侧重力补偿，根据实际情况测量得到（rollPID和腿长PID加i，观察最终两侧输出F值）
    float left_gravity = 80.0f;
    float right_gravity = 80.0f;

    // 跳跃瞬间需要给一个较大的力
    float left_jump = 250.0f;
    float right_jump = 250.0f;

    // 根据是否离地给定左右侧腿部支持力F
    if (l_leave_ground)
    {
        F_L = left_gravity + leftLegLengthResult;
    }
    else
    {
        F_L = left_gravity + legLengthResult + rollResult;
        // F_L = left_gravity + legLengthResult + rollResult;
    }

    if (r_leave_ground)
    {
        F_R = right_gravity + rightLegLengthResult;
    }
    else
    {
        F_R = right_gravity + legLengthResult - rollResult;
        // F_R = right_gravity + legLengthResult - rollResult;
    }

    if (!(l_leave_ground && r_leave_ground) && !jumping)
    {
        F_L += compensation_l;
        F_R += compensation_r;
    }

    // 跳跃模式
    if (jumping)
    {
        switch (jump_state)
        {
        // 0: 腿部下沉，准备跳跃
        case 0:
            uint32_t jump_tick = Time::GetTick() - jump_time;
            leftLegLengthPid.ref = LEG_MID_HEIGHT - jump_tick * 0.0003f;
            leftLegLengthPid.fdb = length_l;
            leftLegLengthPid.UpdateResult();
            leftLegLengthResult = leftLegLengthPid.result;

            rightLegLengthPid.ref = LEG_MID_HEIGHT - jump_tick * 0.0003f;
            rightLegLengthPid.fdb = length_r;
            rightLegLengthPid.UpdateResult();
            rightLegLengthResult = rightLegLengthPid.result;

            F_L = left_gravity + leftLegLengthResult + rollResult;
            F_R = right_gravity + rightLegLengthResult - rollResult;

            if (!c_down)
            {
                jumping = false;
                jump_state = 0;
            }

            if (length_l < 0.15f && length_r < 0.15f)
            {
                if ((distance < (uint16_t)(fabs(s_dot) * 450.0f)) && distance >= 100)
                {
                    jump_state = 1;
                    jump_time = Time::GetTick();
                }
            }
            break;

        // 1: 瞬间给一个较大的力，使腿部离地
        case 1:
            leftLegLengthPid.ref = LEG_MAX_HEIGHT + 0.01f;
            leftLegLengthPid.fdb = length_l;
            leftLegLengthPid.UpdateResult();
            leftLegLengthResult = leftLegLengthPid.result;

            rightLegLengthPid.ref = LEG_MAX_HEIGHT + 0.01f;
            rightLegLengthPid.fdb = length_r;
            rightLegLengthPid.UpdateResult();
            rightLegLengthResult = rightLegLengthPid.result;

            F_L = left_jump + leftLegLengthResult + rollResult;
            F_R = right_jump + rightLegLengthResult - rollResult;

            if (Time::GetTick() - jump_time > 1000 || (length_l > 0.35f && length_r > 0.35f))
            {
                jump_state = 2;
                jump_time = Time::GetTick();
            }
            break;

        // 2: 在空中收腿
        case 2:
            leftLegLengthPid.ref = LEG_MIN_HEIGHT;
            leftLegLengthPid.fdb = length_l;
            leftLegLengthPid.UpdateResult();
            leftLegLengthResult = leftLegLengthPid.result;

            rightLegLengthPid.ref = LEG_MIN_HEIGHT;
            rightLegLengthPid.fdb = length_r;
            rightLegLengthPid.UpdateResult();
            rightLegLengthResult = rightLegLengthPid.result;

            F_L = -20.0f + leftLegLengthResult + rollResult;
            F_R = -20.0f + rightLegLengthResult - rollResult;

            if (Time::GetTick() - jump_time > 500 || (length_l < 0.14f && length_r < 0.14f))
            {
                jump_state = 3;
            }
            break;

        // 3: 收腿完成，回归正常模式&离地状态
        case 3:
            jumping = false;
            jump_finish = Time::GetTick();
            jump_state = 0;
            l_leave_ground = true;
            r_leave_ground = true;
            break;

        default:
            jumping = false;
            jump_finish = Time::GetTick();
            jump_state = 0;
            break;
        }
    }
}

void LegController::CalcMotorTorque()
{
    torque_lf = Jacob_L[0][0] * F_L + Jacob_L[0][1] * T_L;
    torque_lr = Jacob_L[1][0] * F_L + Jacob_L[1][1] * T_L;
    torque_rf = Jacob_R[0][0] * F_R + Jacob_R[0][1] * T_R;
    torque_rr = Jacob_R[1][0] * F_R + Jacob_R[1][1] * T_R;
}

void LegController::SetMotorTorque()
{
    // float torquemax = 30.0f;
    // float torquemin = -30.0f;

    // torque_lf = Math::FloatConstrain(torque_lf, torquemin, torquemax);
    // torque_lr = Math::FloatConstrain(torque_lr, torquemin, torquemax);
    // torque_rf = Math::FloatConstrain(torque_rf, torquemin, torquemax);
    // torque_rr = Math::FloatConstrain(torque_rr, torquemin, torquemax);

    // m_LegMotor[LMT_LeftFront].torqueSet = torque_lf;
    // m_LegMotor[LMT_LeftRear].torqueSet = -torque_lr;
    // m_LegMotor[LMT_RightFront].torqueSet = -torque_rf;
    // m_LegMotor[LMT_RightRear].torqueSet = torque_rr;

    m_LegMotor[LMT_LeftFront].torqueSet = 0.0f;
    m_LegMotor[LMT_LeftRear].torqueSet = 0.0f;
    m_LegMotor[LMT_RightFront].torqueSet = 0.0f;
    m_LegMotor[LMT_RightRear].torqueSet = 0.0f;
}

void LegController::SetPos(float pos)
{
    m_LegMotor[LMT_LeftFront].positionSet = -pos;
    m_LegMotor[LMT_LeftRear].positionSet = pos;
    m_LegMotor[LMT_RightRear].positionSet = -pos;
    m_LegMotor[LMT_RightFront].positionSet = pos;
}

void LegController::SetSpd(float spd)
{
    m_LegMotor[LMT_LeftFront].speedSet = -spd;
    m_LegMotor[LMT_LeftRear].speedSet = spd;
    m_LegMotor[LMT_RightRear].speedSet = -spd;
    m_LegMotor[LMT_RightFront].speedSet = spd;
}

void LegController::SetCoSpd(float spd)
{
    for (int i = 0; i < LMT_LENGTH; ++i)
    {
        m_LegMotor[i].speedSet = spd;
    }
}

void LegController::SetCur(float cur)
{
    m_LegMotor[LMT_LeftFront].torqueSet = cur;
    m_LegMotor[LMT_LeftRear].torqueSet = cur;
    m_LegMotor[LMT_RightRear].torqueSet = cur;
    m_LegMotor[LMT_RightFront].torqueSet = cur;
}

void LegController::RelaxLeg()
{
    for (int i = 0; i < LMT_LENGTH; ++i)
    {
        m_LegMotor[i].controlMode = Motor::RELAX_MODE;
    }
}

void LegController::MotivateLeg()
{
    for (int i = 0; i < LMT_LENGTH; ++i)
    {
        m_LegMotor[i].Motivate();
    }
}

void LegController::LegPosMode()
{
    for (int i = 0; i < LMT_LENGTH; ++i)
    {
        m_LegMotor[i].controlMode = Motor::POS_MODE;
    }
}

void LegController::LegSpdMode()
{
    for (int i = 0; i < LMT_LENGTH; ++i)
    {
        m_LegMotor[i].controlMode = Motor::SPD_MODE;
    }
}

void LegController::LegCurMode()
{
    for (int i = 0; i < LMT_LENGTH; ++i)
    {
        m_LegMotor[i].controlMode = Motor::CUR_MODE;
    }
}

void LegController::SetZeroPos()
{
    for (int i = 0; i < LMT_LENGTH; ++i)
    {
        m_LegMotor[i].SetZeroPos();
    }
}

void LegController::ResetCurrentState()
{
    target_height = LEG_MID_HEIGHT;
    jumping = false;
    jump_finish = Time::GetTick();
    jump_state = 0;
    l_leave_ground = false;
    r_leave_ground = false;
    legLengthPid.Clear();
    leftLegLengthPid.Clear();
    rightLegLengthPid.Clear();
    rollPid.Clear();
}

void LegController::ClearFilters()
{
    lengthDFilter.Clear();
    lengthDotL.Clear();
    lengthDotR.Clear();
    thetaDotL.Clear();
    thetaDotR.Clear();
    lengthDDotL.Clear();
    lengthDDotR.Clear();
    thetaDDotL.Clear();
    thetaDDotR.Clear();
}

void LegFsm::Init()
{
    hasInited = false;
    LegStateRelax::Instance()->Init(m_pOwner);
    LegStateBalance::Instance()->Init(m_pOwner);
    LegStateRemote::Instance()->Init(m_pOwner);
    LegStateInit::Instance()->Init(m_pOwner);
    LegStateFixedPos::Instance()->Init(m_pOwner);
    SetCurrentState(LegStateRelax::Instance());
}

void LegFsm::HandleInput()
{
    Dr16 *p_dr16 = Dr16::Instance();
    // if (relaxState)
    // {
    //     ChangeState(LegStateRelax::Instance());
    //     return;
    // }
    // if (initState)
    // {
    //     ChangeState(LegStateInit::Instance());
    //     return;
    // }
    // if (Dr16::Instance()->QuerySwState(Dr16::RC_SW_L, Dr16::RC_SW_UP))
    // {
    //     ChangeState(LegStateRemote::Instance());
    //     return;
    // }
    // if (Dr16::Instance()->QuerySwState(Dr16::RC_SW_L, Dr16::RC_SW_DOWN))
    // {
    //     ChangeState(LegStateRelax::Instance());
    //     return;
    // }
    ChangeState(LegStateRelax::Instance());
}
