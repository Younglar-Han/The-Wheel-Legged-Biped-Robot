#include "WheelController.hpp"
#include "WheelStateRelax.hpp"
#include "WheelStateBalance.hpp"
#include "WheelStateRemote.hpp"
#include "WheelStateInit.hpp"
#include "arm_math.h"

#include "Math.hpp"
#include "Buzzer.hpp"

// 瓴控9025电机速度PID控制参数定义
const float WheelController::LK9025StandardSpeedKp = 200.0f; // 速度环P参数，原值200f，于0203 14:40修改
const float WheelController::LK9025StandardSpeedKi = 1.0f;   // 速度环I参数
const float WheelController::LK9025StandardSpeedKd = 50.0f;  // 速度环D参数，原值50f，于0204 11:17修改
const float WheelController::LK9025StandardSpeedMaxout = 2000.0f; // 速度环最大输出限制
const float WheelController::LK9025StandardSpeedIMaxout = 0.0f;   // 速度环积分项最大输出限制（为0表示不使用积分项限制）
const float WheelController::LK9025MaxSpeed = 60.0f;         // 瓴控9025电机最大速度限制

// 全局调试指针，用于外部访问轮控制器实例
WheelController *pWheelDebugHandler;

WheelController::WheelController() : ControllerEntity(ECT_WheelController),
                                     m_IMU(AHRSEstimator::Instance()),
                                     torque_leg_l(0.0f),       // 初始化左腿力矩为0
                                     torque_leg_r(0.0f),       // 初始化右腿力矩为0
                                     state_vmc(false),         // 初始化VMC模式为关闭
                                     state_relax(true),        // 初始化为松弛状态
                                     s_dot_acc(0.0f),          // 初始化速度加速度为0
                                     wheelFsm(this)            // 初始化状态机，传入this指针
{
}

void WheelController::Init()
{
    // 设置控制更新周期为2个时钟周期
    SetDefaultTicksToUpdate(2);
    // 设置调试指针指向当前对象
    pWheelDebugHandler = this;

    // 瓴控电机注册 以车前进方向（目前为电池安装方向）为正，左边注册为2，右边为1（瓴控没有0号id），即0x141和0x142
    for (int i = 0; i < WMT_LENGTH; ++i)
    {
        // 按照CAN ID注册电机，左右轮对应不同的ID
        m_WheelMotor[i].RegistMotor(CAN2, 0x142 - i);
        // 设置为电流（力矩）控制模式
        m_WheelMotor[i].controlMode = Motor::CUR_MODE;
    }

    // 设置速度斜坡滤波器参数，防止速度突变
    m_RampFilter_s.SetMargin(0.0024f);
    // 设置偏航角斜坡滤波器参数
    m_RampFilter_yaw.SetMargin(0.008f);
    // 设置速度一阶滤波器时间常数
    m_s_dot_filter.SetTau(0.05f);
    // 设置速度滤波器更新周期
    m_s_dot_filter.SetUpdatePeriod(2);
    // 设置偏航角速度一阶滤波器时间常数
    m_YawDotFilter.SetTau(0.01f);
    // 设置偏航角速度滤波器更新周期
    m_YawDotFilter.SetUpdatePeriod(2);

    // 设置陀螺仪滤波器参数
    for (int i = 0; i < 3;i++)
    {
        m_gyro_filter[i].SetTau(1.0f);
        m_gyro_filter[i].SetUpdatePeriod(2);
    }

    // 获取腿部控制器实例
    m_pLegController = (LegController *)this->GetOwner()->GetEntity(ECT_LegController);

    // 设置底盘跟随PID参数（Kp, Ki, Kd, 最大输出, I项最大输出）
    chassisFollowPid.SetParams(2.0f, 0.0f, 25.0f, 50.0f, 30.0f);
    // 设置位置跟随PID参数
    sFollowPid.SetParams(10.0f, 0.0f, 100.0f, 3.0f, 0.0f);

    // 为轮子电机设置速度PID参数
    for (int i = 0; i < WMT_LENGTH; i++)
    {
        m_WheelMotor[i].pidSpeed.SetParams(LK9025StandardSpeedKp, LK9025StandardSpeedKi, LK9025StandardSpeedKd, LK9025StandardSpeedMaxout, LK9025StandardSpeedIMaxout);
    }

    // 初始化轮子状态机
    wheelFsm.Init();

    // 获取当前偏航角作为初始值
    yaw = m_IMU->GetYaw();
    // 重置当前状态
    ResetCurrentState();
}

void WheelController::Update()
{
    // 处理状态机输入并更新状态
    wheelFsm.HandleInput();
    wheelFsm.Update();

    // 检查左右轮电机是否超时，若超时则触发蜂鸣器警告
    if ((m_WheelMotor[0].sensorFeedBack.IsTimeout() || m_WheelMotor[1].sensorFeedBack.IsTimeout()))
    {
        // Buzzer::Instance()->On();
        buzzer_tick = Time::GetTick();
    }
    else
    {
        // 超时解除后一段时间关闭蜂鸣器
        if (Time::GetTick() - buzzer_tick > 1000)
        {
            Buzzer::Instance()->Off();
        }
    }
    // 记录左右轮电机反馈的更新周期，用于监控通信状态
    wheel1_update_period = Time::GetTick() - m_WheelMotor[0].sensorFeedBack.GetLastUpdateTick();
    wheel2_update_period = Time::GetTick() - m_WheelMotor[1].sensorFeedBack.GetLastUpdateTick();

    CalcLQR();              // 计算LQR控制器增益矩阵
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

    // 根据当前状态决定控制方式
    if (!state_relax)
    {
        // 非松弛状态
        if (!state_spd)
        {
            // 力矩控制模式
            SetWheelTorque(); // 设置轮子力矩
        }
        else
        {
            // 速度控制模式
            WheelSpdMode(); // 设置轮子速度模式
        }
    }
    else
    {
        // 松弛状态
        RelaxWheel();        // 轮子松开
        ResetCurrentState(); // 重置当前状态
    }
}

void WheelController::CalcLQR()
{
    // 获取左腿长度
    float l_l = m_pLegController->GetLengthL();
    // 获取右腿长度
    float l_r = m_pLegController->GetLengthR();
    // 计算左腿长度的平方，用于后续多项式拟合
    float l_l2 = l_l * l_l;
    // 计算右腿长度的平方，用于后续多项式拟合
    float l_r2 = l_r * l_r;
    // 计算左右腿长度的乘积，用于后续多项式拟合
    float l_lr = l_l * l_r;

    // 通过双边腿长拟合LQR的K矩阵及A、B矩阵
    // 这里使用多项式拟合方法，根据双腿长度动态计算适合当前状态的控制参数
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            // 计算LQR反馈增益矩阵K，使用六项多项式拟合
            // K = constSet[0] + constSet[1]*l_l + constSet[2]*l_l^2 + constSet[3]*l_r + constSet[4]*l_r^2 + constSet[5]*l_l*l_r
            LQR_K[i][j] = constSet[0][i][j] + constSet[1][i][j] * l_l + constSet[2][i][j] * l_l2 + constSet[3][i][j] * l_r + constSet[4][i][j] * l_r2 + constSet[5][i][j] * l_lr;
            // 同样方法计算系统状态空间A矩阵
            A_matrix[i][j] = AconstSet[0][i][j] + AconstSet[1][i][j] * l_l + AconstSet[2][i][j] * l_l2 + AconstSet[3][i][j] * l_r + AconstSet[4][i][j] * l_r2 + AconstSet[5][i][j] * l_lr;
            // 只计算B矩阵的前4列
            if (j < 4)
                // 计算系统输入矩阵B
                B_matrix[i][j] = BconstSet[0][i][j] + BconstSet[1][i][j] * l_l + BconstSet[2][i][j] * l_l2 + BconstSet[3][i][j] * l_r + BconstSet[4][i][j] * l_r2 + BconstSet[5][i][j] * l_lr;
        }
    }

    // 离地时对应的K矩阵置零
    // 当左脚离地时，相关控制增益需要调整
    if (m_pLegController->IsLeftLeaveGround())
    {
        for (int i = 0; i < 10; i++)
        {
            // 左轮扭矩相关的控制增益置零
            LQR_K[0][i] = 0.0f;
            // 除了左腿角度和角速度外，左腿扭矩相关的控制增益置零
            if (i != 4 && i != 5)
                LQR_K[2][i] = 0.0f;
        }
        // 右腿扭矩对左腿角度和角速度的影响置零
        LQR_K[3][4] = 0.0f;
        LQR_K[3][5] = 0.0f;
    }
    // 当右脚离地时，相关控制增益需要调整
    if (m_pLegController->IsRightLeaveGround())
    {
        for (int i = 0; i < 10; i++)
        {
            // 右轮扭矩相关的控制增益置零
            LQR_K[1][i] = 0.0f;
            // 除了右腿角度和角速度外，右腿扭矩相关的控制增益置零
            if (i != 6 && i != 7)
                LQR_K[3][i] = 0.0f;
        }
        // 左腿扭矩对右腿角度和角速度的影响置零
        LQR_K[2][6] = 0.0f;
        LQR_K[2][7] = 0.0f;
    }
}

void WheelController::UpdateStateVariables()
{
    // 判断左腿是否离地，同时考虑VMC模式状态
    bool isLeftLeaveGround = m_pLegController->IsLeftLeaveGround() && state_vmc;
    // 判断右腿是否离地，同时考虑VMC模式状态
    bool isRightLeaveGround = m_pLegController->IsRightLeaveGround() && state_vmc;

    // 获取左腿角度反馈
    theta_l = m_pLegController->GetThetaL();        // left leg angle fdb
    // 获取左腿角速度反馈
    theta_dot_l = m_pLegController->GetThetaLDot(); // left leg angle spd fdb

    // 获取右腿角度反馈
    theta_r = m_pLegController->GetThetaR();        // right leg angle fdb
    // 获取右腿角速度反馈
    theta_dot_r = m_pLegController->GetThetaRDot(); // right leg angle spd fdb

    // 获取机体pitch角度反馈（从IMU）
    theta_b = m_IMU->GetPitch();        // body angle fdb
    // 获取机体pitch角速度反馈（从IMU陀螺仪）
    theta_dot_b = m_IMU->tmp_m_gyro[0]; // body angle spd fdb

    // 获取左腿长度
    length_l = m_pLegController->GetLengthL();
    // 获取右腿长度
    length_r = m_pLegController->GetLengthR();

    // 计算电机关节角度误差，用于后续控制系数计算
    float theta_joint[4];
    float min_err = 1.0f;
    for (int i = 0; i < 4;i++)
    {
        // 获取各个腿部电机的位置反馈
        theta_joint[i] = m_pLegController->GetLegMotor(LegController::LegMotorType(i))->pFeedback->positionFdb;
        // 计算关节角度误差，取绝对值和绝对值差值中的较小值
        float err = fabs(theta_joint[i]) > fabs(theta_joint[i] - 1.7f) ? fabs(theta_joint[i] - 1.7f) : fabs(theta_joint[i]);
        // 取所有关节中的最小误差
        min_err = err < min_err ? err : min_err;
    }
    // 根据最小误差计算theta控制系数（限制在0到1之间）
    theta_coe = Math::FloatConstrain(min_err * 4.0f, 0.0f, 1.0f);
    // 根据偏航角误差计算yaw控制系数（偏航角越大，系数越小，限制在0到1之间）
    yaw_coe = Math::FloatConstrain((0.43f - fabs(delta_yaw)) * 3.0f, 0.0f, 1.0f);
    // 当目标速度接近零时，设置yaw系数为1
    if (fabs(real_target_s_dot) < 0.05f)
        yaw_coe = 1.0f;
    // 综合系数为theta系数和yaw系数的乘积
    s_coe = theta_coe * yaw_coe;

    // 保存上一次的速度值
    last_s_dot = s_dot;
    // 计算左轮速度（转换为线速度）
    s_dot_l = m_WheelMotor[0].sensorFeedBack.speedFdb * 0.1f;  // wheel radius * (left wheel spd fdb)
    // 计算右轮速度（转换为线速度，注意符号）
    s_dot_r = -m_WheelMotor[1].sensorFeedBack.speedFdb * 0.1f; // wheel radius * (right wheel spd fdb)
    // 计算整体速度为左右轮速度平均值
    s_dot = 0.5f * (s_dot_l + s_dot_r);                        // 1/2 * (left wheel spd + right wheel spd)

    // 根据轮速差和腿部运动计算偏航角速度
    calc_yaw_dot = (- s_dot_l + s_dot_r) / 0.502f - length_l * cos(theta_l) * theta_dot_l / 0.502f + length_r * cos(theta_r) * theta_dot_r / 0.502f;
    // 将计算的偏航角速度输入到滤波器
    m_YawDotFilter.SetInput(calc_yaw_dot);
    // 更新滤波器
    m_YawDotFilter.Update();
    // 获取滤波后的偏航角速度
    calc_yaw_dot = m_YawDotFilter.GetResult();

    // 根据计算的偏航角速度与IMU测量的偏航角速度差异计算估计系数
    // 差异越小，估计系数越接近1
    float estimate_coe = Math::FloatConstrain(2.0f - 2.0f * fabs(calc_yaw_dot - m_IMU->tmp_m_gyro[2]), 0.0f, 1.0f);

    // 估计速度（使用卡尔曼滤波）
    SpeedEstimate();

    // 将当前速度输入到滤波器
    m_s_dot_filter.SetInput(s_dot);
    // 更新滤波器
    m_s_dot_filter.Update();
    // 获取滤波后的速度
    filtered_s_dot = m_s_dot_filter.GetResult();
    // 保存原始速度值
    original_s_dot = s_dot;
    // 计算轮-身体速度差
    vel_est = vel - body_wheel_spd;
    // 融合滤波速度和估计速度，根据estimate_coe加权
    s_dot = filtered_s_dot * estimate_coe + vel_est * (1.0f - estimate_coe);
	// s_dot = filtered_s_dot;

    // 将计算的速度传给腿部控制器
    m_pLegController->SetSDot(s_dot);

    // 根据速度大小动态调整偏航滤波器的边界值
    float margin = fabs(s_dot) > 0.7f ? 0.006f : (0.05f * fabs(s_dot) * fabs(s_dot) - 0.07f * fabs(s_dot) + 0.025f);
    m_RampFilter_yaw.SetMargin(margin);

    // 判断是否处于运动状态（目标速度大于阈值）
    if (fabs(real_target_s_dot) > 0.01f)
    {
        move_flag = true;
    }

    // 应用速度滤波，获取平滑的目标速度
    target_s_dot = SdotFilter(real_target_s_dot); // wheel spd set

    // 当处于运动状态且目标速度接近零时，设置目标位置为当前位置，准备停止
    if (move_flag && fabs(target_s_dot) < 0.01f)
    {
        target_s = s;
        // 当实际速度也接近零时，清除运动标志
        if (fabs(s_dot) < 0.03f)
            move_flag = false;
    }

    // 位置偏差过大时，认为失控，重置目标位置
    if (fabs(s - target_s) > 2.0f)
    {
        target_s = s;
        m_pLegController->SetLoseJudge(true);
    }
    else
    {
        m_pLegController->SetLoseJudge(false);
    }

    // 保存上一次的目标速度
    last_target_s_dot = real_target_s_dot;

    // 左腿离地时，使用右轮速度作为整体速度
    if (isLeftLeaveGround)
        s_dot = s_dot_r;
    // 右腿离地时，使用左轮速度作为整体速度
    if (isRightLeaveGround)
        s_dot = s_dot_l;
    // 如果双腿都离地，重置状态
    if (isLeftLeaveGround && isRightLeaveGround)
        ResetCurrentState();

    // 更新位置，积分速度（时间步长0.002秒）
    s += s_dot * 0.002f;               // wheel fdb
    // 更新目标位置，积分目标速度
    target_s += target_s_dot * 0.002f; // wheel fdb

    // 应用偏航角滤波
    delta_yaw = YawFilter(real_delta_yaw);
    // 获取当前偏航角
    yaw = m_IMU->GetYaw();
    // 计算目标偏航角
    target_yaw = yaw - delta_yaw;

    // 设置底盘跟随PID的参考值和反馈值
    chassisFollowPid.ref = 0.0f;
    chassisFollowPid.fdb = delta_yaw;
    // 更新PID计算结果
    chassisFollowPid.UpdateResult();
    // 获取IMU偏航角速度反馈
    yaw_dot = m_IMU->tmp_m_gyro[2];            // imu yaw spd fdb
    // 累加PID结果到目标偏航角速度
    target_yaw_dot += chassisFollowPid.result; // imu yaw spd set

    // 根据速度大小调整目标偏航角速度的系数（速度越大，系数越小）
    float s_dot_coe = Math::FloatConstrain(1.05f - 0.3f * fabs(s_dot), 0.5f, 1.0f);
    target_yaw_dot *= s_dot_coe;

    // 使用theta系数调整delta_yaw和target_yaw_dot
    delta_yaw *= theta_coe;
    target_yaw_dot *= theta_coe;

    // 计算补偿系数，当偏航角速度大于阈值时才启用补偿
    float compensation_coe = Math::FloatConstrain(yaw_dot - 0.8f, 0.0f, 1.0f);

    // 计算左右轮速差导致的补偿力矩
    float compensation = (length_l + length_r + 0.5f) * 10.0f * (s_dot_l * s_dot_l - s_dot_r * s_dot_r);
    compensation *= compensation_coe;
    // 分别计算左右腿的补偿力矩
    float compensation_l = compensation > 0.0f ? compensation : 0.0f;
    float compensation_r = compensation < 0.0f ? -compensation : 0.0f;
    // 将补偿力矩传递给腿部控制器
    m_pLegController->SetCompensation(compensation_l, compensation_r);
}

void WheelController::SpeedEstimate()
{
    // 计算机器人整体速度 = 轮子速度 + 腿部贡献的速度
    body_spd = s_dot + (length_l * cos(theta_l) * theta_dot_l + length_r * cos(theta_r) * theta_dot_r) / 2.0f;
    // 计算轮-身体速度差，即腿部运动贡献的速度
    body_wheel_spd = body_spd - s_dot;
    
    // 处理每个轴向的加速度和陀螺仪数据
    for (int i = 0; i < 3;i++)
    {
        // 将IMU测量的加速度转换为m/s^2单位
        body_acc[i] = m_IMU->motion_acc[i] * 9.81f;
        // 获取原始加速度值
        origin_acc[i] = m_IMU->get_m_a(i);
        // 保存上一个时刻的陀螺仪数据
        last_gyro[i] = current_gyro[i];
        // 获取当前陀螺仪数据
        current_gyro[i] = m_IMU->tmp_m_gyro[i];
        // 计算陀螺仪变化率并滤波
        m_gyro_filter[i].SetInput(current_gyro[i] - last_gyro[i]);
        m_gyro_filter[i].Update();
        // 获取滤波后的陀螺仪变化率
        delta_gyro[i] = m_gyro_filter[i].GetResult();
    }
    
    // 计算由偏航角速度引起的额外X方向加速度
    extra_acc_x_yaw = - current_gyro[2] * current_gyro[2] * CENTER_IMU_L + delta_gyro[2] * CENTER_IMU_W;
    // 计算由俯仰角速度引起的额外X方向加速度
    extra_acc_x_pitch = - current_gyro[0] * current_gyro[0] * CENTER_IMU_L - delta_gyro[0] * CENTER_IMU_H;
    // 计算由俯仰角速度引起的额外Z方向加速度
    extra_acc_z_pitch = - current_gyro[0] * current_gyro[0] * CENTER_IMU_H + delta_gyro[0] * CENTER_IMU_L;
    
    // 补偿去除额外加速度影响，获得真实X方向加速度
    acc_x = origin_acc[0] - extra_acc_x_yaw - extra_acc_x_pitch;
    // 补偿去除额外加速度影响，获得真实Z方向加速度
    acc_z = origin_acc[2] - extra_acc_z_pitch;
    
    // 保存上一时刻的实际加速度值
    last_acc = actual_acc;
    // 计算当前实际加速度，考虑机体俯仰角的影响，将加速度投影到行进方向
    actual_acc = acc_x * cos(m_IMU->GetPitch()) + acc_z * sin(m_IMU->GetPitch());
    
    // 取平均加速度
    u = (last_acc + actual_acc) / 2.0f;
    
    // 卡尔曼滤波预测步骤：根据上一状态和当前加速度预测当前速度
    vel_prior = vel + 0.002f * u;
    vel_predict = vel_prior;
    
    // 预测协方差更新，加入过程噪声
    vel_cov = vel_cov + 0.002f * VEL_PROCESS_NOISE;
    
    // 速度测量值为机器人整体速度
    vel_measure = body_spd;
    
    // 计算卡尔曼增益
    k = vel_cov / (vel_cov + VEL_MEASURE_NOISE);
    
    // 测量更新步骤：结合预测值和测量值，得到最优估计
    vel = vel_prior + k * (vel_measure - vel_prior);
    
    // 更新协方差
    vel_cov = (1 - k) * vel_cov;
    
    // 限制协方差范围，防止数值发散
    vel_cov = Math::FloatConstrain(vel_cov, 0.01f, 100.0f);
}

void WheelController::CalcWheelTorque()
{
    // 计算机体俯仰角与目标俯仰角的偏差
    float real_delta_pitch = theta_b - target_theta_b;
    // 限制俯仰角偏差在±0.1弧度范围内，防止控制量过大
    float delta_pitch = Math::FloatConstrain(real_delta_pitch, -0.1f, 0.1f);
    // 构建系统状态向量，包含位置偏差、速度偏差、偏航角、偏航角速度偏差、左右腿角度及角速度、俯仰角偏差及角速度
    float state[10] = {s - target_s, s_dot - target_s_dot, delta_yaw, yaw_dot - target_yaw_dot, theta_l, theta_dot_l, theta_r, theta_dot_r, real_delta_pitch, theta_dot_b};

    // 初始化左右轮和左右腿的力矩为0
    torque_l = 0.0f;
    torque_r = 0.0f;
    torque_leg_l = 0.0f;
    torque_leg_r = 0.0f;

    // 根据俯仰角偏差计算调节系数，偏差越大系数越小
    float pitch_coe = Math::FloatConstrain(4.0f - 20.0f * fabs(real_delta_pitch), 0.0f, 1.0f);

    // LQR控制：u = -K*x，通过状态向量与增益矩阵相乘计算控制输出
    for (int i = 0; i < 10; i++)
    {
        // 计算左轮力矩（LQR控制器增益矩阵与状态向量相乘）
        torque_l += LQR_K[0][i] * state[i];
        // 计算右轮力矩（LQR控制器增益矩阵与状态向量相乘）
        torque_r += LQR_K[1][i] * state[i];
        
        // 特殊处理俯仰角对腿部力矩的影响
        if (i == 8)
        {
            // 对俯仰角偏差应用特殊的调节系数
            torque_leg_l += LQR_K[2][i] * delta_pitch * pitch_coe;
            torque_leg_r += LQR_K[3][i] * delta_pitch * pitch_coe;
        }
        else
        {
            // 其他状态变量对腿部力矩的影响
            torque_leg_l += LQR_K[2][i] * state[i];
            torque_leg_r += LQR_K[3][i] * state[i];
        }
    }

    // 将计算的腿部力矩传递给腿部控制器，注意符号取反
    m_pLegController->SetTorque(-torque_leg_l, -torque_leg_r);
}

void WheelController::PredictOutput()
{
    // 初始化预测值为当前状态
    pred_s = s;
    pred_s_dot = s_dot;
    pred_yaw = yaw;
    pred_yaw_dot = yaw_dot;

    // 预测模型——x(k+1) += Ax(k) + Bu(k)
    // 因为只用到了yaw_dot和s_dot，所以只用了A和B的前四行
    // 构建当前系统状态向量
    float state[10] = {s, s_dot, yaw, yaw_dot, theta_l, theta_dot_l, theta_r, theta_dot_r, theta_b, theta_dot_b};

    // 状态方程预测：x(k+1) = Ax(k)
    // 对每个状态变量进行预测
    for (int i = 0; i < 10; i++)
    {
        // 位置预测
        pred_s += A_matrix[0][i] * 0.002f * state[i];
        // 速度预测
        pred_s_dot += A_matrix[1][i] * 0.002f * state[i];
        // 偏航角预测
        pred_yaw += A_matrix[2][i] * 0.002f * state[i];
        // 偏航角速度预测
        pred_yaw_dot += A_matrix[3][i] * 0.002f * state[i];
    }

    // 构建控制输入向量
    float input[4] = {torque_l, torque_r, torque_leg_l, torque_leg_r};

    // 控制输入对状态的影响：Bu(k)
    // 对每个控制输入进行计算
    for (int i = 0; i < 4; i++)
    {
        // 位置预测加上控制输入的影响
        pred_s += B_matrix[0][i] * 0.002f * input[i];
        // 速度预测加上控制输入的影响
        pred_s_dot += B_matrix[1][i] * 0.002f * input[i];
        // 偏航角预测加上控制输入的影响
        pred_yaw += B_matrix[2][i] * 0.002f * input[i];
        // 偏航角速度预测加上控制输入的影响
        pred_yaw_dot += B_matrix[3][i] * 0.002f * input[i];
    }

    // 计算预测的左右轮速度
    // 左轮速度 = 整体速度 - 0.251 * 偏航角速度（轮距的一半）
    pred_s_dot_l = (pred_s_dot - 0.251f * pred_yaw_dot);
    // 右轮速度 = 整体速度 + 0.251 * 偏航角速度
    pred_s_dot_r = (pred_s_dot + 0.251f * pred_yaw_dot);

    // 调整力矩输出为预测与实际之差乘以一个系数
    // 计算速度调整补偿
    T_l_adapt = (pred_s_dot_l - s_dot_l) * 1.0f;
    T_r_adapt = (pred_s_dot_r - s_dot_r) * 1.0f;
}

void WheelController::SetWheelTorque()
{
    // TODO: 需要修改
    m_WheelMotor[WMT_Left].torqueSet = -torque_l + T_l_adapt; // u = - K (x - x_d) + T_adapt (from prediction model)
    m_WheelMotor[WMT_Right].torqueSet = torque_r - T_r_adapt;

    // m_WheelMotor[WMT_Left].torqueSet = 0.0f; // u = - K (x - x_d) + T_adapt (from prediction model)
    // m_WheelMotor[WMT_Right].torqueSet = 0.0f;
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
    // 设置斜坡滤波器的目标值
    m_RampFilter_s.SetInput(_s_dot);
    // 更新滤波器
    m_RampFilter_s.Update();
    // return m_RampFilter_s.GetResult();
    // 计算速度偏差
    float delta_s_dot = m_RampFilter_s.GetResult() - s_dot;
    // 计算位置偏差
    float delta_s = target_s - s;
    // 更新目标位置，乘以s_coe系数实现平滑过渡
    target_s = s + delta_s * s_coe;
    // 返回滤波后的目标速度，加上速度偏差乘以系数
    return s_dot + delta_s_dot * s_coe;
}

float WheelController::YawFilter(float _yaw)
{
    // 使用当前设置的偏航滤波边界
    m_RampFilter_yaw.SetMargin(m_RampFilter_yaw.GetMargin());
    // 设置滤波器输入为目标偏航角
    m_RampFilter_yaw.SetInput(_yaw);
    // 更新滤波器
    m_RampFilter_yaw.Update();
    // 返回滤波后的偏航角
    return m_RampFilter_yaw.GetResult();
}

void WheelController::ResetCurrentState()
{
    // 重置速度为0
    s_dot = 0.0;
    target_s_dot = 0.0f;
    // 重置斜坡滤波器结果为0
    m_RampFilter_s.SetResult(0.0f);
    m_RampFilter_yaw.SetResult(0.0f);
    // 重置位置为0
    s = 0.0f;
    target_s = 0.0f;
    // 重置目标偏航角为当前偏航角
    target_yaw = yaw;
    // 清除PID控制器
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
    SetCurrentState(WheelStateRelax::Instance());
}

void WheelFsm::HandleInput()
{
    I6X *pI6X = I6X::Instance();

    if (pI6X->QuerySwState(I6X::RC_SW_L1, I6X::RC_SW_UP))
    {
        ChangeState(WheelStateRelax::Instance());
        return;
    }
    if (pI6X->QuerySwState(I6X::RC_SW_R1, I6X::RC_SW_M2D))
    {
        ChangeState(WheelStateInit::Instance());
        return;
    }
    if (pI6X->QuerySwState(I6X::RC_SW_R1, I6X::RC_SW_DOWN))
    {
        ChangeState(WheelStateBalance::Instance());
        return;
    }
    if (pI6X->QuerySwState(I6X::RC_SW_R2, I6X::RC_SW_DOWN))
    {
        ChangeState(WheelStateBalance::Instance());
        return;
    }

    ChangeState(WheelStateRelax::Instance());
}
