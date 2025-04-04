#include "AHRSEstimator.hpp"

uint8_t MFX[0X980];

MFXState_t *motionFX_ptr = (MFXState_t *)MFX;
MFX_knobs_t iKnobs;
MFX_input_t data_in;
MFX_output_t data_out;
float gbias[3] = {0, 0, -0.23131f};
int mfx_size = 0;

void AHRSEstimator::Init()
{
    for (int i = 0; i < 3; i++)
    {
        m_acc_lp_filter[i].SetTau(0.5f);
        m_acc_lp_filter[i].SetUpdatePeriod(5);
    }

    for (int i = 0; i < 3; i++)
    {
        m_gyro_lp_filter[i].SetTau(0.04f);
        m_gyro_lp_filter[i].SetUpdatePeriod(5);
    }
    gyro_filter[0].SetUpdatePeriod(2);
    gyro_filter[1].SetUpdatePeriod(2);
    gyro_filter[2].SetUpdatePeriod(2);

    gyro_filter[0].SetTau(10.0f);
    gyro_filter[1].SetTau(10.0f);
    gyro_filter[2].SetTau(10.0f);

    m_q[0] = m_q[1] = m_q[2] = m_q[3] = 0;

    //-----------------below are AHRS estimator----------------
    // AHRS_init(m_q, m_a, m_mag);

    //-----------------below are MotionFX----------------
    m_ax_err = 0.0f;

    mfx_size = MotionFX_GetStateSize();

    MotionFX_initialize(motionFX_ptr);
    MotionFX_enable_6X(motionFX_ptr, MFX_ENGINE_ENABLE);
    MotionFX_enable_9X(motionFX_ptr, MFX_ENGINE_DISABLE);
    MotionFX_getKnobs(motionFX_ptr, &iKnobs);

    iKnobs.LMode = 1;
    iKnobs.modx = 2;
    iKnobs.start_automatic_gbias_calculation = 0;
    iKnobs.ATime = 8.0f;
    iKnobs.FrTime = 8.0f;
    iKnobs.acc_orientation[0] = 's';
    iKnobs.acc_orientation[1] = 'e';
    iKnobs.acc_orientation[2] = 'u';
    iKnobs.gyro_orientation[0] = 's';
    iKnobs.gyro_orientation[1] = 'e';
    iKnobs.gyro_orientation[2] = 'u';

    iKnobs.gbias_acc_th_sc = (2.0f * 0.00765f);
    iKnobs.gbias_gyro_th_sc = (2.0f * 0.02f);

    iKnobs.output_type = MFX_ENGINE_OUTPUT_ENU;

    MotionFX_setKnobs(motionFX_ptr, &iKnobs);
    MotionFX_setGbias(motionFX_ptr, gbias);

    counter = 0;
    m_ax_err = 0.0f;
}

float r, p, y = 0.0f;
uint32_t cal_count = 5000;

static float last_yaw = 0.0f;
static float abs_yaw = 0.0f;

void AHRSEstimator::Update()
{
    // get accel
    tmp_m_a[0] = m_pImuOnChip->GetAccY() - 0.112596028f;
    tmp_m_a[1] = -m_pImuOnChip->GetAccX() + 0.0902228802f;
    tmp_m_a[2] = m_pImuOnChip->GetAccZ() - 9.83948135f + 9.81f;

    // tmp_m_a[0] = m_pImuOnChip->GetAccY() - 0.112596028f + 0.111555822f;
    // tmp_m_a[1] = -m_pImuOnChip->GetAccX() + 0.0902228802f + 0.080293946f;
    // tmp_m_a[2] = m_pImuOnChip->GetAccZ() - 9.83948135f + 9.81f - 9.55834198f + 9.81f;

    counter++;

    for (int i = 0; i < 3; i++)
    {
        a_sum[i] += tmp_m_a[i];
        a_avg[i] = a_sum[i] / counter;
    }

    // accel filter
    accel_fliter_1[0] = accel_fliter_2[0];
    accel_fliter_2[0] = accel_fliter_3[0];

    accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + tmp_m_a[0] * fliter_num[2];

    accel_fliter_1[1] = accel_fliter_2[1];
    accel_fliter_2[1] = accel_fliter_3[1];

    accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + tmp_m_a[1] * fliter_num[2];

    accel_fliter_1[2] = accel_fliter_2[2];
    accel_fliter_2[2] = accel_fliter_3[2];

    accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + tmp_m_a[2] * fliter_num[2];

    // angular speed update

    gyro_filter[0].SetInput(m_pImuOnChip->GetWy());
    gyro_filter[1].SetInput(-m_pImuOnChip->GetWx());
    gyro_filter[2].SetInput(m_pImuOnChip->GetWz());

    gyro_filter[0].Update();
    gyro_filter[1].Update();
    gyro_filter[2].Update(); //?

    tmp_m_gyro[0] = m_pImuOnChip->GetWy() + 0.00203253538f;  // - 0.00017f;
    tmp_m_gyro[1] = -m_pImuOnChip->GetWx() + 0.00397365214f; // + 0.003f;
    tmp_m_gyro[2] = m_pImuOnChip->GetWz() - 0.000891535659f; // - 0.00038f;//ID=4

    // tmp_m_gyro[0] = m_pImuOnChip->GetWy() + 0.00203253538f;  // - 0.00017f;
    // tmp_m_gyro[1] = -m_pImuOnChip->GetWx() + 0.00397365214f - 0.00418124394f; // + 0.003f;
    // tmp_m_gyro[2] = m_pImuOnChip->GetWz() - 0.000891535659f + 0.0012986915f;  // - 0.00038f;//ID=4

    for (int i = 0; i < 3; i++)
    {
        gyro_sum[i] += tmp_m_gyro[i];
        gyro_avg[i] = gyro_sum[i] / counter;
    }

    for (int i = 0; i < 3; i++)
    {
        m_gyro_lp_filter[i].SetInput(tmp_m_gyro[i]);
        m_gyro_lp_filter[i].Update();
        tmp_filtered_m_gyro[i] = m_gyro_lp_filter[i].GetResult();
    }

    //  mag update
    //    tmp_m_mag[0] = m_pIST8310->m_mag_x;
    //    tmp_m_mag[1] = m_pIST8310->m_mag_y;
    //    tmp_m_mag[2] = m_pIST8310->m_mag_z;

    //  Installation Pose Fix
    for (int i = 0; i < 3; i++)
    {
        m_gyro[i] = installSpinMatrix[i][0] * tmp_m_gyro[0] + installSpinMatrix[i][1] * tmp_m_gyro[1] + installSpinMatrix[i][2] * tmp_m_gyro[2];
        m_a[i] = installSpinMatrix[i][0] * accel_fliter_3[0] + installSpinMatrix[i][1] * accel_fliter_3[1] + installSpinMatrix[i][2] * accel_fliter_3[2];
        m_mag[i] = installSpinMatrix[i][0] * tmp_m_mag[0] + installSpinMatrix[i][1] * tmp_m_mag[1] + installSpinMatrix[i][2] * tmp_m_mag[2];
    }

    //-----------------below are MotionFX----------------
    data_in.acc[0] = m_a[0] * 0.102f;
    data_in.acc[1] = m_a[1] * 0.102f;
    data_in.acc[2] = m_a[2] * 0.102f;

    data_in.gyro[0] = m_gyro[0] * 57.29f;
    data_in.gyro[1] = m_gyro[1] * 57.29f;
    data_in.gyro[2] = m_gyro[2] * 57.29f;

    float t = 0.002f;

    MotionFX_propagate(motionFX_ptr, &data_out, &data_in, &t);
    MotionFX_update(motionFX_ptr, &data_out, &data_in, &t, NULL);

    r = data_out.rotation[2];
    p = data_out.rotation[1];
    y = data_out.rotation[0];

    float delta_yaw = y - last_yaw;

    if (delta_yaw < -180.0f)
    {
        delta_yaw = 360.0f + delta_yaw;
    }
    else if (delta_yaw > 180.0f)
    {
        delta_yaw = 360.0f - delta_yaw;
    }

    abs_yaw += delta_yaw;

    last_yaw = y;

    m_Euler[0] = -abs_yaw * 0.0174532925f;
    m_Euler[1] = -p * 0.0174532925f;
    m_Euler[2] = r * 0.0174532925f;

    motion_acc[0] = data_out.linear_acceleration[0] + data_out.gravity[0];
    motion_acc[1] = data_out.linear_acceleration[1] + data_out.gravity[1];
    motion_acc[2] = data_out.linear_acceleration[2] + data_out.gravity[2];

    //------------below are AHRS estimator----------------
    // // update quaternion
    // AHRS_update(m_q, 0.005, m_gyro, m_a, m_mag); // see main : gyro updated every 5 ms.

    // // calculate AHRS from quaternion
    // m_Euler[0] = get_RPY_yaw(m_q);
    // m_Euler[1] = get_RPY_pitch(m_q);
    // m_Euler[2] = get_RPY_roll(m_q);

    // float delta_yaw = m_Euler[0] - m_lastYaw;

    // if (delta_yaw < -3.14f)
    // {
    //     delta_yaw = 6.28f + delta_yaw;
    // }
    // else if (delta_yaw > 3.14f)
    // {
    //     delta_yaw = 6.28f - delta_yaw;
    // }

    // // convertAccel(m_Euler[2],m_Euler[1],m_Euler[0],new_m_a,accelWorld);//szh

    // m_AbsYaw += delta_yaw;

    // m_lastYaw = m_Euler[0];
}

float get_RPY_yaw(float m_q[4])
{

    float q0 = m_q[0];
    float q1 = m_q[1];
    float q2 = m_q[2];
    float q3 = m_q[3];

    return atan2f(2.0f * (q0 * q3 - q1 * q2), 2.0f * (q0 * q0 + q2 * q2) - 1.0f);
}

float get_RPY_pitch(float m_q[4])
{

    float q0 = m_q[0];
    float q1 = m_q[1];
    float q2 = m_q[2];
    float q3 = m_q[3];

    return atan2f(2.0f * (q0 * q2 - q1 * q3), 2.0f * (q0 * q0 + q3 * q3) - 1.0f);
}

float get_RPY_roll(float m_q[4])
{

    float q0 = m_q[0];
    float q1 = m_q[1];
    float q2 = m_q[2];
    float q3 = m_q[3];

    return asinf(2.0f * q0 * q1 + 2.0f * q2 * q3);
}

void AHRSEstimator::convertAccel(float roll, float pitch, float yaw, float accel[3], float accelWorld[3])
{
    // // 将roll、pitch、yaw转换为弧度
    // float roll_rad = roll;
    // float pitch_rad = pitch;
    // float yaw_rad = yaw;

    // // 计算旋转矩阵
    // float R[3][3] =
    //     {
    //         {arm_cos_f32(yaw_rad) * arm_cos_f32(pitch_rad), arm_cos_f32(yaw_rad) * arm_sin_f32(pitch_rad) * arm_sin_f32(roll_rad) - arm_sin_f32(yaw_rad) * arm_cos_f32(roll_rad), arm_cos_f32(yaw_rad) * arm_sin_f32(pitch_rad) * arm_cos_f32(roll_rad) + arm_sin_f32(yaw_rad) * arm_sin_f32(roll_rad)},
    //         {arm_sin_f32(yaw_rad) * arm_cos_f32(pitch_rad), arm_sin_f32(yaw_rad) * arm_sin_f32(pitch_rad) * arm_sin_f32(roll_rad) + arm_cos_f32(yaw_rad) * arm_cos_f32(roll_rad), arm_sin_f32(yaw_rad) * arm_sin_f32(pitch_rad) * arm_cos_f32(roll_rad) - arm_cos_f32(yaw_rad) * arm_sin_f32(roll_rad)},
    //         {-arm_sin_f32(pitch_rad), arm_cos_f32(pitch_rad) * arm_sin_f32(roll_rad), arm_cos_f32(pitch_rad) * arm_cos_f32(roll_rad)}};

    // // 计算世界坐标系下的加速度
    // for (int i = 0; i < 3; i++)
    // {
    //     accelWorld[i] = 0;
    //     for (int j = 0; j < 3; j++)
    //     {
    //         accelWorld[i] += R[i][j] * accel[j];
    //     }
    // }
}
