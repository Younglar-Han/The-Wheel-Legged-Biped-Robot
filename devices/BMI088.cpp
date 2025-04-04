#include "BMI088.hpp"
#include "BMI088reg.h"
#include "Math.hpp"
#include "math.h"

void BMI088::BspInit()
{
    BMI088_init();
}

void BMI088::Update()
{
    BMI088_read(gyro, accel, &temp);
    m_ax = accel[0];
    m_ay = accel[1];
    m_az = accel[2];
    m_wx = gyro[0];
    m_wy = gyro[1];
    m_wz = gyro[2];
    temperature = temp;
    m_acc = sqrtf(m_ax * m_ax + m_ay * m_ay + m_az * m_az);
    m_acc_angle_x = atan2f(m_ay, sqrtf(m_ax * m_ax + m_az * m_az)) * 180.0f / PI;
    m_acc_angle_y = atan2f(-m_ax, sqrtf(m_ay * m_ay + m_az * m_az)) * 180.0f / PI;
    m_acc_angle_z = atan2f(m_az, sqrtf(m_ax * m_ax + m_ay * m_ay)) * 180.0f / PI;
}
