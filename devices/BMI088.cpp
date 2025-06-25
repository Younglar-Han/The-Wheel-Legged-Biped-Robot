#include "BMI088.hpp"
#include "BMI088reg.h"
#include "Math.hpp"
#include "math.h"

void BMI088::BspInit()
{
    BMI088_init();
    TemperatruePid.Init(); 
    TemperatruePid.kp = 1000;
    TemperatruePid.ki = 100;
    TemperatruePid.kd = 0;
    TemperatruePid.maxOut = 4999;
    TemperatruePid.maxIOut = 3000;
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

    TemperatruePid.ref = 42.0f;
    TemperatruePid.fdb = temperature;
    TemperatruePid.UpdateResult();
    uint16_t pwmresult = (TemperatruePid.result < 0) ? 0 : (uint16_t)TemperatruePid.result;
    // 确保float强转uint16_t时不出现负数转为补码导致IMU烧毁
    bsp_pwm_set(pwmresult);//设置占空比
}
