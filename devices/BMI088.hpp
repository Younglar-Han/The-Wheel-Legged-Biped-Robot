#ifndef BMI088_HPP
#define BMI088_HPP

#include "bsp_bmi088.h"
#include "Pid.hpp"
#include "bsp_heatImu.h"
#include "common_define.h"

class BMI088
{
public:
    static BMI088* Instance()
    {
        static BMI088 instance;
        return &instance;
    }

    ~BMI088(){}

    enum BMI088Cs
    {
        CS_Acc,
        CS_Gyro
    };

    Pid TemperatruePid;
    uint32_t time_count = 0;

    void BspInit();
    void Update();

    float GetAccX(){return m_ax;}
    float GetAccY(){return m_ay;}
    float GetAccZ(){return m_az;}
    float GetAccAngleX(){return m_acc_angle_x;}
    float GetAccAngleY(){return m_acc_angle_y;}
    float GetAccAngleZ(){return m_acc_angle_z;}
	float GetWx(){return m_wx;}
	float GetWy(){return m_wy;}
	float GetWz(){return m_wz;}
    float GetTemperature(){return temperature;}
    void setTemperature(float _temperature){temperature = _temperature;}
private:
    BMI088():m_send_buffer_count(0),m_time_ms_count(0){}
    uint8_t m_send_buffer[16];

    uint8_t m_send_buffer_count;
    int m_time_ms_count;

    float m_ax;
    float m_ay;
    float m_az;
    float m_acc;
    float m_acc_angle_x;
    float m_acc_angle_y;
    float m_acc_angle_z;
		
	float m_wx;
	float m_wy;
	float m_wz;
    float temperature;

    fp32 gyro[3], accel[3], temp;
};

#endif
