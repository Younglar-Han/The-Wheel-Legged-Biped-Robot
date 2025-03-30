#include "BMI088.hpp"
#include "BMI088reg.h"
#include "Math.hpp"
#include "math.h"

void BMI088::BspInit()
{
    bsp_bmi088_init();
		bsp_bmi088_clk_init(1000,168);
    TemperatruePid.Init(); 
    TemperatruePid.kp = 1000;
    TemperatruePid.ki = 100;
    TemperatruePid.kd = 0;
    TemperatruePid.maxOut = 4999;
    TemperatruePid.maxIOut = 3000;
}

void BMI088::ReadRegister(uint8_t _address, uint8_t* _pData, uint32_t _len, BMI088Cs cs)
{
    _address |= 0x80;
    
    if(cs == CS_Acc)
    {
        bsp_select_bmi088_acc();
    }
    else if(cs == CS_Gyro)
    {
        bsp_select_bmi088_gyro();
    }

    bsp_spi_send(&_address,1,50);
    uint32_t _timeout = _len * 50;
    if(cs == CS_Acc) bsp_spi_send(&_address,1,50);
    bsp_spi_receive(_pData,_len,_timeout);
    
    if(cs == CS_Acc)
    {
        bsp_deselect_bmi088_acc();
    }
    else if(cs == CS_Gyro)
    {
        bsp_deselect_bmi088_gyro();
    }
}

void BMI088::WriteRegister(uint8_t _address, uint8_t* _pData, uint32_t _len, BMI088Cs cs)
{
    if(cs == CS_Acc)
    {
        bsp_select_bmi088_acc();
    }
    else if(cs == CS_Gyro)
    {
        bsp_select_bmi088_gyro();
    }

    uint8_t temp_buffer[16];
    temp_buffer[0] = _address;
    for (int i = 1; i <= _len; ++i)
    {
        temp_buffer[i] = _pData[i - 1];
    }
    
	bsp_spi_send(temp_buffer, 1 + _len, 50);

    if(cs == CS_Acc)
    {
        bsp_deselect_bmi088_acc();
    }
    else if(cs == CS_Gyro)
    {
        bsp_deselect_bmi088_gyro();
    }
}

static int acc_reset_flag = 0;
static int gyro_reset_flag = 0;

static uint8_t acc_data[6] = {0};
static uint8_t gyro_data[6] = {0};
static uint8_t tmp_data[2] = {0};

void BMI088::ReadData()
{
	// for(int i = 0;i<6;i++)
	// {
		ReadRegister(BMI088_ACCEL_XOUT_L,&acc_data[0],6,CS_Acc);
	// }
    
    int16_t bmi088_raw_temp;

    bmi088_raw_temp = (int16_t)((acc_data[1]) << 8) | acc_data[0];
	m_ax = (float)bmi088_raw_temp * 0.0008974358974f;

    bmi088_raw_temp = (int16_t)((acc_data[3]) << 8) | acc_data[2];
	m_ay = (float)bmi088_raw_temp * 0.0008974358974f;

    bmi088_raw_temp = (int16_t)((acc_data[5]) << 8) | acc_data[4];
	m_az = (float)bmi088_raw_temp * 0.0008974358974f;

    m_acc = Math::invSqrt(m_ax*m_ax+m_ay*m_ay+m_az*m_az);
	
	m_acc_angle_x = asinf(m_ax/m_acc)*0.31830989f*180.0f;
	m_acc_angle_y = asinf(m_ay/m_acc)*0.31830989f*180.0f;
	m_acc_angle_z = asinf(m_az/m_acc)*0.31830989f*180.0f;

    for(int i = 0;i<6;i++)
	{
		ReadRegister(BMI088_GYRO_X_L+i,&gyro_data[i],1,CS_Gyro);
	}

    bmi088_raw_temp = (int16_t)((gyro_data[1]) << 8) | gyro_data[0];
    m_wx = (float)bmi088_raw_temp * 0.00106526443603169529841533860381f;

    bmi088_raw_temp = (int16_t)((gyro_data[3]) << 8) | gyro_data[2];
    m_wy = (float)bmi088_raw_temp * 0.00106526443603169529841533860381f;

    bmi088_raw_temp = (int16_t)((gyro_data[5]) << 8) | gyro_data[4];
    m_wz = (float)bmi088_raw_temp * 0.00106526443603169529841533860381f;

    if(time_count >= 200 || time_count == 0)
    {

        // for(int i=0;i<2;i++)
        // {
            ReadRegister(BMI088_TEMP_M,&tmp_data[0],2,CS_Acc);
        // }

        bmi088_raw_temp = (int16_t)((tmp_data[0] << 3) | (tmp_data[1] >> 5));
        temperature = bmi088_raw_temp * 0.125f + 23.0f;

        TemperatruePid.ref = 42.0f;
        TemperatruePid.fdb = temperature;
        TemperatruePid.UpdateResult();
        uint16_t pwmresult = (uint16_t)(TemperatruePid.result);
        bsp_pwm_set(pwmresult);//设置占空比
        time_count = 0;

        temperature = bmi088_raw_temp * 0.125f + 23.0f;
    }

    time_count++;
}

int BMI088::AccInit()
{
    uint8_t data;
    if(acc_reset_flag == 0)
    {
        ReadRegister(BMI088_ACC_CHIP_ID,&data,1,CS_Acc);

        data = BMI088_ACC_SOFTRESET_VALUE;
        WriteRegister(BMI088_ACC_SOFTRESET,&data,1,CS_Acc);
		acc_reset_flag = 1;
    }
    else if(acc_reset_flag == 1)
    {
        time_count++;
        if(time_count > 100)
        {
            acc_reset_flag = 2;
        }
    }
    else if(acc_reset_flag == 2)
    {
        ReadRegister(BMI088_ACC_CHIP_ID,&data,1,CS_Acc);
        if (data != BMI088_ACC_CHIP_ID_VALUE)
        {
            return BMI088_NO_SENSOR;
        }
	
		data = BMI088_ACC_ENABLE_ACC_ON;
		WriteRegister(BMI088_ACC_PWR_CTRL,&data,1,CS_Acc);
		
		acc_reset_flag = 3;
    }
	else if(acc_reset_flag == 3)
    {
        time_count++;
        if(time_count > 100)
        {
            acc_reset_flag = 4;
        }
		}
	else if(acc_reset_flag ==4)
	{
		uint8_t init_reg_array[3] = {BMI088_ACC_PWR_CONF,BMI088_ACC_CONF,
            BMI088_ACC_RANGE};

        uint8_t init_data_array[3] = {BMI088_ACC_PWR_ACTIVE_MODE,BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set,
            BMI088_ACC_RANGE_3G};

        for(int i = 0;i < 3;i++)
        {
            WriteRegister(init_reg_array[i],&init_data_array[i],1,CS_Acc);
			
			for(int j =0;j<168000;j++)
			{
			}

        }    

		return 1;
	}
	return 0;
}

int BMI088::GyroInit()
{
    uint8_t data;
    if(gyro_reset_flag == 0)
    {
        ReadRegister(BMI088_GYRO_CHIP_ID,&data,1,CS_Gyro);

        data = BMI088_GYRO_SOFTRESET_VALUE;
        WriteRegister(BMI088_GYRO_SOFTRESET,&data,1,CS_Gyro);
		gyro_reset_flag = 1;
    }
    else if(gyro_reset_flag == 1)
    {
        time_count++;
        if(time_count > 100)
        {
            gyro_reset_flag = 2;
        }
    }
    else if(gyro_reset_flag == 2)
    {
        ReadRegister(BMI088_GYRO_CHIP_ID,&data,1,CS_Gyro);
        if (data != BMI088_GYRO_CHIP_ID_VALUE)
        {
            return BMI088_NO_SENSOR;
        }
	
		data = BMI088_ACC_ENABLE_ACC_ON;
		WriteRegister(BMI088_ACC_PWR_CTRL,&data,1,CS_Gyro);
		
		gyro_reset_flag = 3;
    }
	else if(gyro_reset_flag == 3)
    {
        time_count++;
        if(time_count > 100)
        {
            gyro_reset_flag = 4;
        }
		}
	else if(gyro_reset_flag ==4)
	{
		uint8_t init_reg_array[4] = {BMI088_GYRO_RANGE,BMI088_GYRO_BANDWIDTH,
            BMI088_GYRO_LPM1,BMI088_GYRO_CTRL};

        uint8_t init_data_array[4] = {BMI088_GYRO_2000,BMI088_GYRO_1000_116_HZ| BMI088_GYRO_BANDWIDTH_MUST_Set,
            BMI088_GYRO_NORMAL_MODE,BMI088_DRDY_ON};

        for(int i = 0;i < 4;i++)
        {
            WriteRegister(init_reg_array[i],&init_data_array[i],1,CS_Gyro);
			
			for(int j =0;j<168000;j++)
			{
			}

        }    

		return 1;
	}
	return 0;
}
