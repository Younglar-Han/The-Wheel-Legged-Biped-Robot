#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include "stm32f4xx.h"
#include "BMI088.hpp"
#include "IST8310.hpp"
#include "math_first_order_filter.h"

class PoseEstimator
{
protected:
    float m_Q[4];
    float m_Euler[3];
	float m_lastYaw;
	float m_AbsYaw;

    bool m_have_IST8310;

    BMI088* m_pImuOnChip;
    IST8310* m_pIST8310;
public:
    PoseEstimator():
        m_Q{0.0f},
        m_Euler{0.0f}, 
        m_have_IST8310(false),
		m_pImuOnChip(nullptr)
    {}

    virtual void Init() = 0;

    virtual void Update();

    void SetImu(BMI088* _imu){m_pImuOnChip = _imu;}
    void SetIST(IST8310* _imu)
    {
        m_pIST8310 = _imu;
        m_have_IST8310 = true;
    }

    IST8310* GetIST8310(){return m_pIST8310;}
    BMI088* GetImu(){ return m_pImuOnChip; }

    float GetQuaternion(uint8_t _index){ if(_index > 3) return 0.0f; return m_Q[_index]; }
    float GetEuler(uint8_t _index){ if(_index > 2) return 0.0f; return m_Euler[_index]; }

    float GetRoll(){ return GetEuler(2); }
    float GetPitch(){ return GetEuler(1); }
    float GetYaw(){ return GetEuler(0); }
	float GetAbsYaw(){ return m_AbsYaw; }
};

#endif
