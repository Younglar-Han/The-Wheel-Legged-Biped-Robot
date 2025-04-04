#include "MahonyEstimator.hpp"
#include "MahonyAHRS.h"

void MahonyEstimator::Init()
{
}

void MahonyEstimator::Update()
{
    if(m_have_IST8310)
    {
        MahonyAHRSupdate(m_pImuOnChip->GetWy(),
                            -m_pImuOnChip->GetWx(),
                            m_pImuOnChip->GetWz(),
                            m_pImuOnChip->GetAccY(),
                            -m_pImuOnChip->GetAccX(),
                            m_pImuOnChip->GetAccZ(),
                            m_pIST8310->m_mag_x,
                            m_pIST8310->m_mag_y,
                            m_pIST8310->m_mag_z
                            );
		
		// MahonyAHRSupdateIMU(m_pImuOnChip->GetWx(),
        //                m_pImuOnChip->GetWy(),
        //                m_pImuOnChip->GetWz(),
        //                -m_pImuOnChip->GetAccY(),
        //                -m_pImuOnChip->GetAccZ(),
        //                -m_pImuOnChip->GetAccX()
        //                );
    }
    else{
        MahonyAHRSupdateIMU(m_pImuOnChip->GetWx(),
                        m_pImuOnChip->GetWy(),
                        m_pImuOnChip->GetWz(),
                        m_pImuOnChip->GetAccX(),
                        m_pImuOnChip->GetAccY(),
                        m_pImuOnChip->GetAccZ()
                        );
    }
    
    m_Q[0] = q0;
    m_Q[1] = q1;
    m_Q[2] = q2;
    m_Q[3] = q3;

    PoseEstimator::Update();
}
