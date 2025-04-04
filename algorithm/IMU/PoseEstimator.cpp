#include "PoseEstimator.hpp"
#include "Math.hpp"

#include "math.h"

void PoseEstimator::Update()
{
    float q2q3 = m_Q[1] * m_Q[2];
    float q1q4 = m_Q[0] * m_Q[3];
    float q1q1 = m_Q[0] * m_Q[0];
    float q2q2 = m_Q[1] * m_Q[1];

    float q2q4 = m_Q[1] * m_Q[3];
    float q1q3 = m_Q[0] * m_Q[2];

    float q3q4 = m_Q[2] * m_Q[3];
    float q1q2 = m_Q[0] * m_Q[1];
    float q4q4 = m_Q[3] * m_Q[3];

    // Yaw
    m_Euler[0] = atan2(2.0f * (q2q3 - q1q4), 2.0f * (q1q1 + q2q2) - 1.0f);

    // Pitch
    m_Euler[1] = -asinf(2.0f * (q2q4 + q1q3));

    // Roll
    m_Euler[2] = atan2(2.0f * (q3q4 - q1q2), 2.0f * (q1q1 + q4q4) - 1.0f);

    m_Euler[2] -=3.14159f;

    if(m_Euler[2]<-3.14159f)
    {
        m_Euler[2] += 6.283f;
    }
}
