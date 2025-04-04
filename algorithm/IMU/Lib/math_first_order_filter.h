/**
  ****************************************************************
  * @file    math_first_order_filter.h
  * @author  TofuLemon(5thzhengjiarandom@gmail.com)
  * @date    2020-Jan-16
  ****************************************************************
  */

#ifndef MATH_FIRST_ORDER_FILTER_H_
#define MATH_FIRST_ORDER_FILTER_H_

#include <stdint.h>
#include <math.h>
#include "RobotEngine.hpp"

class FirstOrderFilter
{
private:
    float m_Input;
    float m_Out;
    float m_Tau;
    uint32_t m_LastUpdateTick;
    float m_UpdatePeriod;

public:
    void SetInput(float in){m_Input = in;}
    void SetTau(float tau){m_Tau = tau;}
    void SetResult(float out){m_Out = out;}
    void SetUpdatePeriod(float t){m_UpdatePeriod = t * 0.001f;}/* t in ms */
    
    float GetResult(){return m_Out;}
    float GetTau(){return m_Tau;}
    float GetUpdatePeriod(){return m_UpdatePeriod;}

    void Init()
    {
        m_UpdatePeriod = 0.0f;
    }

    void Update()
    {
        float a = m_UpdatePeriod / (m_Tau);
        m_Out = (1 - a) * m_Out + a * m_Input;
    }

    void Clear()
    {
        m_Out = 0;
        m_Input = 0;
    }

};

#endif /* MATH_FIRST_ORDER_FILTER_H_ */
