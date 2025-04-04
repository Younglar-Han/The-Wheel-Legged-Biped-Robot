#ifndef MATH_RAMP_FILTER_H_
#define MATH_RAMP_FILTER_H_

#include <stdint.h>
#include <math.h>
#include "Math.hpp"
#include "RobotEngine.hpp"

class RampFilter
{
private:
    float m_Input;
    float m_Out;
    float m_Margin;

public:
    void SetInput(float in){m_Input = in;}
    void SetMargin(float margin){m_Margin = margin;}
    void SetResult(float out){m_Out = out;}
    
    float GetResult(){return m_Out;}
    float GetMargin(){return m_Margin;}

    void Init()
    {
        m_Margin = 0.0f;
    }

    void Update()
    {
        m_Out += Math::FloatConstrain(m_Input - m_Out, -m_Margin, m_Margin);
    }

    void Clear()
    {
        m_Out = 0;
        m_Input = 0;
    }

};

#endif /* MATH_RAMP_FILTER_H_ */
