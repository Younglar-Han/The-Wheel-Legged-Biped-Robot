#ifndef MATH_HPP
#define MATH_HPP

#include "stm32f4xx.h"

class Math
{
public:
    static const float Pi;
    static const float PiX2;

    static float LoopFloatConstrain(float input, float minValue, float maxValue)
    {
        if (maxValue < minValue)
        {
            return input;
        }

        if (input > maxValue)
        {
            float len = maxValue - minValue;
            while (input > maxValue)
            {
                input -= len;
            }
        }
        else if (input < minValue)
        {
            float len = maxValue - minValue;
            while (input < minValue)
            {
                input += len;
            }
        }
        return input;
    }

    static float LimitMax(float _input, float _max)
    {
        if(_input > _max)
        {
            return _max;
        }
        else if(_input < -_max)
        {
            return -_max;
        }
        return _input;
    }

    static float FloatConstrain(float Input, float minValue, float maxValue)
    {
        if (maxValue < minValue)
        {
            return Input;
        }

        if (Input > maxValue)
        {
            Input = maxValue;
        }
        else if (Input < minValue)
        {
            Input = minValue;
        }
        return Input;
    }

    static uint32_t ConvertToFixed(float _inNum, float _inMin, float _inPrecision)
    {
        return (uint32_t)((_inNum - _inMin) / _inPrecision);
    }

    static float ConvertFromFixed(uint32_t _inNum, float _inMin, float _inPrecision)
    {
        return (float)(_inNum) * _inPrecision + _inMin;
    }

    static float invSqrt(float x)
    {
        float xhalf = 0.5f * x;
        int i = *(int*)&x; // get bits for floating value
        i = 0x5f3759df - (i >> 1); // gives initial guess
        x = *(float*)&i; // convert bits back to float
        x = x * (1.5f - xhalf * x * x); // Newton step
        return 1.0f/x;
    }

    static float abs(float x)
    {
        return x > 0.0f ? x : -1.0f * x;
    }
};

#endif
