#ifndef PID_HPP
#define PID_HPP

#include "Math.hpp"
#include "Time.hpp"

class Pid
{
public:
    enum PidModeType
    {
        PID_POSITION = 0,
        PID_DELTA,
        PID_DERIVATIVE_FILTERED,
    };

    PidModeType mode;

    float kp;
    float ki;
    float kd;

    float maxOut;
    float maxIOut;

    float ref;
    float fdb;

    float result;
    float pResult;
    float iResult;
    float dResult;

    float dBuf[3];
    float err[3];

    bool firstupdate = 1;
    bool isPitchSpd;
    bool isPitch;
    bool isYaw;
    float err_lowsample[3];
    float gama = 1.0f;
    float last_fdb = 0.0f;

    uint32_t last_updateTick;

    Pid();
    Pid(PidModeType mode, float p, float i, float d, float max, float imax);

    void Init();
    void UpdateResult();
    void Clear();

    void SetParams(float p, float i, float d, float max, float imax);

    void UpdateIResult(float _err){iResult += ki * _err;iResult = Math::LimitMax(iResult, maxIOut);}
};


#endif
