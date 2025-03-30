#include "Pid.hpp"

Pid::Pid() : mode(PID_POSITION),
             kp(0.0f),
             ki(0.0f),
             kd(0.0f),
             maxOut(0.0f),
             maxIOut(0.0f),
             isPitchSpd(false),
             isPitch(false),
             isYaw(false)
{
    dBuf[0] = dBuf[1] = dBuf[2] = 0.0f;
    err[0] = err[1] = err[2] = 0.0f;
    last_updateTick = Time::GetTick();
}

Pid::Pid(PidModeType mode = PID_POSITION,
         float p = 0.0f,
         float i = 0.0f,
         float d = 0.0f,
         float max = 0.0f,
         float imax = 0.0f) : mode(mode),
                              kp(p),
                              ki(i),
                              kd(d),
                              maxOut(max),
                              maxIOut(imax),
                              isPitchSpd(false),
                              isPitch(false)
{
    last_updateTick = Time::GetTick();
}

void Pid::Init(void)
{
    Clear();
}

void Pid::UpdateResult(void)
{
    err[2] = err[1];
    err[1] = err[0];
    if (mode == PID_POSITION)
    {

        err[0] = ref - fdb;

        pResult = kp * err[0];

        iResult += ki * err[0];
        dBuf[2] = dBuf[1];
        dBuf[1] = dBuf[0];

        if (Time::GetTick() - last_updateTick >= 10 || firstupdate)
        {

            err_lowsample[2] = err_lowsample[1];
            err_lowsample[1] = err_lowsample[0];
            err_lowsample[0] = ref - fdb;
            last_updateTick = Time::GetTick();
            firstupdate = 0;
        }

        dBuf[0] = err_lowsample[0] - err_lowsample[1];

        dResult = kd * dBuf[0];
        iResult = Math::LimitMax(iResult, maxIOut);
    }
    else if (mode == PID_DELTA)
    {
        pResult = kp * (err[0] - err[1]);
        iResult = ki * err[0];
        dBuf[2] = dBuf[1];
        dBuf[1] = dBuf[0];
        dBuf[0] = (err[0] - 2.0f * err[1] + err[2]);
        dResult = kd * dBuf[0];
    }
    else if (mode == PID_DERIVATIVE_FILTERED)
    {
        err[0] = ref - fdb;

        pResult = kp * err[0];

        if (err[0] > 0.1f || err[0] < -0.1f)
            iResult += ki * err[0];

        dBuf[2] = dBuf[1];
        dBuf[1] = dBuf[0];
        dBuf[0] = err[0] - err[1];

        dResult = kd * dBuf[0] * (1 - gama) + gama * dResult;
        iResult = Math::LimitMax(iResult, maxIOut);
    }

    result = pResult + iResult + dResult;
    result = Math::LimitMax(result, maxOut);
}

void Pid::Clear(void)
{
    dBuf[0] = dBuf[1] = dBuf[2] = 0.0f;
    err[0] = err[1] = err[2] = 0.0f;
    err_lowsample[0] = err_lowsample[1] = err_lowsample[2] = 0.0f;
    pResult = iResult = dResult = result = 0.0f;
    ref = fdb = 0.0f;
    iResult = 0.0f;
    pResult = 0.0f;
    dResult = 0.0f;
    firstupdate = 1;
}

void Pid::SetParams(float p = 0.0f,
                    float i = 0.0f,
                    float d = 0.0f,
                    float max = 0.0f,
                    float imax = 0.0f)
{
    kp = p;
    ki = i;
    kd = d;
    maxOut = max;
    maxIOut = imax;
}
