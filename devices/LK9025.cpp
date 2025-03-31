#include "LK9025.hpp"

LK9025::LK9025() : Motor(ECT_LK9025)
{
    pFeedback = &sensorFeedBack;
    controlMode = Motor::RELAX_MODE;
    positionSet = 0.0f;
    speedSet = 0.0f;
    accelerationSet = 0.0f;
    currentSet = 0.0f;
    SetDefaultTicksToUpdate(2);
}

void LK9025::Update()
{
    if (controlMode == RELAX_MODE)
    {
        currentSet = 0.0f;
        LKCanMotorCommander::Instance()->Add2Buffer(can, canId, (int16_t)currentSet);
        return;
    }

    if (controlMode == CUR_MODE)
    {
        // currentSet = torqueSet / 0.32f / 32.0f * 2000.0f;
        currentSet = torqueSet * 392.78f;
		currentSet = Math::FloatConstrain(currentSet, -2000, 2000); // max: 2000
        LKCanMotorCommander::Instance()->Add2Buffer(can, canId, (int16_t)currentSet);
        return;
    }

    float _spdFdb = pFeedback->speedFdb;

    if (controlMode == POS_MODE)
    {
        pidPosition.ref = positionSet;

        pidPosition.fdb = sensorFeedBack.positionFdb;

        if (positionSet - pidPosition.fdb > PI)
        {
            pidPosition.ref -= 2 * PI;
        }
        else if (positionSet - pidPosition.fdb < -PI)
        {
            pidPosition.ref += 2 * PI;
        }

        pidPosition.UpdateResult();

        speedSet = pidPosition.result;
    }
	pidSpeed.ref = speedSet;
    pidSpeed.fdb = _spdFdb;

    pidSpeed.UpdateResult();

    currentSet = pidSpeed.result;

    LKCanMotorCommander::Instance()->Add2Buffer(can, canId, (int16_t)currentSet);
}
