#ifndef WHEELSTATEREMOTE_HPP
#define WHEELSTATEREMOTE_HPP

#include "StateMachine.hpp"
#include "WheelController.hpp"
#include "I6X.hpp"
#include "AHRSEstimator.hpp"

class WheelStateRemote : public State<WheelController>
{
private:
    I6X *m_pI6X;
    AHRSEstimator* m_IMU;

    float target_yaw;

public:
    virtual void Init(WheelController* pOwner);
    virtual void Enter(WheelController* pOwner);
    virtual void Execute(WheelController* pOwner);
    virtual void Exit(WheelController* pOwner);

    static WheelStateRemote* Instance()
    {
        static WheelStateRemote instance;
        return &instance;
    }
};

#endif
