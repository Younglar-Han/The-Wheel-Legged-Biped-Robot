#ifndef WHEELSTATEREMOTE_HPP
#define WHEELSTATEREMOTE_HPP

#include "StateMachine.hpp"
#include "WheelController.hpp"
#include "Dr16.hpp"
#include "AHRSEstimator.hpp"

class WheelStateRemote : public State<WheelController>
{
private:
    Dr16 *m_pDr16;
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
