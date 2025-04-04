#ifndef TESTBOT_HPP
#define TESTBOT_HPP

#include "RobotEngine.hpp"
#include "LegController.hpp"
#include "WheelController.hpp"


class Testbot : public RobotEngine
{
private:
    LegController legController;
    WheelController wheelController;
public:
    Testbot() : RobotEngine(this)
    {
    }
};

#endif
