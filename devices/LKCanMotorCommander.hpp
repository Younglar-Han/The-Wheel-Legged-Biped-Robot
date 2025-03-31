#ifndef LKCAMMOTORCOMMANDER_HPP
#define LKCAMMOTORCOMMANDER_HPP

#include "CanManager.hpp"

class LKCanMotorCommander
{
private:
    int16_t msgBuffer[2][4];
    bool dirtyFlag[2];

    LKCanMotorCommander()
    {
        for (int i = 0; i < 2; ++i)
        {
            dirtyFlag[i] = false;
        }
    }

public:
    static LKCanMotorCommander *Instance()
    {
        static LKCanMotorCommander instance;
        return &instance;
    }

    virtual void Update();
    void Add2Buffer(CAN_TypeDef *_can, int32_t _canId, int16_t _current);
};

#endif
