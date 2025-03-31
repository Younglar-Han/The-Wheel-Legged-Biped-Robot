#include "main_task.hpp"
#include "cmsis_os.h"
#include "main.h"

#include "Time.hpp"
#include "Dr16.hpp"
#include "CanManager.hpp"
#include "CanMsgDispatcher.hpp"
#include "LKCanMotorCommander.hpp"

Dr16 *pDr16 = Dr16::Instance();

void main_task(void const * argument)
{
    Dr16::Instance()->Init();
    while(1)
    {
        Time::Tick();
        Dr16::Instance()->Update();
        CanMsgDispatcher::Instance()->Update();
        if (Time::GetTick() % 2 == 0)
            LKCanMotorCommander::Instance()->Update();

        CanManager::Instance()->Update();
        osDelay(1);
    }
}
