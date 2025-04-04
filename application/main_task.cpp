#include "main_task.hpp"
#include "cmsis_os.h"
#include "main.h"

#include "Buzzer.hpp"
#include "Dr16.hpp"
#include "CanManager.hpp"
#include "CanMsgDispatcher.hpp"

#include "BoardPacket.hpp"
#include "Time.hpp"

#include "Testbot.hpp"

#include "IST8310.hpp"
#include "BMI088.hpp"
#include "AHRSEstimator.hpp"
#include "LKCanMotorCommander.hpp"
#include "STP23L.hpp"



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
