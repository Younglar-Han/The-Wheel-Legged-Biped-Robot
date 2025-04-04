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

// Test stuff
Testbot testbot;

BoardPacketManager *pBPM = BoardPacketManager::Instance();

AHRSEstimator *mahonyEstimator = AHRSEstimator::Instance();
BMI088 *bmi088 = BMI088::Instance();
CanMsgDispatcher *cmd = CanMsgDispatcher::Instance();
STP23L *stp23l = STP23L::Instance();
LKCanMotorCommander *LK = LKCanMotorCommander::Instance();
Dr16 *dr16 = Dr16::Instance();

float use_rate;
float count = 0.0f;
uint32_t init_time = 0;
bool is_init = false;


void main_task(void const * argument)
{
    Dr16::Instance()->Init();
    Buzzer::Instance()->Init();

    CanManager::Instance()->Init();

    BMI088::Instance()->BspInit();

    testbot.Init();
    
    Dr16::Instance()->Init();
    while(1)
    {
        Time::Tick();

        Dr16::Instance()->Update();

        CanMsgDispatcher::Instance()->Update();

        BMI088::Instance()->Update();
        AHRSEstimator::Instance()->Update();

        testbot.Tick();

        if (Time::GetTick() % 2 == 0)
            LKCanMotorCommander::Instance()->Update();

        CanManager::Instance()->Update();

        if  (is_init == false)
        {
            is_init = true;
            init_time = HAL_GetTick();
            count = (float)init_time;
        }
        use_rate = count / ((float)HAL_GetTick() / 1000.0f);
        count += 1.0f;
        osDelay(1);
    }
}
