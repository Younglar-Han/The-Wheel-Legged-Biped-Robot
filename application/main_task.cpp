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
#include "bsp_can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// Test stuff
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

uint32_t min_stack_size = 0;


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    CanRxMsg _rxMsg;
    _rxMsg.Header = rx_header;
    for (int i = 0; i < 8; i++)
    {
        _rxMsg.Data[i] = rx_data[i];
    }

    CanManager::Instance()->MsgQueue((hcan==&hcan1?0:1))->Enqueue(&_rxMsg);
}

void main_task(void const * argument)
{
    Testbot testbot;
    Dr16::Instance()->Init();
    Buzzer::Instance()->Init();

    CanManager::Instance()->Init();

    BMI088::Instance()->BspInit();
    can_filter_init();

    testbot.Init();
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

        min_stack_size = uxTaskGetStackHighWaterMark(NULL);

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
