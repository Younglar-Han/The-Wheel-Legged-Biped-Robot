#include "main_task.hpp"
#include "cmsis_os.h"
#include "main.h"

#include "Time.hpp"
#include "Dr16.hpp"

Dr16 *pDr16 = Dr16::Instance();
uint32_t time;

void main_task(void const * argument)
{
    Dr16::Instance()->Init();
    while(1)
    {
        Time::Tick();
        time = Time::GetTick();
        Dr16::Instance()->Update();
        osDelay(1);
    }
}
