#include "main_task.hpp"
#include "cmsis_os.h"
#include "main.h"

#include "Dr16.hpp"

Dr16 *pDr16 = Dr16::Instance();

void main_task(void const * argument)
{
    Dr16::Instance()->Init();
    while(1)
    {
        Dr16::Instance()->Update();
        osDelay(1);
    }
}
