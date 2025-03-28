#include "bsp_heatImu.h"
#include "main.h"

//通过设置PWM占空比来调节
extern TIM_HandleTypeDef htim10;
void bsp_pwm_set(uint16_t set)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, set);
}
