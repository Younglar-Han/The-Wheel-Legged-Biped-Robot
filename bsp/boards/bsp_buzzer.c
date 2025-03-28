#include "bsp_buzzer.h"
#include "main.h"
extern TIM_HandleTypeDef htim4;
void bsp_buzzer_set_freq(uint32_t freq)
{
    __HAL_TIM_PRESCALER(&htim4, 300000 / freq - 1);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 140);
}

void bsp_buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
