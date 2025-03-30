#ifndef TIME_HPP
#define TIME_HPP

#include "stm32f4xx.h"

class Time
{
private:
    uint32_t m_CurrentTick;
    uint32_t m_MsPerTick;

    static Time* Instance()
    {
        static Time instance;
        return &instance;
    }

    void SysTick_Init(uint32_t tick_ms)
    {
	    // SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

	    // uint32_t reload = SystemCoreClock / 8000 * tick_ms;

	    // SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; 			// Enable SysTick Interrupt
	    // SysTick->LOAD = reload;
	    // SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; 			// Enable SysTick
    }

public:
    Time() : m_CurrentTick(0),
             m_MsPerTick(1)
    {
    }

    static void Init(uint32_t _msPerTick)
    {
        Instance()->m_MsPerTick = _msPerTick; 
        Instance()->SysTick_Init(_msPerTick);
    }
    static uint32_t GetMsPerTick(){ return Instance()->m_MsPerTick; }
    static uint32_t GetTick(){ return Instance()->m_CurrentTick; }
    static void Tick(){ ++Instance()->m_CurrentTick; }
};

#endif
