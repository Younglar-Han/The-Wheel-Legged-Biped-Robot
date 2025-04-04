/**
  ****************************************************************
  * @file    Buzzer.hpp
  * @author  TofuLemon(5thzhengjiarandom@gmail.com)
  * @date    2020-Jul-14
  ****************************************************************
  */

#ifndef BUZZER_HPP
#define BUZZER_HPP

#include "bsp_buzzer.h"

class Buzzer
{
private:

    uint32_t m_Freq;

    Buzzer():m_Freq(440){}
    Buzzer(const Buzzer &);
    Buzzer& operator = (const Buzzer &);

public:

    void Init()
    {
        bsp_buzzer_off();
    }
    void On(){ }
    void Off(){ bsp_buzzer_off(); }
    void SetFreq(float freq)
    {
        m_Freq = freq;
        bsp_buzzer_set_freq(freq);
    }
    uint32_t GetFreq(){ return m_Freq; }

    static Buzzer* Instance()
    {
        static Buzzer instance;
        return &instance;
    }

};


#endif /* BUZZER_HPP */
