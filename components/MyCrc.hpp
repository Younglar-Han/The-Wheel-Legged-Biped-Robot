#ifndef MY_CRC_HPP
#define MY_CRC_HPP

#include "stm32f4xx.h"

class Crc
{
private:
    const static uint8_t CRC8_TAB[256];

    const static uint16_t CRC16_TAB[256];

public:
    const static uint8_t CRC8_INIT;
    const static uint16_t CRC16_INIT;
    static uint8_t Get_CRC8_Check_Sum(uint8_t* pchMessage, uint32_t dwLength, uint8_t ucCRC8);
    static bool VerifyCrc8CheckSum(uint8_t* pchMessage, uint32_t dwLength);

    static uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
    static bool VerifyCrc16CheckSum(uint8_t* pchMessage, uint32_t dwLength);
};

#endif
