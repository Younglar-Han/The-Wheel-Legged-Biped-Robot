#include "bsp_tof.h"

typedef struct
{
    int16_t distance;
    uint16_t noise;
    uint32_t peak;
    uint8_t confidence;
    uint32_t intg;
    int16_t reftof;
} LidarPointTypedef;

uint32_t receive_cnt;
uint8_t confidence;
uint16_t distance, noise, reftof, last_distance;
uint32_t peak, intg;

LidarPointTypedef Pack_Data[12];
LidarPointTypedef Pack_sum;
void bsp_tof_init(uint8_t *rx_buf)
{
    // RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    // // RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    // // RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
    // // RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);

    // GPIO_InitTypeDef GPIO_InitStructure;
    // USART_InitTypeDef USART_InitStructure;

    // GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    // GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    // GPIO_Init(GPIOA, &GPIO_InitStructure);

    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    // GPIO_Init(GPIOB, &GPIO_InitStructure);

    // USART_DeInit(USART1);

    // USART_InitStructure.USART_BaudRate = 230400;
    // USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    // USART_InitStructure.USART_StopBits = USART_StopBits_1;
    // USART_InitStructure.USART_Parity = USART_Parity_No;
    // USART_InitStructure.USART_Mode = USART_Mode_Rx;
    // USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    // USART_Init(USART1, &USART_InitStructure);

    // USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    // // USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    // USART_ClearFlag(USART1, USART_FLAG_IDLE);
    // USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

    // USART_Cmd(USART1, ENABLE);

    // // USART1 IDLE Interrupt
    // NVIC_InitTypeDef NVIC_InitStructure;
    // NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStructure);

    // // DMA Initialization
    // DMA_InitTypeDef DMA_InitStructure;
    // RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    // DMA_DeInit(DMA2_Stream5);

    // DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    // DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
    // DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx_buf;
    // DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    // DMA_InitStructure.DMA_BufferSize = 200;
    // DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    // DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    // DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    // DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    // DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    // DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    // DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    // DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    // DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    // DMA_Init(DMA2_Stream5, &DMA_InitStructure);

    // DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);

    // // RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    // // DMA_DeInit(DMA2_Stream7);

    // // DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    // // DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
    // // DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    // // DMA_InitStructure.DMA_BufferSize = 200;
    // // DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // // DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    // // DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    // // DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    // // DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    // // DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    // // DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    // // DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    // // DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    // // DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    // // DMA_Init(DMA2_Stream7, &DMA_InitStructure);

    // // DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);

    // NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStructure);
}

// void bsp_tof_send(uint8_t *_pData, uint16_t _len)
// {
//     DMA_Cmd(DMA2_Stream7, DISABLE);
//     DMA2_Stream7->M0AR = (uint32_t)_pData;

//     DMA_SetCurrDataCounter(DMA2_Stream7, _len);
//     DMA_Cmd(DMA2_Stream7, ENABLE);
//     USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
// }

// uint8_t bsp_tof_send_busy(void)
// {
//     if (DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7) == SET)
//     {
//         return 0x01;
//     }

//     return 0x00;
// }


// void DMA2_Stream7_IRQHandler(void)
// {
//     DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
// }
