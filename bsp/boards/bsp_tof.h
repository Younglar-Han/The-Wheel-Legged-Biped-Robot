#ifndef BSP_TOF_H
#define BSP_TOF_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "stm32f4xx.h"

#define HEADER 0xAA
#define device_address 0x00
#define chunk_offset 0x00
#define PACK_GET_DISTANCE 0x02
#define PACK_RESET_SYSTEM 0x0D
#define PACK_STOP 0x0F
#define PACK_ACK 0x10
#define PACK_VERSION 0x14

void bsp_tof_init(uint8_t *rx_buf);
// void DMA2_Stream7_IRQHandler(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
