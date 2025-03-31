/**
  ****************************************************************
  * @file    bsp_common_define.h
  * @author  TofuLemon(5thzhengjiarandom@gmail.com)
  * @date    2020-Jan-13
  ****************************************************************
  */

#ifndef COMMON_DEFINE_H
#define COMMON_DEFINE_H

#define RC_NVIC 4

#define USART6_NVIC 4

#define CAN1_NVIC 4
#define CAN2_NVIC 4
#define TIM3_NVIC 5
#define TIM6_NVIC 4
#define SPI5_RX_NVIC 5
#define MPU_INT_NVIC 5

#ifndef NULL
#define NULL 0
#endif

#ifndef PI
#define PI 3.14159265358979f
#endif

#ifndef PIX2
#define PIX2 6.283185307f
#endif


#endif /* BSP_COMMON_DEFINE_H_ */
