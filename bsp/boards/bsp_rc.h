/**
 ****************************(C) COPYRIGHT 2016 DJI****************************
* @file       remote_control.c/h
* @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
*             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
*             的方式保证热插拔的稳定性。
* @note       
* @history
*  Version    Date            Author          Modification
*  V1.0.0     Dec-26-2018     RM              1. done
*  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
*
@verbatim
==============================================================================

==============================================================================
@endverbatim
****************************(C) COPYRIGHT 2016 DJI****************************
*/
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "struct_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define SBUS_RX_BUF_NUM 36u
#define NULL 0
#define RC_FRAME_LENGTH 25u

#define RC_CH_VALUE_MIN         ((uint16_t)240)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1807)
#define RC_CH_OFFSET_MAX        ((uint16_t)750)

/* ----------------------- RC Switch Definition----------------------------- */
// #define RC_SW_UP                ((uint16_t)1)
// #define RC_SW_MID               ((uint16_t)3)
// #define RC_SW_DOWN              ((uint16_t)2)
// #define switch_is_down(s)       (s == RC_SW_DOWN)
// #define switch_is_mid(s)        (s == RC_SW_MID)
// #define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
// #define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
// #define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
// #define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
// #define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
// #define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
// #define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
// #define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
// #define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
// #define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
// #define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
// #define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
// #define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
// #define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
// #define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
// #define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
// #define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */
typedef struct __attribute__((packed))
{
        struct __attribute__((packed))
        {
                int16_t ch[6];
                int16_t s[4];
        } rc;
        struct __attribute__((packed))
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        struct __attribute__((packed))
        {
                uint16_t v;
        } key;

} RC_raw_t;

/* ----------------------- Internal Data ----------------------------------- */

extern RC_raw_t* get_remote_control_raw(void);
extern void bsp_remote_control_init(void);
extern void bsp_rc_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void bsp_rc_disable(void);
extern void bsp_rc_enable(void);
extern void bsp_rc_restart(uint16_t dma_buf_num);
extern void USART3_IRQHandler(void);
extern uint8_t bsp_read_rc_update_flag(void);


// extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
// extern void RC_unable(void);
// extern void RC_restart(uint16_t dma_buf_num);

// extern void remote_control_init(void);
// extern const RC_ctrl_t *get_remote_control_point(void);
// extern uint8_t RC_data_is_error(void);
// extern void slove_RC_lost(void);
// extern void slove_data_error(void);
// extern void sbus_to_usart1(uint8_t *sbus);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
