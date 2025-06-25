#include "bsp_rc.h"

#include "main.h"

#include "bsp_usart.h"
#include "string.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_raw_t *rc_raw);

static RC_raw_t rc_raw;
static uint8_t SBUS_rx_buf[2][SBUS_RX_BUF_NUM];

RC_raw_t *get_remote_control_raw(void)
{
	return &rc_raw;
}

static uint8_t rc_update_flag = 0x00;

void bsp_set_update_flag(void)
{
	rc_update_flag = 0x01;
}

void bsp_remote_control_init(void)
{
	bsp_rc_init(SBUS_rx_buf[0], SBUS_rx_buf[1], SBUS_RX_BUF_NUM);
}

void bsp_rc_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}

void bsp_rc_disable(void)
{
    __HAL_UART_DISABLE(&huart3);
}

void bsp_rc_enable(void)
{
	__HAL_UART_ENABLE(&huart3);
}

void bsp_rc_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    hdma_usart3_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    __HAL_UART_ENABLE(&huart3);
}

uint8_t bsp_read_rc_update_flag(void)
{
	uint8_t _update_flag_shadow = rc_update_flag;
	rc_update_flag = 0x00;

	return _update_flag_shadow;
}

void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                SBUS_TO_RC(SBUS_rx_buf[0], &rc_raw);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                SBUS_TO_RC(SBUS_rx_buf[1], &rc_raw);
            }
        }
    }
}

static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_raw_t *rc_raw)
{   
    //数据为空或者遥控器数据头不对(SBUS数据头为0x0f)
    if (sbus_buf == NULL || rc_raw == NULL || sbus_buf[0] != 0x0f)
    {
        return;
    }

    rc_raw->rc.ch[0] = ((sbus_buf[1] >> 0) | (sbus_buf[2] << 8)) & 0x07ff; //!< Channel 1
    rc_raw->rc.ch[1] = ((sbus_buf[2] >> 3) | (sbus_buf[3] << 5)) & 0x07ff; //!< Channel 2
    rc_raw->rc.ch[3] = ((sbus_buf[3] >> 6) | (sbus_buf[4] << 2) | (sbus_buf[5] << 10)) &0x07ff; //!< Channel 4
    rc_raw->rc.ch[2] = ((sbus_buf[5] >> 1) | (sbus_buf[6] << 7)) & 0x07ff; //!< Channel 3 //富斯i6x与DT7通道34相反
    rc_raw->rc.ch[4] = ((sbus_buf[6] >> 4) | (sbus_buf[7] << 4)) & 0x07ff; //!< Channel 5
    rc_raw->rc.ch[5] = ((sbus_buf[7] >> 7) | (sbus_buf[8] << 1) | (sbus_buf[9] << 9)) & 0x07ff; //!< Channel 6

    rc_raw->rc.s[0] = ((sbus_buf[9] >> 2) | (sbus_buf[10] << 6)) & 0x07ff; //!< Switch A
    rc_raw->rc.s[1] = ((sbus_buf[10] >> 5) | (sbus_buf[11] << 3)) & 0x07ff; //!< Switch B
    rc_raw->rc.s[2] = ((sbus_buf[12] >> 0) | (sbus_buf[13] << 8)) & 0x07ff; //!< Switch C
    rc_raw->rc.s[3] = ((sbus_buf[13] >> 3) | (sbus_buf[14] << 5)) & 0x07ff; //!< Switch D


    rc_raw->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_raw->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_raw->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_raw->rc.ch[3] -= RC_CH_VALUE_OFFSET;

    bsp_set_update_flag();

    for (int i = 0; i < 4; i++)
    {
        if (rc_raw->rc.s[i] < 240 + 128)
        {
            rc_raw->rc.s[i] = 1;
        }
        else if (rc_raw->rc.s[i] > 1807 - 128)
        {
            rc_raw->rc.s[i] = 3;
        }
        else
        {
            if (i == 2)  // Only for switch C
                rc_raw->rc.s[i] = 2;
        }
    }
}
