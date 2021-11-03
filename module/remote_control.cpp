//
// Created by WSJ on 2021/11/2.
//

#include <cstring>
#include "remote_control.h"
#include "main.h"
#include "bsp_usart.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
remote_control RC;

void remote_control::RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num) {
    //使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while (hdma_usart3_rx.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }
    hdma_usart3_rx.Instance->PAR = (uint32_t) &(USART3->DR);
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t) (rx1_buf);
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t) (rx2_buf);
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //使能双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}

void remote_control::RC_unable(void) {
    __HAL_UART_DISABLE(&huart3);
}

void remote_control::RC_restart(uint16_t dma_buf_num) {
    __HAL_UART_DISABLE(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    hdma_usart3_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    __HAL_UART_ENABLE(&huart3);
}

/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control::remote_control_init() {
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
RC_ctrl_t *remote_control::get_remote_control_point() {
    return &RC.data;
}

/**
  * @brief          判断遥控器数据是否出错
  * @param[in]      none
  * @retval         none
  */
uint8_t remote_control::RC_data_is_error() {
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(data.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (RC_abs(data.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (RC_abs(data.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (RC_abs(data.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (data.rc.s[0] == 0) {
        goto error;
    }
    if (data.rc.s[1] == 0) {
        goto error;
    }
    return 0;

    error:
    data.rc.ch[0] = 0;
    data.rc.ch[1] = 0;
    data.rc.ch[2] = 0;
    data.rc.ch[3] = 0;
    data.rc.ch[4] = 0;
    data.rc.s[0] = RC_SW_DOWN;
    data.rc.s[1] = RC_SW_DOWN;
    data.mouse.x = 0;
    data.mouse.y = 0;
    data.mouse.z = 0;
    data.mouse.press_l = 0;
    data.mouse.press_r = 0;
    data.key.v = 0;
    return 1;
}

/**
  * @brief          解决遥控器丢失的情况(直接重启)
  * @param[in]      none
  * @retval         none
  */
void remote_control::slove_RC_lost(void) {
    RC_restart(SBUS_RX_BUF_NUM);
}

/**
  * @brief          解决遥控器数据错误的情况(直接重启)
  * @param[in]      none
  * @retval         none
  */
void remote_control::slove_data_error(void) {
    RC_restart(SBUS_RX_BUF_NUM);
}

/**
  * @brief          通过usart1发送sbus数据,在usart3_IRQHandle调用
  * @param[in]      sbus: sbus数据, 18字节
  * @retval         none
  */
static void sbus_to_usart1(uint8_t *sbus) {
    static uint8_t usart_tx_buf[20];
    static uint8_t i = 0;
    usart_tx_buf[0] = 0xA6;
    memcpy(usart_tx_buf + 1, sbus, 18);
    for (i = 0, usart_tx_buf[19] = 0; i < 19; i++) {
        usart_tx_buf[19] += usart_tx_buf[i];
    }
    usart1_tx_dma_enable(usart_tx_buf, 20);
}

/**
  * @brief          取正函数
  * @param[in]      value
  * @retval         取正后的数
  */
int16_t remote_control::RC_abs(int16_t value) {
    if (value > 0) {
        return value;
    } else {
        return -value;
    }
}

static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl) {
    if (sbus_buf == NULL || rc_ctrl == NULL) {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) & 0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

//串口中断
void USART3_IRQHandler(void) {
    if (huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    } else if (USART3->SR & UART_FLAG_IDLE) {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
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

            if (this_time_rx_len == RC_FRAME_LENGTH) {
                sbus_to_rc(sbus_rx_buf[0], &RC.data);
                //记录数据接收时间
                //detect_hook(DBUS_TOE);
                sbus_to_usart1(sbus_rx_buf[0]);
            }
        } else {
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

            if (this_time_rx_len == RC_FRAME_LENGTH) {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &RC.data);
                //记录数据接收时间
                //detect_hook(DBUS_TOE);
                sbus_to_usart1(sbus_rx_buf[1]);
            }
        }
    }

}

