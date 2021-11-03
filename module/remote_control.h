//
// Created by WSJ on 2021/11/2.
//

#ifndef CLASSIS_BOARD_REMOTE_CONTROL_H
#define CLASSIS_BOARD_REMOTE_CONTROL_H
#ifdef __cplusplus

#include <cstdint>

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u
//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700
#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- Rc Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */


//官方遥控器数据包
typedef struct {
    struct {
        int16_t ch[5];
        char s[2];
    }__attribute__((packed)) rc;
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    }__attribute__((packed)) mouse;
    struct {
        uint16_t v;//value:官方键盘开放的键位
    }__attribute__((packed)) key;

}__attribute__((packed)) RC_ctrl_t;


//遥控器控制
class remote_control {
public:
    RC_ctrl_t data;

    void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

    void RC_unable(void);

    void RC_restart(uint16_t dma_buf_num);

    void remote_control_init(void);

    RC_ctrl_t *get_remote_control_point(void);

    uint8_t RC_data_is_error(void);

    void slove_RC_lost(void);

    void slove_data_error(void);

    int16_t RC_abs(int16_t value);

};

#endif
#endif //CLASSIS_BOARD_REMOTE_CONTROL_H
