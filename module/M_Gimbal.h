//
// Created by WSJ on 2021/11/2.
//

#ifndef GIMBAL_BOARD_M_GIMBAL_H
#define GIMBAL_BOARD_M_GIMBAL_H

#include "motor_3508.h"
#include "struct_typedef.h"
#include "pid.h"
#include "first_order_filter.h"
#include "motor_6020.h"
#include "remote_control.h"
#include "CAN_receive.h"

//pitch speed close-loop PID params, max out and max iout
//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP        2900.0f
#define PITCH_SPEED_PID_KI        60.0f
#define PITCH_SPEED_PID_KD        0.0f
#define PITCH_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_SPEED_PID_MAX_IOUT  10000.0f

//yaw speed close-loop PID params, max out and max iout
//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP        3600.0f
#define YAW_SPEED_PID_KI        20.0f
#define YAW_SPEED_PID_KD        0.0f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  5000.0f

//pitch gyro angle close-loop PID params, max out and max iout
//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP 15.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f

#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//yaw gyro angle close-loop PID params, max out and max iout
//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP        26.0f
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD        0.3f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f

//pitch encode angle close-loop PID params, max out and max iout
//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP 15.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.00f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//yaw encode angle close-loop PID params, max out and max iout
//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP        8.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f

//任务开始空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
//yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL 0

//turn 180°
//掉头180 按键
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
//turn speed
//掉头云台速度
#define TURN_SPEED    0.04f
//测试按键尚未使用
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//rocker value deadband
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND   10

#define YAW_RC_SEN    -0.000005f
#define PITCH_RC_SEN  -0.000006f //0.005

#define YAW_MOUSE_SEN   0.00005f
#define PITCH_MOUSE_SEN 0.00015f

#define YAW_ENCODE_SEN    0.01f
#define PITCH_ENCODE_SEN  0.01f

#define GIMBAL_CONTROL_TIME 1

//test mode, 0 close, 1 open
//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

//电机是否接反
#define PITCH_TURN  1
#define YAW_TURN    0

//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED     0.004f
#define GIMBAL_INIT_YAW_SPEED       0.005f

#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET   8000
#define GIMBAL_CALI_STEP_TIME   2000
#define GIMBAL_CALI_GYRO_LIMIT  0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP  1
#define GIMBAL_CALI_PITCH_MIN_STEP  2
#define GIMBAL_CALI_YAW_MAX_STEP    3
#define GIMBAL_CALI_YAW_MIN_STEP    4

#define GIMBAL_CALI_START_STEP  GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP    5

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000

//电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif


class M_Gimbal{
public:
    const RC_ctrl_t  *Rc;

    void init();
};

#endif


