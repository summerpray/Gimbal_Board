//
// Created by summerpray on 2021/11/2.
//

#include <first_order_filter.h>
#include "M_Gimbal.h"
#include "math.h"

extern remote_control RC;

//motor enconde value format, range[0-8191]
//电机编码值规整 0—8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
  * @param          输入的遥控器值
  * @param          输出的死区处理后遥控器值
  * @param          死区值
  */
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


/**
  * @brief          初始化云台
  * @Author         summerpray
  */
void M_Gimbal::init() {
    //电机速度环PID
    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};

    //电机陀螺仪角度环PID
    static const fp32 Pitch_angle_pid[3] = {PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD};
    static const fp32 Yaw_angle_pid[3] = {YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD};

    //电机编码器角度环PID
    static const fp32 Pitch_encode_pid[3] = {PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD};
    static const fp32 Yaw_encode_pid[3] = {YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD};

    gimbal_yaw_motor.gimbal_motor_measure = gimbal_can.get_gimbal_motor_measure_point(YAW);
    gimbal_pitch_motor.gimbal_motor_measure = gimbal_can.get_gimbal_motor_measure_point(PITCH);

    //TODO: 在INS初始化移植完毕后取消注释这里
    //init->gimbal_INT_angle_point = get_INS_angle_point();
    //init->gimbal_INT_gyro_point = get_gyro_data_point();

    //遥控器数据指针获取
    Rc = RC.get_remote_control_point();

    //初始化电机控制模式
    gimbal_motor_mode = last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;

    //初始化yaw电机pid
    gimbal_yaw_motor.gimbal_motor_gyro_pid.PID_init(PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT,
                                                          YAW_SPEED_PID_MAX_IOUT);
    gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.PID_init(PID_POSITION, Yaw_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT,
                                                                    YAW_GYRO_ABSOLUTE_PID_MAX_IOUT);
    gimbal_yaw_motor.gimbal_motor_relative_angle_pid.PID_init(PID_POSITION, Yaw_encode_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT,
                                                                    YAW_ENCODE_RELATIVE_PID_MAX_IOUT);

    //初始化pitch电机pid
    gimbal_pitch_motor.gimbal_motor_gyro_pid.PID_init(PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT,
                                                          PITCH_SPEED_PID_MAX_IOUT);
    gimbal_pitch_motor.gimbal_motor_absolute_angle_pid.PID_init(PID_POSITION, Pitch_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT,
                                                                      PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT);
    gimbal_pitch_motor.gimbal_motor_relative_angle_pid.PID_init(PID_POSITION, Pitch_encode_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT,
                                                                      PITCH_ENCODE_RELATIVE_PID_MAX_IOUT);

    //定义yaw和pitch的限位
    //TODO:需要测试,先关闭
    //max_yaw = 0.0;
    //min_yaw = 0.0;
    //max_pitch = 0.0;
    //min_pitch = 0.0;

    //更新云台数据
    feedback_update();

    gimbal_yaw_motor.absolute_angle_set = gimbal_yaw_motor.absolute_angle;
    gimbal_yaw_motor.relative_angle_set = gimbal_yaw_motor.relative_angle;
    gimbal_yaw_motor.motor_gyro_set = gimbal_yaw_motor.motor_gyro;

    gimbal_pitch_motor.absolute_angle_set = gimbal_pitch_motor.absolute_angle;
    gimbal_pitch_motor.relative_angle_set = gimbal_pitch_motor.relative_angle;
    gimbal_pitch_motor.motor_gyro_set = gimbal_pitch_motor.motor_gyro;
}

/**
  * @brief          更新云台数据
  * @Author         summerpray
  */
void M_Gimbal::feedback_update()
{

    //云台数据更新
    gimbal_pitch_motor.absolute_angle = *(gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);

#if PITCH_TURN
    gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                   gimbal_pitch_motor.offset_ecd);
#else

    feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          feedback_update->gimbal_pitch_motor.offset_ecd);
#endif

    gimbal_pitch_motor.motor_gyro = *(gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

    gimbal_yaw_motor.absolute_angle = *(gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);

#if YAW_TURN
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);

#else
    gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                gimbal_yaw_motor.offset_ecd);
#endif
    gimbal_yaw_motor.motor_gyro = cos(double(gimbal_pitch_motor.relative_angle)) * (*(gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                - sin(double(gimbal_pitch_motor.relative_angle)) * (*(gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
}

/**
  * @brief          设置云台控制模式,主要在'behaviour_mode_set'函数中改变
  * @Author         summerpray
  */
void M_Gimbal::set_mode() {
    //虽然这边就一个函数,但还是先留着,不要不知好歹
    behaviour_mode_set();
}


void M_Gimbal::behaviour_mode_set() {

    //云台行为状态机设置
    behavour_set();

    //根据云台行为状态机设置电机状态机
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE){
        gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_INIT){
        gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE){
        gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE){
        gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS){
        gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }

}

/**
  * @brief          云台行为状态机设置
  * @Author         summerpray
  */
void M_Gimbal::behavour_set(){
    //TODO:校准模式未写

    //初始化模式判断是否到达中值位置
    if (gimbal_behaviour == GIMBAL_INIT){
        static uint16_t init_time = 0;
        static uint16_t init_stop_time = 0;
        init_time++;

        if ((fabs(gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
             fabs(gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR)){
            if (init_stop_time < GIMBAL_INIT_STOP_TIME){
                init_stop_time++;
            }
        }
        else{
            if (init_time < GIMBAL_INIT_TIME){
                init_time++;
            }
        }

        //超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态开关打下档，或者掉线
        //TODO:掉线未写
        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
            !switch_is_down(Rc->rc.s[GIMBAL_MODE_CHANNEL]))
        {
            return;
        }
        else
        {
            init_stop_time = 0;
            init_time = 0;
        }

    }

    //开关控制 云台状态
    if (switch_is_down(Rc->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
    else if (switch_is_mid(Rc->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
    }
    else if (switch_is_up(Rc->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    }

    //TODO:此处要设置一个遥控器离线检测使云台不上电
    /*
    if( toe_is_error(DBUS_TOE))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
    */

    //enter init mode
    //判断进入init状态机
    {
        static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
        if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE)
        {
            gimbal_behaviour = GIMBAL_INIT;
        }
        last_gimbal_behaviour = gimbal_behaviour;
    }
}

/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  * @Author         summerpray
  */
fp32 M_Gimbal::motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd){
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}




/*****************************(C) GIMBAL PID *******************************/
void M_Gimbal::PID_clear(pid *gimbal_pid_clear) {
    if (gimbal_pid_clear == NULL)
        return;
    gimbal_pid_clear->error[3] = gimbal_pid_clear->set = gimbal_pid_clear->fdb = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}


/*****************************(C) GIMBAL PID *******************************/