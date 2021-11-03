//
// Created by summerpray on 2021/11/2.
//

#include <first_order_filter.h>
#include "M_Gimbal.h"


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


void M_Gimbal::init(M_Gimbal *init) {
    //电机速度环PID
    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};

    //电机陀螺仪角度环PID
    static const fp32 Pitch_angle_pid[3] = {PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD};
    static const fp32 Yaw_angle_pid[3] = {YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD};

    //电机编码器角度环PID
    static const fp32 Pitch_encode_pid[3] = {PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD};
    static const fp32 Yaw_encode_pid[3] = {YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD};

    init->gimbal_yaw_motor.gimbal_motor_measure = gimbal_can.get_gimbal_motor_measure_point(YAW);
    init->gimbal_pitch_motor.gimbal_motor_measure = gimbal_can.get_gimbal_motor_measure_point(PITCH);

    //TODO: 在INS初始化移植完毕后取消注释这里
    //init->gimbal_INT_angle_point = get_INS_angle_point();
    //init->gimbal_INT_gyro_point = get_gyro_data_point();

    //遥控器数据指针获取
    init->gimbal_rc_ctrl = RC.get_remote_control_point();

    //初始化电机控制模式
    init->gimbal_motor_mode = init->last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;

    //初始化yaw电机pid
    init->gimbal_yaw_motor.gimbal_motor_gyro_pid.PID_init(PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT,
                                                          YAW_SPEED_PID_MAX_IOUT);
    init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.PID_init(PID_POSITION, Yaw_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT,
                                                                    YAW_GYRO_ABSOLUTE_PID_MAX_IOUT);
    init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid.PID_init(PID_POSITION, Yaw_encode_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT,
                                                                    YAW_ENCODE_RELATIVE_PID_MAX_IOUT);

    //初始化pitch电机pid
    init->gimbal_pitch_motor.gimbal_motor_gyro_pid.PID_init(PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT,
                                                          PITCH_SPEED_PID_MAX_IOUT);
    init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid.PID_init(PID_POSITION, Pitch_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT,
                                                                      PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT);
    init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid.PID_init(PID_POSITION, Pitch_encode_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT,
                                                                      PITCH_ENCODE_RELATIVE_PID_MAX_IOUT);

    //清除所有PID
    gimbal_total_pid_clear(init);
    //更新云台数据
    gimbal_feedback_update(init);

    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;

    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
}

/**
  * @brief          更新云台数据
  * @retval
  */
void M_Gimbal::gimbal_feedback_update(M_Gimbal *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    //云台数据更新
    feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);

#if PITCH_TURN
    feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                    feedback_update->gimbal_pitch_motor.offset_ecd);
#else

    feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          feedback_update->gimbal_pitch_motor.offset_ecd);
#endif

    feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

    feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);

#if YAW_TURN
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);

#else
    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                 feedback_update->gimbal_yaw_motor.offset_ecd);
#endif
    feedback_update->gimbal_yaw_motor.motor_gyro = cos(double(feedback_update->gimbal_pitch_motor.relative_angle)) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                   - sin(double(feedback_update->gimbal_pitch_motor.relative_angle)) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
}


/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
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

void M_Gimbal::gimbal_total_pid_clear(M_Gimbal *pid_clear){
    PID_clear(&(pid_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);
    PID_clear(&(pid_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);
    PID_clear(&(pid_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);

    PID_clear(&(pid_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);
    PID_clear(&(pid_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid);
    PID_clear(&(pid_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid);
}