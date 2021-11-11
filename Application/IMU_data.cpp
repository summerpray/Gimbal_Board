//
// Created by worker on 2021/11/9.
//

#include "IMU_data.h"


IMU::IMU() {
    *INS_angle = *get_INS_angle_point();
    *INS_gyro = *get_gyro_data_point();
    *INS_accel = *get_accel_data_point();
    *INS_quat = *get_INS_quat_point();
    *INS_mag = *get_mag_data_point();
}