/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-04-2021     summerpray       1. doing
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef INS_Task_H
#define INS_Task_H
#ifdef __cplusplus
extern "C"{
#endif

#include "INS_task.h"
#include "pid.h"

#ifdef __cplusplus
}
#endif
#include "main.h"
#include "cmsis_os.h"

class IMU{
public:
    IMU();
    fp32 *INS_angle;
    fp32 *INS_quat;
    fp32 *INS_gyro;
    fp32 *INS_accel;
    fp32 *INS_mag;
};

#endif
