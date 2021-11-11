/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             ��Ҫ����������bmi088��������ist8310�������̬���㣬�ó�ŷ���ǣ�
  *             �ṩͨ��bmi088��data ready �ж�����ⲿ�������������ݵȴ��ӳ�
  *             ͨ��DMA��SPI�����ԼCPUʱ��.
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
