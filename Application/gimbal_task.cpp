/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ����������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-03-2021     Summerpray      1. doing
  *
  *
  @verbatim
  ==============================================================================
 *      ������       ������
 *   �������� �ة��������������� �ة�����
 *   ��                 ��
 *   ��       ������       ��
 *   ��  ���Щ�       ���Щ�  ��
 *   ��                 ��
 *   ��       ���ة�       ��
 *   ��                 ��
 *   ����������         ����������
 *       ��         ��
 *       ��         ��
 *       ��         ��
 *       ��         ��������������������������������
 *       ��                        ��
 *       ��                        ������
 *       ��                        ������
 *       ��                        ��
 *       ������  ��  �����������������Щ�����  ��������
 *         �� ���� ����       �� ���� ����
 *         �������ة�����       �������ة�����
 *                ���ޱ���
 *               ������BUG!
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "gimbal_task.h"
#include "M_Gimbal.h"

void gimbal_task(void *pvParameters){
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    M_Gimbal gimbal;
    gimbal.init();
    while(1){
        ;
    }
}