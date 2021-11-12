//
// Created by WSJ on 2021/11/2.
//

//#ifdef __cplusplus
//extern "C"{
//#endif
//#ifdef __cplusplus
//}
//#endif

#include "System_Config.h"
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "task.h"


extern remote_control RC;
#define Tiny_Stack_Size       64
#define Small_Stack_Size      128
#define Normal_Stack_Size     256
#define Large_Stack_Size      512
#define Huge_Stack_Size       1024
#define PriorityVeryLow       1
#define PriorityLow           2
#define PriorityBelowNormal   3
#define PriorityNormal        4
#define PriorityAboveNormal   5
#define PriorityHigh          6
#define PrioritySuperHigh     7
#define PriorityRealtime      8

TaskHandle_t Gimbal_Task_Handle;



/**
* @brief Load and start User Tasks.
* @note  Edit this function to add tasks into the activated tasks list.
*/
void Task_start(void) {
    /* Syetem Service init --------------*/
    /* Applications Init ----------------*/
    xTaskCreate(gimbal_task, "gimbal_task", Normal_Stack_Size, nullptr, PriorityAboveNormal, &Gimbal_Task_Handle);
}