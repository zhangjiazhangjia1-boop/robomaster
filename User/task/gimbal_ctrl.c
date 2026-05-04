/*
    gimbal_ctrl Task
    
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "component/ahrs.h"
#include "module/gimbal.h"
#include "module/config.h"

/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
Gimbal_t gimbal;
Gimbal_IMU_t gimbal_imu;
Gimbal_CMD_t gimbal_cmd;
Gimbal_CMD_t gimbal_cmd_remote;
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Task_gimbal_ctrl(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  
  /* 计算任务运行到指定频率需要等待的tick数 */
  const uint32_t delay_tick = osKernelGetTickFreq() / GIMBAL_CTRL_FREQ;

  osDelay(GIMBAL_CTRL_INIT_DELAY); /* 延时一段时间再开启任务 */

  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  /* USER CODE INIT BEGIN */
  Gimbal_Init(&gimbal, &Config_GetRobotParam()->gimbal_param, GIMBAL_CTRL_FREQ);
  /* USER CODE INIT END */
  
  while (1) {
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* USER CODE BEGIN */
/* 陀螺仪数据更新 */
		if(osMessageQueueGet(task_runtime.msgq.gimbal.imu, &gimbal_imu, NULL, 0)==osOK){
      Gimbal_UpdateIMU(&gimbal, &gimbal_imu);
		}
    osMessageQueueGet(task_runtime.msgq.gimbal.cmd, &gimbal_cmd, NULL, 0);

    Gimbal_UpdateFeedback(&gimbal);
    Gimbal_Control(&gimbal,&gimbal_cmd);
//    Gimbal_Output(&gimbal);

    osMessageQueueReset(task_runtime.msgq.chassis.yaw);
    osMessageQueuePut(task_runtime.msgq.chassis.yaw,&gimbal.feedback.motor.yaw.rotor_abs_angle, 0, 0);
    /* USER CODE END */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
  
}