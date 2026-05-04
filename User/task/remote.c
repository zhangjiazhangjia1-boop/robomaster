/*
    remote Task
    
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "module/chassis.h"
#include "module/gimbal.h"
#include "module/cmd/cmd.h"
#include "device/dr16.h"
#include "module/shoot.h"
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
// Chassis_CMD_t c_rc_cmd;
// Gimbal_CMD_t g_rc_cmd;
// Shoot_CMD_t s_rc_cmd;
// DR16_t remote_dr16;
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Task_remote(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  
  /* 计算任务运行到指定频率需要等待的tick数 */
  const uint32_t delay_tick = osKernelGetTickFreq() / REMOTE_FREQ;

  osDelay(REMOTE_INIT_DELAY); /* 延时一段时间再开启任务 */

  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  /* USER CODE INIT BEGIN */

  /* USER CODE INIT END */
  
  while (1) {
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* USER CODE BEGIN */
// if(osMessageQueueGet(task_runtime.msgq.remote.dr16, &remote_dr16, NULL, 0)==osOK);
// 	Chassis_Cmd(&c_rc_cmd,&remote_dr16);
//   Gimbal_Cmd(&g_rc_cmd,&remote_dr16);
//   Shoot_Cmd(&s_rc_cmd,&remote_dr16);
  
// 	osMessageQueueReset(task_runtime.msgq.chassis.cmd);
// 	osMessageQueuePut(task_runtime.msgq.chassis.cmd,&c_rc_cmd, 0, 0);
// 	osMessageQueueReset(task_runtime.msgq.gimbal.cmd); 
// 	osMessageQueuePut(task_runtime.msgq.gimbal.cmd,&g_rc_cmd, 0, 0);
//   osMessageQueueReset(task_runtime.msgq.shoot.cmd); 
// 	osMessageQueuePut(task_runtime.msgq.shoot.cmd,&s_rc_cmd, 0, 0);
    /* USER CODE END */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
  
}