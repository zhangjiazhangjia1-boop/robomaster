/*
    shoot_ctrl Task
    
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "module/shoot.h"
#include "module/config.h"
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
Shoot_t shoot;
Shoot_CMD_t shoot_cmd;
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Task_shoot_ctrl(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  
  /* 计算任务运行到指定频率需要等待的tick数 */
  const uint32_t delay_tick = osKernelGetTickFreq() / SHOOT_CTRL_FREQ;

  osDelay(SHOOT_CTRL_INIT_DELAY); /* 延时一段时间再开启任务 */

  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  /* USER CODE INIT BEGIN */
  Shoot_Init(&shoot, &Config_GetRobotParam()->shoot_param, SHOOT_CTRL_FREQ);
  /* USER CODE INIT END */
  
  while (1) {
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* USER CODE BEGIN */
	  osMessageQueueGet(task_runtime.msgq.shoot.cmd, &shoot_cmd, NULL, 0);
	  Shoot_UpdateFeedback(&shoot);
    Shoot_SetMode(&shoot,shoot_cmd.mode);
//	  Shoot_Control(&shoot,&shoot_cmd);
    /* USER CODE END */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
  
}