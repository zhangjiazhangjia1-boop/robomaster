/*
    cmd_ctrl Task
    
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "module/cmd/cmd.h"
#include "module/config.h"
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
#if CMD_RCTypeTable_Index == 0
DR16_t cmd_dr16;
#elif CMD_RCTypeTable_Index == 1
AT9S_t cmd_at9s;
#endif
Shoot_CMD_t    *cmd_for_shoot;
Chassis_CMD_t  *cmd_for_chassis;
Gimbal_CMD_t   *cmd_for_gimbal;

static CMD_t cmd;
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Task_cmd_ctrl(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  
  /* 计算任务运行到指定频率需要等待的tick数 */
  const uint32_t delay_tick = osKernelGetTickFreq() / CMD_CTRL_FREQ;

  osDelay(CMD_CTRL_INIT_DELAY); /* 延时一段时间再开启任务 */

  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  /* USER CODE INIT BEGIN */
 CMD_Init(&cmd, &Config_GetRobotParam()->cmd_param);
  /* USER CODE INIT END */
  
  while (1) {
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* USER CODE BEGIN */
    #if CMD_RCTypeTable_Index == 0
    osMessageQueueGet(task_runtime.msgq.cmd.rc, &cmd_dr16, NULL, 0);
    #elif CMD_RCTypeTable_Index == 1
    osMessageQueueGet(task_runtime.msgq.cmd.rc, &cmd_at9s, NULL, 0);
    #endif
    CMD_Update(&cmd);
    
    /* 获取命令发送到各模块 */
    cmd_for_chassis = CMD_GetChassisCmd(&cmd);
    cmd_for_gimbal = CMD_GetGimbalCmd(&cmd);
    cmd_for_shoot = CMD_GetShootCmd(&cmd);
	osMessageQueueReset(task_runtime.msgq.gimbal.cmd);
    osMessageQueuePut(task_runtime.msgq.gimbal.cmd, cmd_for_gimbal, 0, 0);
	osMessageQueueReset(task_runtime.msgq.shoot.cmd);
    osMessageQueuePut(task_runtime.msgq.shoot.cmd, cmd_for_shoot, 0, 0);
	osMessageQueueReset(task_runtime.msgq.chassis.cmd);
    osMessageQueuePut(task_runtime.msgq.chassis.cmd, cmd_for_chassis, 0, 0);
    /* USER CODE END */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
  
}