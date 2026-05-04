/*
    rc Task
    
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "device/dr16.h"
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
 DR16_t dr16;
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Task_rc(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  
  /* 计算任务运行到指定频率需要等待的tick数 */
  const uint32_t delay_tick = osKernelGetTickFreq() / RC_FREQ;

  osDelay(RC_INIT_DELAY); /* 延时一段时间再开启任务 */

  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  /* USER CODE INIT BEGIN */
 DR16_Init(&dr16); /* 初始化dr16 */
  /* USER CODE INIT END */
  
  while (1) {
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* USER CODE BEGIN */
    DR16_StartDmaRecv(&dr16);
    if (DR16_WaitDmaCplt(20)) {
      DR16_ParseData(&dr16);
      
    } 
		else {
      DR16_Offline(&dr16);
    }
		
		osMessageQueueReset(task_runtime.msgq.cmd.rc);
		osMessageQueuePut(task_runtime.msgq.cmd.rc,&dr16, 0, 0);
    /* USER CODE END */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
  
}