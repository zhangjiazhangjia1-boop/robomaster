/*
    Init Task
    任务初始化，创建各个线程任务和消息队列
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"

/* USER INCLUDE BEGIN */
#include "device/dr16.h"
#include "module/chassis.h"
#include "module/gimbal.h"
#include "module/shoot.h"
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */

/**
 * \brief 初始化
 *
 * \param argument 未使用
 */
void Task_Init(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */
    /* USER CODE INIT BEGIN */

    /* USER CODE INIT END */
  osKernelLock(); /* 锁定内核，防止任务切换 */
  
  /* 创建任务线程 */
  task_runtime.thread.chassis_ctrl = osThreadNew(Task_chassis_ctrl, NULL, &attr_chassis_ctrl);
  task_runtime.thread.rc = osThreadNew(Task_rc, NULL, &attr_rc);
  task_runtime.thread.remote = osThreadNew(Task_remote, NULL, &attr_remote);
  task_runtime.thread.gimbal_ctrl = osThreadNew(Task_gimbal_ctrl, NULL, &attr_gimbal_ctrl);
  task_runtime.thread.atti_esti = osThreadNew(Task_atti_esti, NULL, &attr_atti_esti);
  task_runtime.thread.shoot_ctrl = osThreadNew(Task_shoot_ctrl, NULL, &attr_shoot_ctrl);
  task_runtime.thread.cmd_ctrl = osThreadNew(Task_cmd_ctrl, NULL, &attr_cmd_ctrl);

  // 创建消息队列
  /* USER MESSAGE BEGIN */
  task_runtime.msgq.user_msg= osMessageQueueNew(2u, 10, NULL);
  task_runtime.msgq.remote.dr16= osMessageQueueNew(2u, sizeof(DR16_t), NULL);
  task_runtime.msgq.chassis.cmd= osMessageQueueNew(2u, sizeof(Chassis_CMD_t), NULL);
  task_runtime.msgq.chassis.yaw= osMessageQueueNew(2u, sizeof(float), NULL);
  task_runtime.msgq.gimbal.imu= osMessageQueueNew(2u, sizeof(Gimbal_IMU_t), NULL);
  task_runtime.msgq.gimbal.cmd= osMessageQueueNew(2u, sizeof(Gimbal_CMD_t), NULL);
  task_runtime.msgq.shoot.cmd= osMessageQueueNew(2u, sizeof(Shoot_CMD_t), NULL);
  #if CMD_RCTypeTable_Index == 0
  task_runtime.msgq.cmd.rc= osMessageQueueNew(3u, sizeof(DR16_t), NULL);
#elif CMD_RCTypeTable_Index == 1
  task_runtime.msgq.cmd.rc= osMessageQueueNew(3u, sizeof(AT9S_t), NULL);
#endif
  /* USER MESSAGE END */

  osKernelUnlock(); // 解锁内核
  osThreadTerminate(osThreadGetId()); // 任务完成后结束自身
}