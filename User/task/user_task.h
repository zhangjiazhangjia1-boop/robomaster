#pragma once

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ----------------------------------------------------------------- */
#include <cmsis_os2.h>
#include "FreeRTOS.h"
#include "task.h"

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */
/* Exported constants ------------------------------------------------------- */
/* 任务运行频率 */
#define CHASSIS_CTRL_FREQ (500.0)
#define RC_FREQ (500.0)
#define REMOTE_FREQ (500.0)
#define GIMBAL_CTRL_FREQ (500.0)
#define ATTI_ESTI_FREQ (500.0)
#define SHOOT_CTRL_FREQ (500.0)
#define CMD_CTRL_FREQ (500.0)

/* 任务初始化延时ms */
#define TASK_INIT_DELAY (100u)
#define CHASSIS_CTRL_INIT_DELAY (0)
#define RC_INIT_DELAY (0)
#define REMOTE_INIT_DELAY (0)
#define GIMBAL_CTRL_INIT_DELAY (0)
#define ATTI_ESTI_INIT_DELAY (0)
#define SHOOT_CTRL_INIT_DELAY (0)
#define CMD_CTRL_INIT_DELAY (0)

/* Exported defines --------------------------------------------------------- */
/* Exported macro ----------------------------------------------------------- */
/* Exported types ----------------------------------------------------------- */

/* 任务运行时结构体 */
typedef struct {
    /* 各任务，也可以叫做线程 */
    struct {
        osThreadId_t chassis_ctrl;
        osThreadId_t rc;
        osThreadId_t remote;
        osThreadId_t gimbal_ctrl;
        osThreadId_t atti_esti;
        osThreadId_t shoot_ctrl;
        osThreadId_t cmd_ctrl;
    } thread;

    /* USER MESSAGE BEGIN */
    struct {
        osMessageQueueId_t user_msg; /* 用户自定义任务消息队列 */
		struct {
            osMessageQueueId_t cmd;
			osMessageQueueId_t yaw;					
        }chassis;
        struct {
            osMessageQueueId_t imu;
            osMessageQueueId_t cmd;
        }gimbal;
		struct {
            osMessageQueueId_t cmd; /* 发射命令队列 */
        }shoot;
				struct {
            osMessageQueueId_t mode; /* 发射命令队列 */
            osMessageQueueId_t dr16;/* 发射命令队列 */
        }remote;  
				struct { /* 发射命令队列 */
            osMessageQueueId_t rc;/* 发射命令队列 */
        }cmd;      
	} msgq;
    /* USER MESSAGE END */

    /* 机器人状态 */
    struct {
        float battery; /* 电池电量百分比 */
        float vbat; /* 电池电压 */
        float cpu_temp; /* CPU温度 */
    } status;

    /* USER CONFIG BEGIN */
		struct {
            osMessageQueueId_t mode; /* 发射命令队列 */
            osMessageQueueId_t dr16;/* 发射命令队列 */
        }remote;
    /* USER CONFIG END */

    /* 各任务的stack使用 */
    struct {
        UBaseType_t chassis_ctrl;
        UBaseType_t rc;
        UBaseType_t remote;
        UBaseType_t gimbal_ctrl;
        UBaseType_t atti_esti;
        UBaseType_t shoot_ctrl;
        UBaseType_t cmd_ctrl;
    } stack_water_mark;

    /* 各任务运行频率 */
    struct {
        float chassis_ctrl;
        float rc;
        float remote;
        float gimbal_ctrl;
        float atti_esti;
        float shoot_ctrl;
        float cmd_ctrl;
    } freq;

    /* 任务最近运行时间 */
    struct {
        float chassis_ctrl;
        float rc;
        float remote;
        float gimbal_ctrl;
        float atti_esti;
        float shoot_ctrl;
        float cmd_ctrl;
    } last_up_time;

} Task_Runtime_t;

/* 任务运行时结构体 */
extern Task_Runtime_t task_runtime;

/* 初始化任务句柄 */
extern const osThreadAttr_t attr_init;
extern const osThreadAttr_t attr_chassis_ctrl;
extern const osThreadAttr_t attr_rc;
extern const osThreadAttr_t attr_remote;
extern const osThreadAttr_t attr_gimbal_ctrl;
extern const osThreadAttr_t attr_atti_esti;
extern const osThreadAttr_t attr_shoot_ctrl;
extern const osThreadAttr_t attr_cmd_ctrl;

/* 任务函数声明 */
void Task_Init(void *argument);
void Task_chassis_ctrl(void *argument);
void Task_rc(void *argument);
void Task_remote(void *argument);
void Task_gimbal_ctrl(void *argument);
void Task_atti_esti(void *argument);
void Task_shoot_ctrl(void *argument);
void Task_cmd_ctrl(void *argument);

#ifdef __cplusplus
}
#endif