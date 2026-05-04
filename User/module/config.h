/*
 * 配置相关
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "module/chassis.h"
#include "module/gimbal.h"
#include "module/shoot.h"
#include "cmd/cmd.h"


/**
 * @brief 机器人参数配置结构体
 * @note 在此添加您的配置参数
 */
typedef struct {
    // 示例配置项（可根据实际需求修改或删除）
	
    
    /* USER CODE BEGIN Config_RobotParam */
    // 在此添加您的配置参数
 Chassis_Params_t chassis_param;
 Gimbal_Params_t gimbal_param;
 Shoot_Params_t shoot_param;
 CMD_Config_t cmd_param;

    /* USER CODE END Config_RobotParam */
} Config_RobotParam_t;

/* Exported functions prototypes -------------------------------------------- */

/**
 * @brief 获取机器人配置参数
 * @return 机器人配置参数指针
 */
Config_RobotParam_t* Config_GetRobotParam(void);

#ifdef __cplusplus
}
#endif
