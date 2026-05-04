#include "task/user_task.h"

Task_Runtime_t task_runtime;

const osThreadAttr_t attr_init = {
    .name = "Task_Init",
    .priority = osPriorityRealtime,
    .stack_size = 256 * 4,
};

/* User_task */
const osThreadAttr_t attr_chassis_ctrl = {
    .name = "chassis_ctrl",
    .priority = osPriorityNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_rc = {
    .name = "rc",
    .priority = osPriorityNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_remote = {
    .name = "remote",
    .priority = osPriorityNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_gimbal_ctrl = {
    .name = "gimbal_ctrl",
    .priority = osPriorityNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_atti_esti = {
    .name = "atti_esti",
    .priority = osPriorityNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_shoot_ctrl = {
    .name = "shoot_ctrl",
    .priority = osPriorityNormal,
    .stack_size = 256 * 4,
};
const osThreadAttr_t attr_cmd_ctrl = {
    .name = "cmd_ctrl",
    .priority = osPriorityNormal,
    .stack_size = 256 * 4,
};