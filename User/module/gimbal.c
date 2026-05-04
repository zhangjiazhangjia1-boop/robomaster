/* Includes ----------------------------------------------------------------- */
#include "gimbal.h"
/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* Private function  -------------------------------------------------------- */
int8_t Gimbal_Init(Gimbal_t *g,Gimbal_Params_t *param,
                   float target_freq){
    if(g == NULL || param == NULL){
        return GIMBAL_ERR_NULL;
    }
    g ->param = param;
    g ->mode = GIMBAL_MODE_RELAX;

    //初始化云台电机
	  PID_Init(&(g->pid.yaw_angle), KPID_MODE_CALC_D, target_freq,
           &(g->param->pid.yaw_angle));
    PID_Init(&(g ->pid.yaw_omega), KPID_MODE_CALC_D, target_freq,
           &(g->param->pid.yaw_omega));
    PID_Init(&(g ->pid.pit_omega), KPID_MODE_CALC_D, target_freq,
           &(g->param->pid.pit_omega));           
    PID_Init(&(g ->pid.pit_angle), KPID_MODE_CALC_D, target_freq,
           &(g->param->pid.pit_angle));

    //低通滤波器初始化
    LowPassFilter2p_Init(&g->filter_out.yaw, target_freq,
                       g->param->low_pass_cutoff_freq.out);	
    LowPassFilter2p_Init(&g->filter_out.pit, target_freq,
                       g->param->low_pass_cutoff_freq.out);
    
    g->limit.pit.max = g->param->Limit_t.pit_max;
    g->limit.pit.min = -g->param->Limit_t.pit_max;
                  
    BSP_CAN_Init();
    MOTOR_RM_Register(&(g->param->motor.yaw_rm_motor));
    MOTOR_RM_Register(&(g->param->motor.pit_rm_motor));
    return 0;
}

static int8_t Gimbal_SetMode(Gimbal_t *g, Gimbal_Mode_t mode) {
  if (g == NULL)
    return -1;
  if (mode == g->mode)
    return GIMBAL_OK;

    PID_Reset(&g->pid.yaw_angle);
    PID_Reset(&g->pid.yaw_omega);	
    // PID_Reset(&g->pid.yaw_angle);
    // PID_Reset(&g->pid.yaw_omega);
    PID_Reset(&g->pid.pit_angle);
    PID_Reset(&g->pid.pit_omega);
    LowPassFilter2p_Reset(&g->filter_out.yaw, 0.0f);
    LowPassFilter2p_Reset(&g->filter_out.pit, 0.0f);

    AHRS_ResetEulr(&(g->feedback.imu.eulr)); /* 切换模式后重置设定值 */

    g->setpoint.eulr.pit = g->feedback.imu.eulr.pit;
    g->setpoint.eulr.yaw = g->feedback.imu.eulr.yaw;

    g->mode = mode;
    return 0;
}
int8_t Gimbal_UpdateFeedback(Gimbal_t *gimbal) {
      if (gimbal == NULL)
    return GIMBAL_ERR_NULL;

    MOTOR_RM_UpdateAll();
    MOTOR_RM_t *rm_motor_yaw = MOTOR_RM_GetMotor(&(gimbal->param->motor.yaw_rm_motor));
		if(rm_motor_yaw != NULL)
		 gimbal->feedback.motor.yaw = rm_motor_yaw->feedback;

    MOTOR_RM_t *rm_motor_pit = MOTOR_RM_GetMotor(&(gimbal->param->motor.pit_rm_motor));
		if(rm_motor_pit != NULL)
		 gimbal->feedback.motor.pit = rm_motor_pit->feedback;
    return GIMBAL_OK;
}

int8_t Gimbal_UpdateIMU(Gimbal_t *gimbal, const Gimbal_IMU_t *imu) {
    if (gimbal == NULL || imu == NULL)
        return GIMBAL_ERR_NULL;

    gimbal->feedback.imu.gyro = imu->gyro;
    gimbal->feedback.imu.eulr = imu->eulr;
    gimbal->feedback.imu.accl = imu->accl;
    return GIMBAL_OK;
}


int8_t Gimbal_Control(Gimbal_t *g, Gimbal_CMD_t *g_cmd) {
  if (g == NULL || g_cmd == NULL) {
    return GIMBAL_ERR_NULL;
  }
    g->dt = (BSP_TIME_Get_us() - g->lask_wakeup) / 1000000.0f;
    g->lask_wakeup = BSP_TIME_Get_us();
    Gimbal_SetMode(g, g_cmd->mode);

     /* 处理yaw控制命令，软件限位 - 使用电机绝对角度 */
    float delta_yaw = g_cmd->delta_yaw * g->dt * 1.0f;
  //   if (g->param->motor.limit_yaw == true) {
	//   float motor_imu_offset;
  //   switch (g->mode)
  //    {
  //   case GIMBAL_MODE_ABSOLUTE:
  //       motor_imu_offset = g->feedback.motor.major_yaw.rotor_abs_angle - g->feedback.imu.eulr.yaw;
  //       break;
  //   case GIMBAL_MODE_RELATIVE:
  //       motor_imu_offset = g->feedback.motor.major_yaw.rotor_abs_angle - g->feedback.imu.eulr.yaw;
  //       break;
  //   }
  //   /* 处理跨越±π的情况 */
  //  if (motor_imu_offset > M_PI) motor_imu_offset -= M_2PI;
  //  if (motor_imu_offset < -M_PI) motor_imu_offset += M_2PI;

  //      /* 计算到限位边界的距离 */
  //   const float delta_max = CircleError(g->limit.yaw.max,
  //       (g->setpoint.eulr.yaw + motor_imu_offset + delta_yaw), M_2PI);
  //   const float delta_min = CircleError(g->limit.yaw.min,
  //       (g->setpoint.eulr.yaw + motor_imu_offset + delta_yaw), M_2PI);
  //   /* 限制控制命令 */
  //   if (delta_yaw > delta_max) delta_yaw = delta_max;
  //   if (delta_yaw < delta_min) delta_yaw = delta_min;
  //   }
    CircleAdd(&(g->setpoint.eulr.yaw), delta_yaw, M_2PI);

  /* 处理pitch控制命令，软件限位 - 使用电机绝对角度 */
  float delta_pit = g_cmd->delta_pit * g->dt * 8.0f;
  if (g->param->motor.limit_pit == true) {
    /* 计算当前电机角度与IMU角度的偏差 */
    float motor_imu_offset;
    switch (g->mode) {
      case GIMBAL_MODE_ABSOLUTE:
        motor_imu_offset = g->feedback.motor.pit.rotor_abs_angle - g->feedback.imu.eulr.pit;
        break;
      case GIMBAL_MODE_RELATIVE:
        motor_imu_offset = g->feedback.motor.pit.rotor_abs_angle - g->feedback.imu.eulr.pit;
        break;
    }
    /* 处理跨越±π的情况 */
    if (motor_imu_offset > M_PI) motor_imu_offset -= M_2PI;
    if (motor_imu_offset < -M_PI) motor_imu_offset += M_2PI;
    
    /* 计算到限位边界的距离 */
    const float delta_max = CircleError(g->limit.pit.max,
        (g->setpoint.eulr.pit + motor_imu_offset + delta_pit), M_2PI);
    const float delta_min = CircleError(g->limit.pit.min,
        (g->setpoint.eulr.pit + motor_imu_offset + delta_pit), M_2PI);
    
    /* 限制控制命令 */
    if (delta_pit > delta_max) delta_pit = delta_max;
    if (delta_pit < delta_min) delta_pit = delta_min;
  }

  CircleAdd(&(g->setpoint.eulr.pit), delta_pit, M_2PI);


  float yaw_omega_set_point, pit_omega_set_point;
  switch (g->mode) {
  case GIMBAL_MODE_RELAX:
    g->out.yaw = 0.0f;
    g->out.pit = 0.0f;
    break;

  case GIMBAL_MODE_ABSOLUTE:
    yaw_omega_set_point = PID_Calc(&(g->pid.yaw_angle), g->setpoint.eulr.yaw,
                                   g->feedback.imu.eulr.yaw, 0.0f, g->dt);
    g->out.yaw = PID_Calc(&(g->pid.yaw_omega), yaw_omega_set_point,
                          g->feedback.imu.gyro.z, 0.f, g->dt);

    pit_omega_set_point = PID_Calc(&(g->pid.pit_angle), g->setpoint.eulr.pit,
                                   g->feedback.imu.eulr.pit, 0.0f, g->dt);
    g->out.pit = PID_Calc(&(g->pid.pit_omega), pit_omega_set_point,
                          g->feedback.imu.gyro.x, 0.f, g->dt);

    /* 输出滤波 */
    g->out.yaw = LowPassFilter2p_Apply(&g->filter_out.yaw, g->out.yaw);
    g->out.pit = LowPassFilter2p_Apply(&g->filter_out.pit, g->out.pit);
    break;
  
  // case GIMBAL_MODE_RELATIVE:
  //   yaw_omega_set_point = PID_Calc(&(g->pid.major_yaw_angle), g->setpoint.eulr.yaw,
  //                                  g->feedback.imu.eulr.yaw, 0.0f, g->dt);
  //   g->out.major_yaw = PID_Calc(&(g->pid.major_yaw_omega), yaw_omega_set_point,
  //                         g->feedback.imu.gyro.z, 0.0f, g->dt);

  //   pit_omega_set_point = PID_Calc(&(g->pid.pit_angle), g->setpoint.eulr.pit,
  //                                  g->feedback.imu.eulr.pit, 0.0f, g->dt);
  //   g->out.pit = PID_Calc(&(g->pid.pit_omega), pit_omega_set_point,
  //                         g->feedback.imu.gyro.x, 0.0f, g->dt);

  //   /* 输出滤波 */
  //   g->out.major_yaw = LowPassFilter2p_Apply(&g->filter_out.major_yaw, g->out.major_yaw);
  //   g->out.pit = LowPassFilter2p_Apply(&g->filter_out.pit, g->out.pit);
  //   break;
  }
    return 0;
}

void Gimbal_Output(Gimbal_t *g)
{   
	  MOTOR_RM_SetOutput(&g->param->motor.yaw_rm_motor, g->out.yaw);
    MOTOR_RM_Ctrl(&g->param->motor.yaw_rm_motor);

    MOTOR_RM_SetOutput(&g->param->motor.pit_rm_motor, g->out.pit);
    MOTOR_RM_Ctrl(&g->param->motor.pit_rm_motor);

  }