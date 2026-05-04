/*
 * 云台模组
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */

#include "bsp\can.h"
#include "bsp\time.h"
#include "component\pid.h"
#include "component\ahrs.h"
#include "component\filter.h"
#include "device\motor.h"
#include "device\motor_rm.h"
/* Exported constants ------------------------------------------------------- */
#define GIMBAL_OK (0)        /* 运行正常 */
#define GIMBAL_ERR (-1)      /* 运行时发现了其他错误 */
#define GIMBAL_ERR_NULL (-2) /* 运行时发现NULL指针 */
#define GIMBAL_ERR_MODE (-3) /* 运行时配置了错误的CMD_GimbalMode_t */

/* Exported macro ----------------------------------------------------------- */
/* Exported types ----------------------------------------------------------- */

typedef enum {
  GIMBAL_MODE_RELAX,    /* 放松模式，电机不输出。一般情况云台初始化之后的模式 */
  GIMBAL_MODE_ABSOLUTE, /* 绝对坐标系控制，控制在空间内的绝对姿态 */
  GIMBAL_MODE_RELATIVE, /* 相对坐标系控制，控制相对于底盘的姿态 */
} Gimbal_Mode_t;

typedef enum {
  GIMBAL_MODE_REMOTE,    /* 遥控器模式 */
  GIMBAL_MODE_NUC, 			 /* 操作手模式 */	
  GIMBAL_MODE_AI, 			 /* 自瞄模式 */
	
} Gimbal_Control_Mode_t;

typedef enum {
  Gyro_x,   
  Gyro_y, 		
	Gyro_z,
} Gimbal_Gyro_t;

typedef enum {
  Pit,   
  Yaw, 		
	Rol,
} Gimbal_Eulr_t;

typedef enum {
  Accl_x,   
  Accl_y, 		
	Accl_z,
} Gimbal_Accl_t;

typedef enum {
  Q0,   
  Q1, 		
	Q2,
	Q3,
} Gimbal_Quat_t;

typedef struct {
  Gimbal_Mode_t mode; 
  Gimbal_Control_Mode_t ctrl_mode; 	
  float delta_yaw;
  float delta_pit;
  float set_yaw;
  float set_pit;	
} Gimbal_CMD_t;

/* 软件限位 */
typedef struct {
  float max;
  float min;
} Gimbal_Limit_t;

typedef	struct{
	struct{
    float yaw; /*  零点行程 */
    float pit;
  }travel;
  float pit;		/*零点*/
  float yaw;
} Gimbal_zero_t;	

typedef struct {

	bool limit_yaw;/*是否开启限位*/
	bool limit_pit;

  MOTOR_RM_Param_t yaw_rm_motor; /* 大yaw轴电机参数 */	
  MOTOR_RM_Param_t pit_rm_motor; /* pitch轴电机参数 */
	
}Gimbal_MOTOR_t_Param_t;

/* 云台参数的结构体，包含所有初始化用的参数，通常是const，存好几组。*/
typedef struct {
	struct{
	float k;
	float p;	
	}acclctrl;

	Gimbal_MOTOR_t_Param_t motor;
  struct {
	  KPID_Params_t yaw_omega; /* yaw轴角速度环PID参数 */
    KPID_Params_t yaw_angle; /* yaw轴角位置环PID参数 */

    KPID_Params_t pit_omega; /* pitch轴角速度环PID参数 */
    KPID_Params_t pit_angle; /* pitch轴角位置环PID参数 */
  } pid;

	  /* 前馈系数 */
  struct {
		struct{
		bool yaw;
		bool pit;
		float K_yaw;  
    float K_pit;			
		}imu;
  }feedforward;	
  /* 低通滤波器截止频率 */
  struct {
    float out;  /* 电机输出 */
    float gyro; /* 陀螺仪数据 */
  } low_pass_cutoff_freq;

	struct {
  float pit_max;		/*pit的限位*/
  float pit_min;
  float yaw_max;		/*yaw的限位*/
  float yaw_min;		
		struct {
			float pit_encoder;		/*零点*/
			float yaw_encoder;
              }zero;
		struct{	
				float yaw;
				float pit;
			        }travel;	
	}Limit_t;
	
	
} Gimbal_Params_t;

typedef struct {
  AHRS_Gyro_t gyro;
  AHRS_Eulr_t eulr;
  AHRS_Quaternion_t quat;		
  AHRS_Accl_t accl;	
} Gimbal_IMU_t;
/* 云台反馈数据的结构体，包含反馈控制用的反馈数据 */
typedef struct {
  Gimbal_IMU_t imu;
  struct {
    MOTOR_Feedback_t yaw; /* yaw轴电机反馈 */
    MOTOR_Feedback_t pit; /* pitch轴电机反馈 */
  } motor;
} Gimbal_Feedback_t;

/* 云台输出数据的结构体*/
typedef struct {
  float yaw;	
  float pit; /* pitch轴电机输出 */
} Gimbal_Output_t;
/*
 * 运行的主结构体，所有这个文件里的函数都在操作这个结构体。
 * 包含了初始化参数，中间变量，输出变量。
 */
typedef struct {
  uint64_t lask_wakeup;
  float dt;

  Gimbal_Params_t *param; /* 云台的参数，用Gimbal_Init设定 */
	
  /* 模块通用 */
	Gimbal_Control_Mode_t ctrl_mode;
  Gimbal_Mode_t mode; /* 云台模式 */
	Gimbal_zero_t zero;
  /* PID计算的目标值 */
  struct {
    AHRS_Eulr_t eulr; /* 表示云台姿态的欧拉角 */
		AHRS_Eulr_t ecd;
  } setpoint;

  struct {
    KPID_t yaw_angle; /* yaw轴角位置环PID */
    KPID_t yaw_omega; /* yaw轴角速度环PID */
    KPID_t pit_angle; /* pitch轴角位置环PID */
    KPID_t pit_omega; /* pitch轴角速度环PID */	
  } pid;

  struct {
	  Gimbal_Limit_t yaw;
    Gimbal_Limit_t pit;
  } limit;

  struct {		
		float yaw;
		float pit;
  } relative_angle;
	
  struct {	
    LowPassFilter2p_t yaw;
    LowPassFilter2p_t pit;
  } filter_out;

  Gimbal_Output_t out;        /* 云台输出 */
  Gimbal_Feedback_t feedback; /* 反馈 */

} Gimbal_t;

/* Exported functions prototypes -------------------------------------------- */

/**
 * \brief 初始化云台
 *
 * \param g 包含云台数据的结构体
 * \param param 包含云台参数的结构体指针
 * \param target_freq 任务预期的运行频率
 *
 * \return 函数运行结果
 */
int8_t Gimbal_Init(Gimbal_t *g,Gimbal_Params_t *param,
                   float target_freq);

/**
 * \brief 通过CAN设备更新云台反馈信息
 *
 * \param gimbal 云台
 * \param can CAN设备
 *
 * \return 函数运行结果
 */

int8_t Gimbal_UpdateFeedback(Gimbal_t *gimbal);

int8_t Gimbal_UpdateIMU(Gimbal_t *gimbal, const Gimbal_IMU_t *imu);
/**
 * \brief 运行云台控制逻辑
 *
 * \param g 包含云台数据的结构体
 * \param fb 云台反馈信息
 * \param g_cmd 云台控制指令
 * \param dt_sec 两次调用的时间间隔
 *
 * \return 函数运行结果
 */
int8_t Gimbal_Control(Gimbal_t *g, Gimbal_CMD_t *g_cmd);

/**
 * \brief 云台输出
 *
 * \param s 包含云台数据的结构体
 * \param out CAN设备云台输出结构体
 */
void Gimbal_Output(Gimbal_t *g);

#ifdef __cplusplus
}
#endif
