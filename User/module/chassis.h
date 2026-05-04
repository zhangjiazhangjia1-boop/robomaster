/*
底盘头文件
 */
#pragma once


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include <cmsis_os2.h>
#include "bsp/can.h"
#include "component\filter.h"
#include "component\mixer.h"
#include "component\pid.h"
#include "component\ahrs.h"
#include "device/motor_rm.h"
/* Exported constants ------------------------------------------------------- */
#define CHASSIS_OK (0)        /*运行正常*/
#define CHASSIS_ERR (-1)      /*运行错误*/
#define CHASSIS_ERR_NULL (-2) /*空指针错误*/
#define CHASSIS_ERR_MODE (-3) /*模式错误*/
#define CHASSIS_ERR_TYPE (-4) /*类型错误*/
	
#define MAX_MOTOR_CURRENT 20.0f 
/* 底盘控制模式 */
typedef enum {
  CHASSIS_MODE_RELAX, /*放松模式，电机不转动*/
  CHASSIS_MODE_BREAK, /* 制动模式，电机立即停止 */
  CHASSIS_MODE_FOLLOW_GIMBAL, /* 跟随模式，电机根据云台角度旋转 */
  CHASSIS_MODE_FOLLOW_GIMBAL_35, /* 跟随模式，电机根据云台角度旋转35度 */
  CHASSIS_MODE_ROTOR, /* 旋转模式，电机根据旋转角度旋转 */
  CHASSIS_MODE_INDEPENDENT,/* 独立模式，电机根据指令独立旋转 */
  CHASSIS_MODE_OPEN, /* 开放模式，电机根据指令直接旋转 */
} Chassis_Mode_t;
	
/* 底盘小陀螺模式 */
typedef enum {
  ROTOR_MODE_CW,  // 顺时针旋转
  ROTOR_MODE_CCW,  // 逆时针旋转
  ROTOR_MODE_RAND, // 随机旋转
} Chassis_RotorMode_t;

/* 底盘控制命令 */
typedef struct {
  Chassis_Mode_t mode;     /* 底盘运动模式*/
  Chassis_RotorMode_t mode_rotor; /* 小陀螺模式旋转方向*/
  MoveVector_t ctrl_vec;  /* 底盘运动控制向量*/
} Chassis_CMD_t;

	/* 底盘类型 */
typedef enum {
  CHASSIS_TYPE_MECANUM,    /* 麦克纳姆轮底盘 */
  CHASSIS_TYPE_PARLFIX4,   /* 平行固定4轮底盘 */
  CHASSIS_TYPE_PARLFIX2,   /* 平行固定2轮底盘 */
  CHASSIS_TYPE_OMNI_CROSS, /* 十字.omni轮底盘 */
  CHASSIS_TYPE_OMNI_PLUS,  /* X型.omni轮底盘 */
  CHASSIS_TYPE_DRONE,     /* 无人机底盘 */
  CHASSIS_TYPE_SINGLE,     /* 单轮底盘 */
} Chassis_Type_t;


/* 底盘参数结构体 ，初始化参数*/
typedef struct {
	  MOTOR_RM_Param_t motor_param[4]; /* 电机参数 */
	  struct {
    KPID_Params_t motor_pid_param;  /* 电机转速PID参数 */
    KPID_Params_t follow_pid_param; /* 跟随PID参数 */
  } pid;
  Chassis_Type_t type; /* 底盘类型 */

/* 低通滤波器截止频率设置 */
  struct {
    float in;  /* 输入 */
    float out; /* 输出 */
  } low_pass_cutoff_freq;
  /* 限制 */
	struct {
        float max_vx, max_vy, max_wz;
        float max_current; 
    } limit;  

  float mech_zero;/*云台6020的机械中点*/
} Chassis_Params_t;

// typedef struct {
//   AHRS_Gyro_t gyro;
//   AHRS_Eulr_t eulr;
// } Chassis_IMU_t;

typedef struct {
   /* 电机反馈 */
  MOTOR_Feedback_t motor[4];          
  float gimbal_yaw_encoder;
} Chassis_Feedback_t;

/* 底盘输出结构体 */
typedef struct {
	float motor[4];  /* 电机输出电流目标值，范围[-1,1],对应最大电流 */
} Chassis_Output_t;

// typedef struct {
// 	float chassis_mech_zero_yaw;
// 	float chassis_encoder_yaw;
// } chassis_ctrl_eulr_t;

	typedef struct
	{
		float vx;
		float vy;
		float wz;
	} ChassisMove_Vec;

/* 底盘主结构体 */
typedef struct {
  uint64_t last_wakeup;
  float dt;
  uint8_t keeping_angle_flag; // 是否处于保持角度模式

//  chassis_ctrl_eulr_t eulr; /* 底盘控制用欧拉角 */
  Chassis_Params_t *param; /* 底盘参数指针 */
  /* 底盘状态 */
  Chassis_Mode_t mode; /* 底盘运动模式 */
  /* 底盘设计 */
  int8_t num_wheel; /* 底盘轮子数量 */
  ChassisMove_Vec move_vec; /* 底盘运动控制向量 */
  MOTOR_RM_t *motors[4]; /* 电机指针数组 */
	float mech_zero; /* 机械零点 */
  float wz_multi; /* 小陀螺模式旋转角度系数 */

  /*PID计算目标值*/
  struct {
    float motor_rpm_in[4]; /* 电机转速目标值，单位rpm */
    float motor_rpm_out[4]; /* 电机转速输出值，单位rpm */
  } setpoint;

/* 反馈控制PID计算 */
  struct {
    KPID_t motor[4]; /* 电机转速PID */
    KPID_t follow; /* 跟随PID */
  } pid;

  /*滤波器*/
  struct {
    LowPassFilter2p_t in[3];  /* 电机速度反馈滤波器 */
    LowPassFilter2p_t out[4]; /* 电机输出滤波器 */
  } filter;
  Chassis_Output_t out;          /* 底盘输出结构体 */
  Chassis_Feedback_t feedback;
} Chassis_t;

/* Exported functions prototypes -------------------------------------------- */

/**
 * \brief 底盘初始化
 *
 * \param c 底盘结构体指针
 * \param param 底盘参数指针
 * \param target_freq 目标频率，单位Hz
 *
 * \return 运行结果，CHASSIS_OK:成功
 */
int8_t Chassis_Init(Chassis_t *c, const Chassis_Params_t *param,
                    float target_freq);

/**
 * \brief 更新底盘反馈
 *
 * \param c 底盘结构体指针
 *
 * \return 运行结果，CHASSIS_OK:成功
 */
int8_t Chassis_UpdateFeedback(Chassis_t *c);

/**
 * \brief 底盘控制
 *
 * \param c 底盘结构体指针
 * \param c_cmd 底盘控制命令指针
 * \param now 当前时间戳，单位ms
 *
 * \return 运行结果，CHASSIS_OK:成功
 */
int8_t Chassis_Control(Chassis_t *c, const Chassis_CMD_t *c_cmd,
                       uint32_t now);


/**
  * \brief 底盘输出
  * \param c 底盘结构体指针
 */

void Chassis_Output(Chassis_t *c);

/**
 * \brief 底盘输出重置
 *
 * \param c 底盘结构体指针
 */
// void Chassis_ResetOutput(Chassis_t *c);

/**
 * \brief 底盘功率限制
 *
 * \param c 底盘结构体指针
 * \param cap 电容数据指针
 * \param ref 比赛数据指针
 *
 * \return 运行结果，CHASSIS_OK:成功
 */
//<2F><>û<EFBFBD>м<EFBFBD><D0BC>룬waiting<6E><67><EFBFBD><EFBFBD><EFBFBD><EFBFBD><EFBFBD><EFBFBD><EFBFBD><EFBFBD><EFBFBD><EFBFBD>int8_t Chassis_PowerLimit(Chassis_t *c, const CAN_Capacitor_t *cap,
//                          const Referee_ForChassis_t *ref);


//底盘速度计算函数，计算运动向量并进行运动学逆解算得到电机转速目标值
void Chassis_speed_calculate(Chassis_t *c, Chassis_CMD_t *c_cmd);


/**
 * \brief 底盘UI数据填充
 *
 * \param c 底盘结构体指针
 * \param ui 底盘UI数据指针
 *
 * \return 无
 */
//void Chassis_DumpUI(const Chassis_t *c, Referee_ChassisUI_t *ui);


#ifdef __cplusplus
}
#endif