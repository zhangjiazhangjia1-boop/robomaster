/*
 * 配置相关
 */

/* Includes ----------------------------------------------------------------- */
#include "module/config.h"
#include "component/user_math.h"
#include "bsp\can.h"


/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */

/* Exported variables ------------------------------------------------------- */

/**
 * @brief 机器人参数配置
 * @note 在此配置机器人参数
 */
Config_RobotParam_t robot_config = {
    /* USER CODE BEGIN robot_config */


		.chassis_param = {
    /* DJI3508电机*/
    .motor_param = {
      { 
			.can = BSP_CAN_1, 
			.id = 0x204, 
			.module = MOTOR_M2006, 
			.reverse = false,
			.gear =  false
			},
      { 
			.can = BSP_CAN_1,
			.id = 0x202, 
			.module = MOTOR_M3508, 
			.reverse = false,
			.gear =  true
			},
      { 
			.can = BSP_CAN_1, 
			.id = 0x203, 
			.module = MOTOR_M3508, 
			.reverse = false,
			.gear =  true				
			},
      { 
			.can = BSP_CAN_1,
			.id = 0x204, 
			.module = MOTOR_M3508, 
			.reverse = false,
			.gear =  true	
			},
    },

    /* PID  */
    .pid = {
      /* 底盘电机 PID */
      .motor_pid_param = {
				.k = 0.05f,
				.p = 1.0f,
				.i = 0.5f,
				.d = 0.0f,
				.i_limit = 1.0f,
				.out_limit = 1.0f,
				.d_cutoff_freq = -1.0f,
				.range = -1.0f,
      },

      /* 跟随 */
      .follow_pid_param = {
					.k = 0.5f,
					.p = 1.0f,
					.i = 0.5f,
					.d = 0.0f,
					.i_limit = 1.0f,
					.out_limit = 1.0f,
					.d_cutoff_freq = -1.0f,
					.range = M_2PI,
      },
    },
	.type = CHASSIS_TYPE_MECANUM,
    .low_pass_cutoff_freq = {
      .in = 50.0f,
      .out = 50.0f,
    },
	.limit = {
      .max_vx = 3.0f,     
      .max_vy = 3.0f,
      .max_wz = 2.0f,    
      .max_current = 5000.0f 
    },
    .mech_zero = 4.5f, /*云台6020的机械中点*/
  },



    /* 云台欧拉角与角速度自由选择 */
.gimbal_param = {
	/*欧拉角限位和电机角度限位*/
	 .Limit_t= {
				.pit_max= 0.462614686,
				.pit_min=-0.518002629,
		 			/*零点参数*/
				.zero={
					.yaw_encoder=1.26,
					
									
					},
				.travel={
						.yaw=1.5f,
						.pit = 0.6f,
					},
			},
	 

			.feedforward={
				.imu = {
					.yaw=false,
					.pit=false,
					},

			},		
	.motor={
			.limit_yaw=false,
			.limit_pit=true,
			.pit_rm_motor={BSP_CAN_2,0x20A,MOTOR_GM6020,true,false},
			.yaw_rm_motor={BSP_CAN_2,0x209,MOTOR_GM6020,false,false},	
			},

			
    .low_pass_cutoff_freq = {
      .out = -1.0f,
      .gyro = 1000.0f,
    },

       .pid = {
					/*欧拉角控制参数*/
             .yaw_omega = {
                .k = 0.45f,
                .p = 1.0f,
                .i = 6.0f,
                .d = 0.0008f,
                .i_limit = 1.0f,
                .out_limit = 1.0f,
                .d_cutoff_freq = -1.0f,
                .range = -1.0f,
            },
            .yaw_angle = {
                .k = 1.0f,
                .p =2.0f ,
                .i =0.0f,
                .d = 0.0f,
                .i_limit = 0.0f,
                .out_limit = 10.0f,
                .d_cutoff_freq = -1.0f,
                .range = M_2PI,
            },
            .pit_omega = {
                .k = 0.25f,
                .p = 1.0f,
                .i = 0.0f,
                .d = 0.001901f,
                .i_limit = 1.0f,
                .out_limit = 1.0f,
                .d_cutoff_freq = -1.0f,
                .range = -1.0f,
            },
            .pit_angle = {
				.k = 2.0f,
				.p = 1.0f,
				.i = 2.5f,
				.d = 0.0f,
				.i_limit = 0.0f,
				.out_limit = 10.0f,
				.d_cutoff_freq = -1.0f,
				.range = M_2PI,
            },		
			}        
		},
.shoot_param = {
        .basic={
          	.projectileType=SHOOT_PROJECTILE_17MM,
			.fric_num=2,
			.extra_deceleration_ratio=1.0f,
          	.num_trig_tooth=5,
          	.shot_freq=1.0f,
          	.shot_burst_num=3,
        },   
        .jamDetection={
            .enable=true,
            .threshold=120.0f, 
            .suspectedTime=0.5f, 
        },
        .motor={
            .fric = {
                {
                        .can = BSP_CAN_2,
                        .id = 0x205,
                        .module = MOTOR_M3508,
                        .reverse = true,
                        .gear = false,
                    },
                {
                        .can = BSP_CAN_2,
                        .id = 0x206,
                        .module = MOTOR_M3508,
                        .reverse = false,
                        .gear = false,
                    },

                },
            .trig = {
                .can = BSP_CAN_2,
                .id = 0x207,
                .module = MOTOR_M2006,
                .reverse = true,
                .gear=true,
            },
        },
        .pid={      
            .fric_follow = {
                .k=1.0f,
                .p=1.5f,
                .i=0.3f,
                .d=0.0f,
                .i_limit=0.2f,
                .out_limit=0.9f, 
                .d_cutoff_freq=-1.0f,
                .range=-1.0f,
            },
            .trig_2006 = {
                .k=2.5f,
                .p=1.0f,
                .i=0.1f,
                .d=0.04f,
                .i_limit=0.4f,
                .out_limit=1.0f,
                .d_cutoff_freq=-1.0f,
                .range=M_2PI,
            },
            .trig_omg_2006 = {
                .k=1.0f,
                .p=1.5f,
                .i=0.3f,
                .d=0.5f,
                .i_limit=0.2f,
                .out_limit=1.0f,
                .d_cutoff_freq=-1.0f,
                .range=-1.0f,
            },
        },
		.filter={
            .fric = {
                .in = 30.0f,
                .out = 30.0f,
            },
            .trig = {
                .in = 30.0f,
                .out = 30.0f,
            },
        },  
    },
.cmd_param = { 
    /* 灵敏度设置 */
    .sensitivity = {
        .mouse_sens = 0.8f,
        .move_sens = 1.0f,
        .move_fast_mult = 1.5f,
        .move_slow_mult = 0.5f,
    },
    
    /* RC拨杆模式映射 */
    .rc_mode_map = {
        /* 左拨杆控制底盘模式 */
        .sw_left_up   = CHASSIS_MODE_ROTOR,
        .sw_left_mid  = CHASSIS_MODE_FOLLOW_GIMBAL,
        .sw_left_down = CHASSIS_MODE_BREAK ,
        
        /* 用于云台模式 */
        .gimbal_sw_up   = GIMBAL_MODE_ABSOLUTE,
        .gimbal_sw_mid  = GIMBAL_MODE_ABSOLUTE,
        .gimbal_sw_down = GIMBAL_MODE_RELATIVE,
    },
    
    }
};
    
    // 在此添加您的配置参数初始化
    
    /* USER CODE END robot_config */


/* Private function prototypes ---------------------------------------------- */
/* Exported functions ------------------------------------------------------- */




/**
 * @brief 获取机器人配置参数
 * @return 机器人配置参数指针
 */
Config_RobotParam_t* Config_GetRobotParam(void) {
    return &robot_config;
}