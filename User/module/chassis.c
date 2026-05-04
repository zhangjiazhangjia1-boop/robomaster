/*
底盘模组
*/

#include "cmsis_os2.h"
#include <stdlib.h>
#include "bsp/mm.h"
#include "bsp/can.h"
#include "component/ahrs.h"
#include "device/motor_rm.h"
#include "device/motor.h"
#include "module/chassis.h"

int8_t Chassis_Init(Chassis_t *c, const Chassis_Params_t *param,float target_freq) 
{
    if (!c) return CHASSIS_ERR_NULL;  

    BSP_CAN_Init();
    c->param = (Chassis_Params_t *)param;  // 修复：移除const限定符
    c->mode = CHASSIS_MODE_RELAX;
    c->mech_zero = c->param->mech_zero;/*云台6020的机械中点*/
    c->num_wheel = 4;

    //初始化时间戳
	c->last_wakeup = 0;
	c->dt = 0.0f;
	//初始化PID和滤波
    for (uint8_t i = 0; i < c->num_wheel; i++) {
        PID_Init(&c->pid.motor[i], KPID_MODE_NO_D, target_freq, &param->pid.motor_pid_param);
        LowPassFilter2p_Init(&c->filter.in[i], target_freq, param->low_pass_cutoff_freq.in);
        LowPassFilter2p_Init(&c->filter.out[i], target_freq, param->low_pass_cutoff_freq.out);
		//清零电机反馈
		c->feedback.motor[i].rotor_speed = 0;
        c->feedback.motor[i].torque_current = 0;
        c->feedback.motor[i].rotor_abs_angle = 0;
        c->feedback.motor[i].temp = 0;
        }
        //初始化PID
        PID_Init(&c->pid.follow, KPID_MODE_NO_D, target_freq, &param->pid.follow_pid_param);
        //清零运动向量和输出
        c->move_vec.vx = c->move_vec.vy = c->move_vec.wz = 0.0f;
        for (uint8_t i = 0; i < c->num_wheel; i++) { 
				c->out.motor[i] = 0.0f;
		    }
		//注册大疆电机
		for (int i = 0; i < c->num_wheel; i++) {
			MOTOR_RM_Register(&(c->param->motor_param[i]));
			 
		}
            MOTOR_RM_Register(&(c->param-> motor_param[4])); /* 注册云台编码器 */
    return CHASSIS_OK;
}



void Chassis_speed_calculate(Chassis_t *c, Chassis_CMD_t *c_cmd) {
    if (!c || !c_cmd) return;
    
    float angle = M_PI / 4.0f; // 45度，十字全向轮布局
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);
    float wheel_radius = 0.05f; // 轮子半径（m）
    float gear_ratio = 18.0f;    // 减速比   
    float scale_factor = 2000.0f / (2 * M_PI * wheel_radius * gear_ratio); // 单位转换系数
    float to_center = 2.0f; // 轮子到中心的距离（m），根据实际底盘设计调整
    
    float gimbal_relative_angle = 0.0f; // 提前声明变量
    
    switch (c->mode) {

    
        case CHASSIS_MODE_RELAX:
        c->move_vec.vx = c->move_vec.vy = c->move_vec.wz = 0.0f;
        break;
        case CHASSIS_MODE_BREAK:
        c->move_vec.vx = c->move_vec.vy = c->move_vec.wz = 0.0f;
        break;
        case CHASSIS_MODE_INDEPENDENT:
            // 独立模式的解算
            if (c->num_wheel == 4) {
            c->move_vec.vx = c_cmd->ctrl_vec.vx ;
            c->move_vec.vy = c_cmd->ctrl_vec.vy ;
            c->move_vec.wz = 0.0f;
             } else {
//        goto error;
      }
            break;


        case CHASSIS_MODE_FOLLOW_GIMBAL:
            // 跟随云台模式的解算

            gimbal_relative_angle = c->feedback.gimbal_yaw_encoder - c->mech_zero;
            // 限制角度在[-π, π]范围内
            while (gimbal_relative_angle > M_PI) gimbal_relative_angle -= 2 * M_PI;
            while (gimbal_relative_angle < -M_PI) gimbal_relative_angle += 2 * M_PI;

            c->move_vec.wz = 10*PID_Calc(&c->pid.follow, 0.0f,  gimbal_relative_angle, 0.0f, c->dt); 
            // 使用PID计算旋转速度

            c->move_vec.vx = c_cmd->ctrl_vec.vx ;
            c->move_vec.vy = c_cmd->ctrl_vec.vy ;

            

  
            break;
            default:
            // 其他模式的解算
            // ... 原有代码
            break;
                }
			/*给输出的Vx，Vy，Vw进行滤波*/
            c->move_vec.vx = LowPassFilter2p_Apply(&c->filter.in[0], c->move_vec.vx);
            c->move_vec.vy = LowPassFilter2p_Apply(&c->filter.in[1], c->move_vec.vy);
            c->move_vec.wz = LowPassFilter2p_Apply(&c->filter.in[2], c->move_vec.wz);
            // 计算电机转速（全向轮解算）
			c->setpoint.motor_rpm_in[0] = scale_factor * (-cos_angle * c->move_vec.vx + sin_angle * c->move_vec.vy + c->move_vec.wz * to_center);
            c->setpoint.motor_rpm_in[1] = scale_factor * (-cos_angle * c->move_vec.vx - sin_angle * c->move_vec.vy + c->move_vec.wz * to_center);
            c->setpoint.motor_rpm_in[2] = scale_factor * (cos_angle * c->move_vec.vx - sin_angle * c->move_vec.vy + c->move_vec.wz * to_center);
            c->setpoint.motor_rpm_in[3] = scale_factor * (cos_angle * c->move_vec.vx + sin_angle * c->move_vec.vy + c->move_vec.wz * to_center);
            //进行pid计算
            for (uint8_t i = 0; i < c->num_wheel; i++) {
            c->setpoint.motor_rpm_out[i] = PID_Calc(&c->pid.motor[i], c->setpoint.motor_rpm_in[i], c->feedback.motor[i].rotor_speed, 0.0f, c->dt);
            //对输出进行滤波
             c->out.motor[i] = 5*   LowPassFilter2p_Apply(&c->filter.out[i], c->setpoint.motor_rpm_out[i]);
    }

           
//error:
//            for (uint8_t i = 0; i < c->num_wheel; i++) c->setpoint.motor_rpm_out[i] = 0;
            return; // 修复：void函数不能返回值
        }



/**
 * @brief 底盘电机控制
 * @param c 底盘结构体指针
 * @param c_cmd 控制命令
 * @param now 当前时间戳(ms)
 * @return CHASSIS_OK:成功 CHASSIS_ERR_NULL:空
 */
int8_t Chassis_Control(Chassis_t *c, const Chassis_CMD_t *c_cmd, uint32_t now) {
    if (!c || !c_cmd) return CHASSIS_ERR_NULL;
    //计算控制周期
    c->dt = (float)(now - c->last_wakeup) / 1000.0f; 
    c->last_wakeup = now;
		if (!isfinite(c->dt) || c->dt <= 0.0f) {
			c->dt = 0.001f;            
		}
		if (c->dt < 0.0005f) c->dt = 0.0005f;   
		if (c->dt > 0.050f)  c->dt = 0.050f;
    //设置模式
    //重置PID和滤波
    for (uint8_t i = 0; i < c->num_wheel; i++) {
        PID_Reset(&c->pid.motor[i]);
        LowPassFilter2p_Reset(&c->filter.in[i], 0.0f);
        LowPassFilter2p_Reset(&c->filter.out[i], 0.0f);
    }
        c->mode = c_cmd->mode;
         if (c->mode != c_cmd->mode)
    {
        return CHASSIS_ERR_MODE; /* 设置模式失败 */
    }
    Chassis_speed_calculate(c, (Chassis_CMD_t *)c_cmd); // 修复：移除const限定符

    return CHASSIS_OK;
}

int8_t Chassis_UpdateFeedback(Chassis_t *c) {
    if (!c) return CHASSIS_ERR_NULL;
    
    MOTOR_RM_UpdateAll();
      /*更新所有电机数据*/
    for (uint8_t i = 0; i < c->num_wheel; i++) {
			MOTOR_RM_t *rm_motor = MOTOR_RM_GetMotor(&(c->param->motor_param[i]));
			c->motors[i] = rm_motor;
			MOTOR_RM_t *rm = c->motors[i];
         if (rm_motor != NULL) {
            c->feedback.motor[i] = rm_motor->feedback;
             }else 
					{ 
					return CHASSIS_ERR_NULL; 
					} 
		}

    return CHASSIS_OK;
}

void Chassis_Output(Chassis_t *c) {
    if (!c) 
			return ;

    for (uint8_t i = 0; i < c->num_wheel; i++) {
        MOTOR_RM_t *rm = c->motors[i];
        if (!rm) continue;
        MOTOR_RM_SetOutput(&rm->param, c->out.motor[i]);
    }

    MOTOR_RM_t *rm = c->motors[0];
    if (rm) {
        MOTOR_RM_Ctrl(&rm->param);
    }
}