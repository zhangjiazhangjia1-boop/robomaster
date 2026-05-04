/*
    atti_esti Task
    
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "bsp/mm.h"
#include "bsp/pwm.h"
#include "bsp/gpio.h"
#include "component/ahrs.h"
#include "component/pid.h"
#include "device/bmi088.h"
#include "module/gimbal.h"
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
BMI088_t bmi088;
AHRS_t gimbal_ahrs;
KPID_t imu_temp_ctrl_pid;
Gimbal_IMU_t gimbal_to_send;
AHRS_Magn_t magn;
AHRS_Eulr_t eulr_to_send;
BMI088_Cali_t cali_bmi088= {
  .gyro_offset = {-0.00147764047f,-0.00273479894f,0.00154074503f},
};


static const KPID_Params_t imu_temp_ctrl_pid_param = {
    .k = 0.3f,
    .p = 1.0f,
    .i = 0.01f,
    .d = 0.0f,
    .i_limit = 1.0f,
    .out_limit = 1.0f,
};

/* 校准相关变量 */
typedef enum {
  CALIB_IDLE,     // 空闲状态
  CALIB_RUNNING,  // 正在校准
  CALIB_DONE      // 校准完成
} CalibState_t;

static CalibState_t calib_state = CALIB_IDLE;
static uint16_t calib_count = 0;
static struct {
  float x_sum;
  float y_sum; 
  float z_sum;
} gyro_sum= {0.0f,0.0f,0.0f};

static void start_gyro_calibration(void) {
  if (calib_state == CALIB_IDLE) {
    calib_state = CALIB_RUNNING;
    calib_count = 0;
    gyro_sum.x_sum = 0.0f;
    gyro_sum.y_sum = 0.0f;
    gyro_sum.z_sum = 0.0f;
  }
}
#define CALIB_SAMPLES 5000  // 校准采样数量
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */
void Task_atti_esti(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  
  /* 计算任务运行到指定频率需要等待的tick数 */
  const uint32_t delay_tick = osKernelGetTickFreq() / ATTI_ESTI_FREQ;

  osDelay(ATTI_ESTI_INIT_DELAY); /* 延时一段时间再开启任务 */

  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  /* USER CODE INIT BEGIN */
  BMI088_Init(&bmi088, &cali_bmi088);
  AHRS_Init(&gimbal_ahrs, &magn, BMI088_GetUpdateFreq(&bmi088));
  PID_Init(&imu_temp_ctrl_pid, KPID_MODE_NO_D,  1.0f / BMI088_GetUpdateFreq(&bmi088), &imu_temp_ctrl_pid_param);
  BSP_PWM_Start(BSP_PWM_IMU_HEAT_PWM);
  /* USER CODE INIT END */
  
  while (1) {
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* USER CODE BEGIN */
	BMI088_WaitNew();
    BMI088_AcclStartDmaRecv();
    BMI088_AcclWaitDmaCplt();

    BMI088_GyroStartDmaRecv();
    BMI088_GyroWaitDmaCplt();

    /* 锁住RTOS内核防止数据解析过程中断，造成错误 */
    osKernelLock();
    /* 接收完所有数据后，把数据从原始字节加工成方便计算的数据 */
	  BMI088_ParseAccl(&bmi088);
	  BMI088_ParseGyro(&bmi088);
  	/* 根据设备接收到的数据进行姿态解析 */
    AHRS_Update(&gimbal_ahrs, &bmi088.accl, &bmi088.gyro, &magn);

    /* 根据解析出来的四元数计算欧拉角 */
    AHRS_GetEulr(&eulr_to_send, &gimbal_ahrs);
	  osKernelUnlock();
    
    /* 陀螺仪校准处理 */
    if (calib_state == CALIB_RUNNING) {
      /* 累加陀螺仪数据用于计算零偏 */
      gyro_sum.x_sum += bmi088.gyro.x;
      gyro_sum.y_sum += bmi088.gyro.y;
      gyro_sum.z_sum += bmi088.gyro.z;
      calib_count++;

      /* 达到采样数量后计算平均值并更新零偏 */
      if (calib_count >= CALIB_SAMPLES) {
        /* 计算平均值作为零偏 */
        cali_bmi088.gyro_offset.x = gyro_sum.x_sum / CALIB_SAMPLES;
        cali_bmi088.gyro_offset.y = gyro_sum.y_sum / CALIB_SAMPLES;
        cali_bmi088.gyro_offset.z = gyro_sum.z_sum / CALIB_SAMPLES;

        /* 校准完成，重置为空闲状态以便下次校准 */
        calib_state = CALIB_IDLE;

        /* 重新初始化BMI088以应用新的校准数据 */
        BMI088_Init(&bmi088, &cali_bmi088);
        AHRS_Init(&gimbal_ahrs, &magn, BMI088_GetUpdateFreq(&bmi088));
	    }
	  }
		  BSP_PWM_SetComp(BSP_PWM_IMU_HEAT_PWM, PID_Calc(&imu_temp_ctrl_pid, 40.0f, bmi088.temp, 0.0f, 0.0f));
      gimbal_to_send.eulr = eulr_to_send;
      gimbal_to_send.gyro = bmi088.gyro;
      gimbal_to_send.accl = bmi088.accl;
      gimbal_to_send.quat = gimbal_ahrs.quat;
      osMessageQueueReset(task_runtime.msgq.gimbal.imu);
      osMessageQueuePut(task_runtime.msgq.gimbal.imu, &gimbal_to_send, 0, 0);
	  
    /* USER CODE END */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
  
}