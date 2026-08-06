#ifndef ALLDATA_H_
#define ALLDATA_H_

/* ================= 1. C 语言标准库 ================= */
#include <stdint.h>     // 用于标准数据类型
#include <stdbool.h>    // 用于 bool 类型
#include <math.h>       // 用于数学函数计算 (mahony, pid_ctrl)
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

/* ================= 2. TI SDK 底层配置 ================= */
#include "ti_msp_dl_config.h"

/* ================= 3. FreeRTOS 系统库 ================= */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ================= 4. 用户自定义模块头文件 ================= */
// rtos_tasks.h 必须最先引入 — 它定义了所有 USE_XXX 宏开关,
// 后续各外设头文件依赖这些宏来做条件编译
#include "rtos_tasks.h"
#include "Delay.h"
// #include "encoder.h"
#include "lsm6dsr.h"
#include "mahony.h"
#include "motor.h"
#include "pid_ctrl.h"
#include "encoder.h"
#include "uart.h"
#include "Key.h"
#include "ultrasonic.h"
#include "Sensor.h"
#include "OLED.h"
#include "OLED_Data.h"
#include "hc05.h"
#include "ZDT_X42S.h"
#include "vofa.h"


/* ================= 5. 全局共享变量声明 =================
 * 使用 extern 声明，将 rtos_tasks.c 中解算好的姿态角暴露给全工程。
 * 这样任何包含了 alldata.h 的文件，都能直接读取最新的欧拉角，
 * 用来喂给 PID 算法进行闭环控制。
 */
extern volatile float roll;
extern volatile float pitch;
extern volatile float yaw;

/* 控制模式 (rtos_tasks.c 定义) */
extern volatile uint8_t g_ctrl_mode;  // 0=非循迹(停止), 1=循迹

/* ================= 6. Debug 观测变量 (CCS Expressions 窗口) ================= */
#if USE_IMU_SENSOR
extern volatile float   debug_gyro_dps[3];
extern volatile float   debug_accel_g[3];
extern volatile float   debug_euler_deg[3];
extern volatile int32_t debug_loop_count;
extern volatile int     debug_static_state;
extern volatile float   debug_dt_ms;
extern volatile int     lsm6dsr_fault;
#endif // USE_IMU_SENSOR

/* 堆内存监控 — cpu_monitor_task 写入, 不受 IMU 开关影响 */
extern volatile uint32_t debug_free_heap;


/* 如果有全局宏定义可以写在这里，比如 CPU 频率：
 * #define CPUCLK_FREQ 32000000
 */

#endif /* ALLDATA_H_ */