#ifndef RTOS_TASKS_H_
#define RTOS_TASKS_H_

/* =========================================================================
 * 硬件外设与任务功能模块开关 (1: 启用, 0: 禁用)
 * ========================================================================= */
#define USE_IMU_SENSOR       1   // IMU 姿态传感器
#define USE_ULTRASONIC       0   // 超声波测距
#define USE_GRAY_SENSOR      1   // 灰度传感器
#define USE_OLED_DISPLAY     1   // OLED 显示屏
#define USE_ZDT_STEPPER      1   // 张大头步进电机
#define USE_BLUETOOTH        1   // 蓝牙模块
#define USE_VOFA_DEBUG       1  // VOFA 调试输出

/* 供 main 函数调用的唯一接口 */
void RTOS_Tasks_Init(void);

#endif /* RTOS_TASKS_H_ */