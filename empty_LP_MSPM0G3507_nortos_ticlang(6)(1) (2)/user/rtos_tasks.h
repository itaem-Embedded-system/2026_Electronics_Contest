#ifndef RTOS_TASKS_H_
#define RTOS_TASKS_H_

#include <stdint.h>

/* =========================================================================
 * 硬件外设与任务功能模块开关 (1: 启用, 0: 禁用)
 * ========================================================================= */
#define USE_IMU_SENSOR       0   // IMU 姿态传感器
#define USE_ULTRASONIC       0   // 超声波测距
#define USE_GRAY_SENSOR      1   // 灰度传感器
#define USE_OLED_DISPLAY     1   // OLED 显示屏
#define USE_ZDT_STEPPER      1   // 张大头步进电机
#define USE_BLUETOOTH        0   // 蓝牙模块
#define USE_VOFA_DEBUG       1  // VOFA 调试输出

typedef struct {
    float straight_speed;
    float turn_speed;
    int32_t turn_in_error;
    int32_t turn_out_error;
    float turn_deadband;
    float kp;
    uint8_t stop_line_black_min;
    uint8_t stop_line_confirm;
    uint8_t stop_line_ignore_cycles;
} LineTrace_Config_t;

typedef struct {
    int32_t min_pulse;
    int32_t max_pulse;
    int32_t max_step_per_update;
    float cmd_to_pulse;
    uint16_t rpm;
    uint8_t acc;
} RodActuator_Config_t;

/* 供 main 函数调用的接口 */
void RTOS_Tasks_Init(void);
void Chassis_MoveRelativeCm(float distance_cm);
void LineTrace_Start(const LineTrace_Config_t *config);
void LineTrace_Stop(void);
uint8_t LineTrace_IsRunning(void);
void RodActuator_Init(const RodActuator_Config_t *config);
void RodActuator_SetCenter(void);
bool RodActuator_SetTargetPulse(int32_t target_pulse);
bool RodActuator_SetTargetCmd(float cmd);
bool RodActuator_ReturnCenter(void);
int32_t RodActuator_GetTargetPulse(void);
uint8_t RodActuator_IsReady(void);

#endif /* RTOS_TASKS_H_ */