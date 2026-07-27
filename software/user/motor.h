#ifndef MOTOR_H_
#define MOTOR_H_

#include "ti_msp_dl_config.h"

/* PWM 最大比较值 — 适配 10kHz 降频计划
 * L: TIMG6 @80MHz, timerCount = 7999 -> 10kHz, max compare = 7998
 * R: TIMG8 @40MHz, timerCount = 3999 -> 10kHz, max compare = 3998
 * 减 1 的原因是: 比较值 == 周期值时 PWM 输出归零 (硬件 bug)
 */
#define PWM_L_MAX_CMP    7998
#define PWM_R_MAX_CMP    3998

// 驱动衰减模式枚举
typedef enum {
    MOTOR_DRIVE_COAST_DECAY = 0, // 快衰减 (滑行): 当前默认模式，低电平续流
    MOTOR_DRIVE_SLOW_DECAY  = 1  // 慢衰减 (刹车): 100%高电平驱动，高电平刹车
} Motor_DriveMode_t;

// 设置衰减模式
void Motor_SetDriveMode(Motor_DriveMode_t mode);

// 设置电机 PWM (-100 到 100)
void Motor_SetPWM(uint8_t n, int32_t PWM);

#endif /* MOTOR_H_ */