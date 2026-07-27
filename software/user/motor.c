#include "alldata.h"

// 默认使用原来的快衰减模式，确保改完代码后行为一致
static Motor_DriveMode_t current_drive_mode = MOTOR_DRIVE_COAST_DECAY;

void Motor_SetDriveMode(Motor_DriveMode_t mode)
{
    current_drive_mode = mode;
}

void Motor_SetPWM(uint8_t n, int32_t PWM)
{   
    uint32_t compareValue = 0;
    uint32_t max_cmp = 0;
    uint32_t val_IN1 = 0;
    uint32_t val_IN2 = 0;
    
    int32_t safe_PWM = PWM;

    // 1. 安全限幅保护
    if (safe_PWM > 100)  safe_PWM = 100;
    if (safe_PWM < -100) safe_PWM = -100;

    // 计算绝对值
    int32_t abs_PWM = (safe_PWM >= 0) ? safe_PWM : -safe_PWM;
    
    // 2. 获取对应电机的 MAX_CMP
    if (n == 1) {
        max_cmp = PWM_L_MAX_CMP;
    } else if (n == 2) {
        max_cmp = PWM_R_MAX_CMP;
    } else {
        return; // 防错
    }

    // 计算目标 PWM 对应的比较值
    compareValue = (max_cmp * abs_PWM) / 100;

    // 3. 根据衰减模式分配两路 IN 的比较值
    if (current_drive_mode == MOTOR_DRIVE_COAST_DECAY) {
        // 【快衰减】: 驱动相输出 PWM，非驱动相恒为 0 (低电平)
        if (safe_PWM >= 0) {
            val_IN1 = compareValue;
            val_IN2 = 0;
        } else {
            val_IN1 = 0;
            val_IN2 = compareValue;
        }
    } else {
        // 【慢衰减】: 驱动相恒为 100% (max_cmp)，非驱动相输出 100% - PWM
        // 这样在 PWM off 期间，两路 IN 均为高电平，触发 DRV8871 刹车环路
        if (safe_PWM >= 0) {
            val_IN1 = max_cmp;
            val_IN2 = max_cmp - compareValue;
        } else {
            val_IN1 = max_cmp - compareValue;
            val_IN2 = max_cmp;
        }
    }

    // 4. 硬件控制执行
    if (n == 1)
    {
        // ================= 左电机 (n=1) =================
        // PWM_motor_L (TIMG6): C1 对应 L-IN1, C0 对应 L-IN2
        DL_TimerG_setCaptureCompareValue(PWM_motor_L_INST, val_IN1, GPIO_PWM_motor_L_C1_IDX);
        DL_TimerG_setCaptureCompareValue(PWM_motor_L_INST, val_IN2, GPIO_PWM_motor_L_C0_IDX);
    }
    else if (n == 2)
    {
        // ================= 右电机 (n=2) =================
        // PWM_motor_R (TIMG8): C0 对应 R-IN1, C1 对应 R-IN2
        DL_TimerG_setCaptureCompareValue(PWM_motor_R_INST, val_IN2, GPIO_PWM_motor_R_C0_IDX);
        DL_TimerG_setCaptureCompareValue(PWM_motor_R_INST, val_IN1, GPIO_PWM_motor_R_C1_IDX);
    }
}