#ifndef LSM6DSR_H_
#define LSM6DSR_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* SPI CS 引脚映射 — 由 SysConfig 在 ti_msp_dl_config.h 中定义:
 *   GPIO_IMU_PORT  = GPIOB
 *   GPIO_IMU_CS_PIN = DL_GPIO_PIN_6 (PB6)                          */

/* Debug 观测变量 — 在 CCS Expressions 窗口中可直接观察原始数据 */
#if USE_IMU_SENSOR
extern volatile int16_t debug_accel_raw[3];
extern volatile int16_t debug_gyro_raw[3];
#endif

/* ====== 灵敏度换算系数 (LSM6DSR 数据手册 Table 2, DS11976 Rev 2) ====== */
#define LSM6DSR_GYRO_SENSITIVITY_MDPS   8.75f    // mdps/LSB @ +/-250dps
#define LSM6DSR_ACCEL_SENSITIVITY_MG    0.061f   // mg/LSB  @ +/-2g

/* 原始 int16_t -> 物理单位 */
#define LSM6DSR_GYRO_RAW_TO_DPS(raw)    ((float)(raw) * LSM6DSR_GYRO_SENSITIVITY_MDPS / 1000.0f)
#define LSM6DSR_GYRO_RAW_TO_RADS(raw)   ((float)(raw) * LSM6DSR_GYRO_SENSITIVITY_MDPS / 1000.0f * 3.14159265f / 180.0f)
#define LSM6DSR_ACCEL_RAW_TO_G(raw)     ((float)(raw) * LSM6DSR_ACCEL_SENSITIVITY_MG  / 1000.0f)

/* ====== 微秒级精确延时 — 自动适配 CPUCLK_FREQ ====== */
#define DELAY_US(us)  delay_cycles((uint32_t)((uint64_t)(us) * CPUCLK_FREQ / 1000000ULL))

/* ====== 采样率配置 (ODR) — 修改 ODR 时同步改这三处 ====== */
#define LSM6DSR_ODR_HZ              104.0f
#define LSM6DSR_ODR_XL_REG_VAL      0x40   /* CTRL1_XL[7:4]=0100 → 104Hz, FS_XL=00 → ±2g */
#define LSM6DSR_ODR_G_REG_VAL       0x40   /* CTRL2_G[7:4]=0100  → 104Hz, FS_G=00  → ±250dps */

#if USE_IMU_SENSOR
// 初始化 LSM6DSR
void LSM6DSR_Init(void);

// 读取 6 轴原始数据 (需传入长度为 3 的 int16_t 数组)
void LSM6DSR_Read_RawData(int16_t *accel, int16_t *gyro);

// 读取 6 轴并换算为物理单位 (accel: g, gyro: dps)
void LSM6DSR_Read_Scaled(float *accel_g, float *gyro_dps);

// 验证设备 ID (正常返回 0x6B)
uint8_t LSM6DSR_Check_WhoAmI(void);
#endif // USE_IMU_SENSOR

#endif /* LSM6DSR_H_ */
