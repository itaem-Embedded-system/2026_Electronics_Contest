#include "alldata.h"
#include "rtos_tasks.h"

#if USE_IMU_SENSOR
/* Debug 观测变量 — 在 CCS Expressions 窗口中可直接观察 */
volatile int16_t debug_accel_raw[3];
volatile int16_t debug_gyro_raw[3];

/* LSM6DSR 核心寄存器地址 */
#define LSM6DSR_CTRL1_XL      0x10   /* 加速度计 ODR + 满量程 */
#define LSM6DSR_CTRL2_G       0x11   /* 陀螺仪 ODR + 满量程 */
#define LSM6DSR_CTRL3_C       0x12   /* BDU + 软件复位 */
#define LSM6DSR_INT1_CTRL     0x0D   /* INT1 引脚控制 */
#define LSM6DSR_INT2_CTRL     0x0E   /* INT2 引脚控制 */
#define LSM6DSR_CTRL5_C       0x14   /* 采样对齐 (ROUNDING) */
#define LSM6DSR_CTRL6_C       0x15   /* 加速度计高性能模式 + Gyro LPF1 带宽 */
#define LSM6DSR_CTRL7_G       0x16   /* 陀螺仪高性能模式 + HPF 配置 */
#define LSM6DSR_OUTX_L_G      0x22
#define LSM6DSR_OUTX_L_A      0x28
#define LSM6DSR_WHO_AM_I      0x0F

/* ================================================================
 *  内部 SPI 读写接口 (不对外暴露)
 * ================================================================ */

static void LSM6DSR_WriteReg(uint8_t reg, uint8_t data) {
    DL_GPIO_clearPins(GPIO_IMU_PORT, GPIO_IMU_CS_PIN);

    DL_SPI_transmitData8(SPI_0_INST, reg & 0x7F);      /* 写操作: 最高位=0 */
    while (DL_SPI_isBusy(SPI_0_INST));
    DL_SPI_receiveData8(SPI_0_INST);                   /* 清空 TX 产生的无用返回 */

    DL_SPI_transmitData8(SPI_0_INST, data);
    while (DL_SPI_isBusy(SPI_0_INST));
    DL_SPI_receiveData8(SPI_0_INST);

    DL_GPIO_setPins(GPIO_IMU_PORT, GPIO_IMU_CS_PIN);
}

static uint8_t LSM6DSR_ReadReg(uint8_t reg) {
    uint8_t val;
    DL_GPIO_clearPins(GPIO_IMU_PORT, GPIO_IMU_CS_PIN);

    DL_SPI_transmitData8(SPI_0_INST, reg | 0x80);      /* 读操作: 最高位=1 */
    while (DL_SPI_isBusy(SPI_0_INST));
    DL_SPI_receiveData8(SPI_0_INST);                   /* 地址阶段的无用返回 */

    DL_SPI_transmitData8(SPI_0_INST, 0x00);            /* 发送 dummy 时钟读出数据 */
    while (DL_SPI_isBusy(SPI_0_INST));
    val = DL_SPI_receiveData8(SPI_0_INST);

    DL_GPIO_setPins(GPIO_IMU_PORT, GPIO_IMU_CS_PIN);
    return val;
}

/* ================================================================
 *  传感器初始化
 * ================================================================ */

void LSM6DSR_Init(void) {
    /* 1. 软件复位 */
    LSM6DSR_WriteReg(LSM6DSR_CTRL3_C, 0x01);
    vTaskDelay(pdMS_TO_TICKS(15));   /* 等待复位完成, 使用 RTOS 延时交出 CPU */

    /* 2. 配置加速度计 (104Hz ODR, +/-2g) */
    LSM6DSR_WriteReg(LSM6DSR_CTRL1_XL, LSM6DSR_ODR_XL_REG_VAL);

    /* 3. 配置陀螺仪 (104Hz ODR, +/-250dps) */
    LSM6DSR_WriteReg(LSM6DSR_CTRL2_G, LSM6DSR_ODR_G_REG_VAL);

    /* 4. 使能 BDU (Block Data Update) — 防止高低字节撕裂 */
    LSM6DSR_WriteReg(LSM6DSR_CTRL3_C, 0x44);

    /* 5. INT2 引脚控制: Gyro + Accel Data Ready 信号
     *    关键: 只有 SPI Burst Read 整套连贯流程 (一次性 CS 读完 12 字节)
     *    执行完毕后, 传感器的 INT2 硬件引脚才会被真正清零拉低!
     *    如果分 12 次单字节读, INT2 行为会错乱. */
    LSM6DSR_WriteReg(LSM6DSR_INT1_CTRL, 0x00);   /* 禁用 INT1 */
    LSM6DSR_WriteReg(LSM6DSR_INT2_CTRL, 0x03);   /* INT2: DRDY_G | DRDY_XL */

    /* 6. 显式精度配置 (基于 AN5358 + DS11976 Rev 2) */

    /* CTRL5_C (0x14): ROUNDING=1, 对齐加速度计和陀螺仪的采样时刻 */
    LSM6DSR_WriteReg(LSM6DSR_CTRL5_C, 0x04);

    /* CTRL6_C (0x15): XL_HM_MODE=0(加速度计高性能), FTYPE=011(Gyro LPF1 ~8.3Hz@104HzODR)
     *   电赛小车物理带宽 <5Hz, 33Hz LPF 过宽会引入电机高频振动和齿轮噪声.
     *   降至 8.3Hz 预期噪声降低 √(33/8.3) ≈ 2 倍. */
    LSM6DSR_WriteReg(LSM6DSR_CTRL6_C, 0x03);

    /* CTRL7_G (0x16): G_HM_MODE=0(陀螺仪高性能), HP_EN=0(禁用 HPF), HPM=00 */
    LSM6DSR_WriteReg(LSM6DSR_CTRL7_G, 0x00);
}

/* ================================================================
 *  SPI Burst Read — 一次 CS 读完 6 轴 12 字节
 *
 *  对比旧方案 (12 次独立 CS 片选):
 *    旧: for (i=0; i<12; i++) buffer[i] = LSM6DSR_ReadReg(0x22 + i);
 *        → 每次 CS↓ → 发地址 → 收 1B → CS↑, 12 次 = 12×SPI 事务开销
 *    新: CS↓ → 发起始地址 → 连续收 12B → CS↑, 1 次 SPI 事务
 *
 *  优势:
 *    - 消除 11 次多余的 CS 翻转 + 地址发送, 节省 ~60% SPI 耗时
 *    - 12 字节在同一采样周期内读出, 无数据撕裂
 *    - INT2 引脚行为正确 (连贯读完才会清零)
 *
 *  依赖: LSM6DSR 上电默认启用 IF_INC (地址自动递增)
 * ================================================================ */

void LSM6DSR_Read_RawData(int16_t *accel, int16_t *gyro) {
    uint8_t buffer[12];

    /* --- 拉低 CS, 开始一次完整的 SPI Burst 事务 --- */
    DL_GPIO_clearPins(GPIO_IMU_PORT, GPIO_IMU_CS_PIN);

    /* 发送起始寄存器地址 (OUTX_L_G = 0x22, | 0x80 = 读模式) */
    DL_SPI_transmitData8(SPI_0_INST, LSM6DSR_OUTX_L_G | 0x80);
    while (DL_SPI_isBusy(SPI_0_INST));
    DL_SPI_receiveData8(SPI_0_INST);   /* 地址阶段的无用返回, 清空 */

    /* 连续发 12 个 dummy 字节, 靠地址自动递增一口气读完:
     *   [0-1]  OUTX_L_G, OUTX_H_G   → Gyro X
     *   [2-3]  OUTY_L_G, OUTY_H_G   → Gyro Y
     *   [4-5]  OUTZ_L_G, OUTZ_H_G   → Gyro Z
     *   [6-7]  OUTX_L_A, OUTX_H_A   → Accel X
     *   [8-9]  OUTY_L_A, OUTY_H_A   → Accel Y
     *   [10-11] OUTZ_L_A, OUTZ_H_A  → Accel Z  */
    for (int i = 0; i < 12; i++) {
        DL_SPI_transmitData8(SPI_0_INST, 0x00);
        while (DL_SPI_isBusy(SPI_0_INST));
        buffer[i] = DL_SPI_receiveData8(SPI_0_INST);
    }

    /* --- 读完 12 字节, 拉高 CS, 结束事务 --- */
    DL_GPIO_setPins(GPIO_IMU_PORT, GPIO_IMU_CS_PIN);

    /* 数据拼接: LSB 在前, MSB 在后 */
    gyro[0]  = (int16_t)((buffer[1]  << 8) | buffer[0]);   /* Gx */
    gyro[1]  = (int16_t)((buffer[3]  << 8) | buffer[2]);   /* Gy */
    gyro[2]  = (int16_t)((buffer[5]  << 8) | buffer[4]);   /* Gz */

    accel[0] = (int16_t)((buffer[7]  << 8) | buffer[6]);   /* Ax */
    accel[1] = (int16_t)((buffer[9]  << 8) | buffer[8]);   /* Ay */
    accel[2] = (int16_t)((buffer[11] << 8) | buffer[10]);  /* Az */

    /* 更新 Debug 观测变量 (CCS Expressions 窗口可见) */
    debug_gyro_raw[0]  = gyro[0];
    debug_gyro_raw[1]  = gyro[1];
    debug_gyro_raw[2]  = gyro[2];
    debug_accel_raw[0] = accel[0];
    debug_accel_raw[1] = accel[1];
    debug_accel_raw[2] = accel[2];
}

/* ================================================================
 *  便捷接口: 原始值 → 物理单位
 * ================================================================ */

void LSM6DSR_Read_Scaled(float *accel_g, float *gyro_dps) {
    int16_t accel_raw[3], gyro_raw[3];
    LSM6DSR_Read_RawData(accel_raw, gyro_raw);

    for (int i = 0; i < 3; i++) {
        gyro_dps[i] = LSM6DSR_GYRO_RAW_TO_DPS(gyro_raw[i]);
        accel_g[i]  = LSM6DSR_ACCEL_RAW_TO_G(accel_raw[i]);
    }
}

/* ================================================================
 *  设备检测
 * ================================================================ */

uint8_t LSM6DSR_Check_WhoAmI(void) {
    return LSM6DSR_ReadReg(LSM6DSR_WHO_AM_I);
}

#endif // USE_IMU_SENSOR
