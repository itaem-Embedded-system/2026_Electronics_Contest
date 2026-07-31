#ifndef __ZDT_X42S_H__
#define __ZDT_X42S_H__

#include <stdint.h>
#include <stdbool.h>

/* ================== 基础参数定义 ================== */
#define ZDT_DEFAULT_ADDR    0x01    // 电机默认出厂地址
#define ZDT_DEFAULT_CHECK   0x6B    // 默认校验码包尾
#define ZDT_MAX_FRAME_LEN   16      // 当前常用反馈帧最大长度

/* ================== 运动状态枚举 ================== */
// 旋转方向
typedef enum {
    ZDT_DIR_CW  = 0x00,  // 顺时针
    ZDT_DIR_CCW = 0x01   // 逆时针 
} ZDT_Dir_t;

// 位置运动模式
typedef enum {
    ZDT_MODE_REL_PREV = 0x00, // 相对上一目标位置运动 (多用于连续插补)
    ZDT_MODE_ABS      = 0x01, // 绝对位置运动 (打哪指哪)
    ZDT_MODE_REL_CUR  = 0x02  // 相对当前实际位置运动
} ZDT_MoveMode_t;

/* ================== 核心控制结构体 ================== */
/**
 * @brief 闭环步进电机状态对象
 * 用于保存主控从电机读取到的最新物理状态
 */
typedef struct {
    // 基础信息
    uint8_t  address;       // 该电机的通信地址 (默认0x01)
    bool     isEnable;      // 使能状态：true=处于使能(锁轴), false=脱机(松开)
    
    // 实时物理反馈
    int32_t  realPosition;  // 实时位置 (累计脉冲数或换算后的角度)
    int16_t  realSpeed;     // 实时转速 (RPM，有正负表示方向)
    uint16_t realCurrent;   // 实时相电流 (mA，用于判断负载大小)
    
    // 异常与报警
    bool     isStall;       // 堵转标志位：true=发生了堵转
    uint8_t  errorCode;     // 详细故障码 (过压/过流等)
    
    // 通信状态
    bool     rxReady;       // 接收队列中存在完整反馈帧
    uint32_t rxFrameCount;  // 已接收有效帧计数
    uint32_t rxDropCount;   // 接收队列满或异常丢帧计数
} ZDT_Motor_t;

/* ================== 全局变量声明 ================== */
#if USE_ZDT_STEPPER
// 声明全局电机对象，供其他文件直接调用读取状态
extern ZDT_Motor_t Motor1;


// ================== API 接口声明 ==================

// 初始化
void ZDT_Init(void);

// 底层通信
bool ZDT_UART_Send(const uint8_t *data, uint16_t len);

// 读取一帧反馈数据，返回 true 表示成功取到完整帧
bool ZDT_ReadFrame(uint8_t *frame, uint8_t *len);

// 基础控制
bool ZDT_Enable(uint8_t addr, bool enable, bool sync);

// 数据读取
bool ZDT_ReadPosition(uint8_t addr);

// 运动指令
bool ZDT_MoveAbsolute(uint8_t addr, ZDT_Dir_t dir, uint16_t rpm, uint8_t acc, uint32_t pulses);

// ================== 实战高频运动指令 ==================

// 1. 紧急停止
bool ZDT_Stop(uint8_t addr, bool sync);

// 2. 速度模式
bool ZDT_RunVelocity(uint8_t addr, ZDT_Dir_t dir, uint16_t rpm, uint8_t acc, bool sync);

// 3. 相对位置运动
bool ZDT_MoveRelative(uint8_t addr, ZDT_Dir_t dir, uint16_t rpm, uint8_t acc, uint32_t pulses);

// 4. 设置当前位置为机械零点 (配合限位开关做开机校准)
bool ZDT_SetCurrentPositionAsZero(uint8_t addr);
#endif // USE_ZDT_STEPPER

#endif /* __ZDT_X42S_H__ */
