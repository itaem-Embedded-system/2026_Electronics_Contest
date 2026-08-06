#include "alldata.h"


volatile int32_t encoder_L = 0;
volatile int32_t encoder_R = 0;

void Encoder_Init(void)
{
    // 开启 GPIOA 中断 (对应右轮A相 RA: PA9)
    NVIC_EnableIRQ(ENCODER_A_GPIOA_INT_IRQN); 
    
    // 开启 GPIOB 多路中断 (对应左编码器 A 相 LA: PB13, 超声 Echo: PB23, IMU INT1: PB0 三路共享 GROUP1 GPIOB)
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN); 

    // 使能系统全局中断 (注意：如果在 main 函数里已经调过，这里可以省略，但保留也没事)
    __enable_irq();
}


// 获取并清零速度计数值（供 PID 定时器调用）
int16_t Encoder_Get(uint8_t n)
{
    int32_t Temp = 0;

    // 1. 保存当前中断状态, 关闭全局中断, 进入临界区
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (n == 1) // 获取左轮
    {
        Temp = -encoder_L; // 如果左轮装反了，这里加负号可以修正
        encoder_L = 0;
    }
    else if (n == 2) // 获取右轮
    {
        Temp = encoder_R;
        encoder_R = 0;
    }

    // 2. 仅在调用前中断开着的情况下恢复 (避免从已关中断上下文调用时误开)
    if (!primask) __enable_irq();

    return (int16_t)Temp;
}