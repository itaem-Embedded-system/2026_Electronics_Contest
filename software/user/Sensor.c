#include "alldata.h"
#include "rtos_tasks.h"

#if USE_GRAY_SENSOR
// === 定义黑板变量 ===
volatile uint8_t g_gray_raw_data = 0;
volatile int32_t g_gray_error = 0;

/**
 * @brief  跨端口获取七路状态，并按 OUT1-OUT7 顺序重映射
 * @note   硬件映射关系:
 *         OUT1 (权重: -300) -> PA27
 *         OUT2 (权重: -200) -> PA26
 *         OUT3 (权重: -100) -> PA25
 *         OUT4 (中心,  权重: 0) -> PA24
 *         OUT5 (权重: +100) -> PB25
 *         OUT6 (权重: +200) -> PB24
 *         OUT7 (权重: +300) -> PB20
 * @return 返回 8 位位图。Bit0~Bit6 分别对应 OUT1~OUT7。
 *         1 代表黑线，0 代表白底。
 */
uint8_t Sensor_GetRawData(void)
{
    uint8_t status = 0;
    
    // 1. 一次性读取两个端口的完整状态，减少寄存器访问耗时
    uint32_t reg_porta = GPIOA->DIN31_0;
    uint32_t reg_portb = GPIOB->DIN31_0;

    // 2. 精准映射：将分散的引脚按 OUT1 到 OUT7 的顺序塞入 Bit 0 到 Bit 6
    // 注意：通过判断对应引脚的电平状态，直接将对应的位置 1
    
    // --- 组装左侧与中心传感器 (PORTA) ---
    if (reg_porta & (1 << 27)) status |= (1 << 0); // OUT1 (左侧最外)
    if (reg_porta & (1 << 26)) status |= (1 << 1); // OUT2 
    if (reg_porta & (1 << 25)) status |= (1 << 2); // OUT3 
    if (reg_porta & (1 << 24)) status |= (1 << 3); // OUT4 (中心)
    
    // --- 组装右侧传感器 (PORTB) ---
    if (reg_portb & (1 << 25)) status |= (1 << 4); // OUT5 
    if (reg_portb & (1 << 24)) status |= (1 << 5); // OUT6 
    if (reg_portb & (1 << 20)) status |= (1 << 6); // OUT7 (右侧最外)

    return status;
}

/**
 * @brief  加权偏差计算
 */
int32_t Sensor_GetError(void)
{
    uint8_t status = Sensor_GetRawData(); 
    int32_t sum = 0;
    int32_t count = 0;
    static int32_t last_error = 0; 

    // 【核心改造】：权重放大 100 倍，全程使用 int32_t

    const int32_t weights[] = {-300, -200, -100, 0, 100, 200, 300};

    for (int i = 0; i < 7; i++)
    {
        if (status & (1 << i)) 
        {
            sum += weights[i];
            count++;
        }
    }

    // 1. 丢线处理：保持 last_error 让 PID 继续输出大转弯力
    if (count == 0) return last_error;

    // 2. 正常计算中心位置偏差
    last_error = sum / count; 
    return last_error;
}

/**
 * @brief  灰度传感器后台任务 (FreeRTOS)
 * @note   优先级建议设置为中低优先级 (osPriorityNormal)
 */
void Gray_Task(void *pvParameters)
{
    while(1)
    {
        // 1. 异步采集与计算 
        uint8_t raw = Sensor_GetRawData();
        int32_t err = Sensor_GetError();
        
        // 2. 其他任务随时可读
        g_gray_raw_data = raw;
        g_gray_error = err;
        
        // 3.10ms 的采集周期
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#endif // USE_GRAY_SENSOR