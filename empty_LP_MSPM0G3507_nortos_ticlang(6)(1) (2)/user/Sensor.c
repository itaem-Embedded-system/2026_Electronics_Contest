#include "alldata.h"
#include "rtos_tasks.h"

#if USE_GRAY_SENSOR
// === 定义黑板变量 ===
volatile uint8_t g_gray_raw_data = 0;
volatile int32_t g_gray_error = 0;
volatile uint8_t g_gray_stop_line_latched = 0;

#define GRAY_STOP_LINE_HOLD_CYCLES  5

static int32_t Sensor_CalcErrorFromRaw(uint8_t status);
static uint8_t Sensor_IsStopLineRaw(uint8_t status);

/**
 * @brief  跨端口获取八路状态，并按 OUT1-OUT8 顺序重映射
 * @note   硬件映射关系:
 *         OUT1 (原12路第1路,  权重: -550) -> PA27
 *         OUT2 (原12路第3路,  权重: -350) -> PA26
 *         OUT3 (原12路第5路,  权重: -150) -> PA25
 *         OUT4 (原12路第6路,  权重: -50)  -> PA24
 *         OUT5 (原12路第7路,  权重: +50)  -> PB25
 *         OUT6 (原12路第8路,  权重: +150) -> PB24
 *         OUT7 (原12路第10路, 权重: +350) -> PB20
 *         OUT8 (原12路第12路, 权重: +550) -> PA22
 * @return 返回 8 位位图。Bit0~Bit7 分别对应 OUT1~OUT8。
 *         1 代表黑线，0 代表白底。
 */
uint8_t Sensor_GetRawData(void)
{
    uint8_t status = 0;
    
    // 1. 一次性读取两个端口的完整状态，减少寄存器访问耗时
    uint32_t reg_porta = GPIOA->DIN31_0;
    uint32_t reg_portb = GPIOB->DIN31_0;

    // 2. 精准映射：将分散的引脚按 OUT1 到 OUT8 的顺序塞入 Bit 0 到 Bit 7
    // 注意：通过判断对应引脚的电平状态，直接将对应的位置 1
    
    // --- 组装左侧与中心传感器 (PORTA) ---
    if (reg_porta & (1 << 27)) status |= (1 << 0); // OUT1 (左侧最外)
    if (reg_porta & (1 << 26)) status |= (1 << 1); // OUT2 
    if (reg_porta & (1 << 25)) status |= (1 << 2); // OUT3 
    if (reg_porta & (1 << 24)) status |= (1 << 3); // OUT4
    if (reg_porta & (1 << 22)) status |= (1 << 7); // OUT8 (右侧最外)
    
    // --- 组装右侧传感器 (PORTB) ---
    if (reg_portb & (1 << 25)) status |= (1 << 4); // OUT5 
    if (reg_portb & (1 << 24)) status |= (1 << 5); // OUT6 
    if (reg_portb & (1 << 20)) status |= (1 << 6); // OUT7

    return status;
}

/**
 * @brief  根据同一次采样数据计算加权偏差
 */
static int32_t Sensor_CalcErrorFromRaw(uint8_t status)
{
    int32_t sum = 0;
    int32_t count = 0;
    static int32_t last_error = 0;

    // 原 12 路去掉第 2/4/9/11 路后，保留传感器对应原始物理位置: 1,3,5,6,7,8,10,12。
    const int32_t weights[] = {-550, -350, -120, -50, 50, 120, 350, 550};

    for (int i = 0; i < 8; i++)
    {
        if (status & (1U << i))
        {
            sum += weights[i];
            count++;
        }
    }

    // 丢线时保持最后一次偏差，让循迹控制继续按原方向找线。
    if (count == 0) return last_error;

    last_error = sum / count;
    return last_error;
}

/**
 * @brief  加权偏差计算
 */
int32_t Sensor_GetError(void)
{
    return Sensor_CalcErrorFromRaw(Sensor_GetRawData());
}

static uint8_t Sensor_IsStopLineRaw(uint8_t status)
{
    uint8_t black_count = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (status & (1U << i)) black_count++;
    }

    return (black_count >= 4U) ||
           ((status & 0x07U) == 0x07U) ||
           ((status & 0xE0U) == 0xE0U);
}

/**
 * @brief  灰度传感器后台任务 (FreeRTOS)
 * @note   优先级建议设置为中低优先级 (osPriorityNormal)
 */
void Gray_Task(void *pvParameters)
{
    uint8_t stop_line_hold_count = 0;

    while(1)
    {
        // 1. 异步采集与计算 
        uint8_t raw = Sensor_GetRawData();
        int32_t err = Sensor_CalcErrorFromRaw(raw);

        if (Sensor_IsStopLineRaw(raw)) {
            stop_line_hold_count = GRAY_STOP_LINE_HOLD_CYCLES;
        } else if (stop_line_hold_count > 0U) {
            stop_line_hold_count--;
        }
        
        // 2. 其他任务随时可读
        g_gray_raw_data = raw;
        g_gray_error = err;
        g_gray_stop_line_latched = (stop_line_hold_count > 0U);
        
        // 3.10ms 的采集周期
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#endif // USE_GRAY_SENSOR