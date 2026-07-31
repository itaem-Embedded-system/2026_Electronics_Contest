#include "ultrasonic.h"
#include "rtos_tasks.h"

#if USE_ULTRASONIC
/* Sonar_Task_Handle 由 rtos_tasks.c 在创建任务时填入 */
extern TaskHandle_t Sonar_Task_Handle;

/* ISR → 任务传递的 echo 脉宽计数值 (timer ticks)
 * 由 ISR 在下降沿写入, 任务在收到通知后读取 */
static volatile uint32_t echo_pulse_cnt = 0;

/* 全局共享的最新距离数据 (cm), -1 代表超时/超量程 */
volatile float global_distance_cm = -1.0f;

void Ultrasonic_Init(void)
{
    /* 确保 TRIG 初始为低电平 */
    DL_GPIO_clearPins(GPIO_Ultrasonic_PORT, GPIO_Ultrasonic_PIN_Trig_PIN);

    /* 停止并清零定时器 (确保干净的初始状态) */
    DL_TimerG_stopCounter(TIMER_Ultrasonic_INST);
    DL_TimerG_setTimerCount(TIMER_Ultrasonic_INST, 0);
}

/* ==========================================
 * 硬件中断回调函数 (GROUP1_IRQHandler → 此处)
 *
 * SysConfig 配置为 BOTH 边沿触发:
 *   上升沿 → 启动定时器
 *   下降沿 → 停止定时器 + 通知任务
 * ========================================== */
void Ultrasonic_ISR_Handler(void)
{
    if (DL_GPIO_readPins(GPIO_Ultrasonic_PORT, GPIO_Ultrasonic_PIN_Echo_PIN))
    {
        /* 上升沿: echo 脉冲开始, 启动定时器 */
        DL_TimerG_setTimerCount(TIMER_Ultrasonic_INST, 0);
        DL_TimerG_startCounter(TIMER_Ultrasonic_INST);
    }
    else
    {
        /* 下降沿: echo 脉冲结束, 停止定时器并读取脉宽 */
        DL_TimerG_stopCounter(TIMER_Ultrasonic_INST);
        echo_pulse_cnt = DL_TimerG_getTimerCount(TIMER_Ultrasonic_INST);

        /* 任务通知: 唤醒阻塞在 ulTaskNotifyTake 的 ultrasonic_task */
        if (Sonar_Task_Handle != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(Sonar_Task_Handle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/* ==========================================
 * 超声波任务 (ISR 驱动, 无 busy-wait)
 *
 * 流程: Trig 脉冲 → 阻塞等待 ISR 通知 → 计算距离
 * 在 echo 等待期间 CPU 完全释放给其他任务.
 * ========================================== */
void ultrasonic_task(void *pvParameters)
{
    Ultrasonic_Init();

    /* 初始化通知值 (先消费掉可能残留的通知) */
    ulTaskNotifyTake(pdTRUE, 0);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        /* 1. 发射 20µs TRIG 脉冲 (阻塞极短, 可接受) */
        DL_GPIO_setPins(GPIO_Ultrasonic_PORT, GPIO_Ultrasonic_PIN_Trig_PIN);
        delay_cycles(CPUCLK_FREQ / 50000);
        DL_GPIO_clearPins(GPIO_Ultrasonic_PORT, GPIO_Ultrasonic_PIN_Trig_PIN);

        /* 2. 阻塞等待 ISR 在下降沿发来的通知 (CPU 被释放!)
         *    50ms 超时 ≈ 8.5m 最大量程, 超时则标记 -1 */
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50)) != 0) {
            global_distance_cm = (float)echo_pulse_cnt * 0.0017f;
        } else {
            global_distance_cm = -1.0f;
        }

        /* 3. 100ms 测距周期 (vTaskDelayUntil 防累积漂移) */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

/* ==========================================
 * 数据获取接口 (其他任务调用时瞬间返回)
 * ========================================== */
float Ultrasonic_GetDistance(void)
{
    return global_distance_cm;
}

#endif // USE_ULTRASONIC