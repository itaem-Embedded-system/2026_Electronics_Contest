/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "alldata.h"
#include "stdio.h"
#include "string.h"

volatile uint32_t g_fault_code = 0;          // 0=正常, 1=malloc失败, 2=任务栈溢出
volatile char g_fault_task_name[16] = {0};
volatile uint32_t g_fault_free_heap = 0;
volatile uint32_t g_fault_min_ever_free_heap = 0;

static void Fault_RecordAndHalt(uint32_t code, const char *task_name)
{
    __disable_irq();
    Motor_SetPWM(1, 0);
    Motor_SetPWM(2, 0);

    g_fault_code = code;
    g_fault_free_heap = xPortGetFreeHeapSize();
    g_fault_min_ever_free_heap = xPortGetMinimumEverFreeHeapSize();

    for (uint8_t i = 0; i < sizeof(g_fault_task_name) - 1U; i++) {
        if (task_name != NULL && task_name[i] != '\0') {
            g_fault_task_name[i] = task_name[i];
        } else {
            g_fault_task_name[i] = '\0';
            break;
        }
    }
    g_fault_task_name[sizeof(g_fault_task_name) - 1U] = '\0';

    DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_0_PIN);
    DL_GPIO_clearPins(GPIO_LED_PORT,  GPIO_LED_Freertos_LED_PIN);

    while(1) {}
}

int main(void)
{
    // 1. 初始化 SysConfig 自动生成的底层硬件 
    SYSCFG_DL_init();

    // 2. 开启 UART1/UART2 的 NVIC 中断。SysConfig 只打开外设 RX 中断，这里让 CPU 真正响应中断。
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    // 3. 初始化编码器中断
    Encoder_Init();

    // 4. 初始化所有的 RTOS 任务
    RTOS_Tasks_Init();

    // 5. 启动 FreeRTOS 任务调度器！(从此 CPU 控制权交给 RTOS)
    vTaskStartScheduler();

    // 正常情况下，程序永远不会执行到这里
    while (1) {
    }
}




/* ================== 预留的 FreeRTOS 钩子函数 (防止编译报错) ================== */
void vApplicationMallocFailedHook(void)
{
    /* 1. 立即屏蔽所有中断, 冻结现场, 防止已损坏的栈被 ISR 进一步破坏 */
    __disable_irq();
    /* 2. 紧急停止双电机, 防止小车失控乱跑 (纯寄存器写入, 不需要中断) */
    Motor_SetPWM(1, 0);
    Motor_SetPWM(2, 0);
    /* 3. 蜂鸣器长鸣 + LED 点亮指示致命错误 */
    DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_0_PIN);
    DL_GPIO_clearPins(GPIO_LED_PORT,  GPIO_LED_Freertos_LED_PIN);
    /* 4. 短暂延时让错误指示可见, 然后触发芯片软复位恢复系统 */
    for (volatile uint32_t i = 0; i < 400000; i++) { __NOP(); }  /* ~5ms @ 80MHz */
    NVIC_SystemReset();
    /* 5. 如果复位因某种原因失败, 死循环兜底 */
    while(1) {}
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    /* 与 malloc 失败钩子相同的紧急处理流程 */
    __disable_irq();
    Motor_SetPWM(1, 0);
    Motor_SetPWM(2, 0);
    DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_0_PIN);
    DL_GPIO_clearPins(GPIO_LED_PORT,  GPIO_LED_Freertos_LED_PIN);
    for (volatile uint32_t i = 0; i < 8000000; i++) { __NOP(); }
    NVIC_SystemReset();
    while(1) {}
}


