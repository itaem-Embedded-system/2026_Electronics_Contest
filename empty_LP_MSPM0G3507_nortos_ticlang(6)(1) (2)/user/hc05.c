#include "alldata.h"
#include "rtos_tasks.h"

#if USE_BLUETOOTH
/* =========================================================================
 * 1. 全局变量与数据结构定义
 * ========================================================================= */
typedef enum {
    STATE_WAIT_HEADER1 = 0,
    STATE_WAIT_HEADER2,
    STATE_WAIT_CMD,
    STATE_WAIT_DATA1,
    STATE_WAIT_DATA2,
    STATE_WAIT_CHECKSUM,
    STATE_WAIT_TAIL
} RxState_e;

// FreeRTOS 蓝牙消息队列句柄 (连接中断与任务的桥梁)
QueueHandle_t BT_RxQueue = NULL;

// 全局控制指令变量 (供任务读取)
BT_Ctrl_Cmd_t g_bt_cmd = {0, 0, 0};

// 状态机内部静态变量 (用于记录解析过程中的中间状态，不对外暴露)

static RxState_e current_state = STATE_WAIT_HEADER1;
static uint8_t rx_cmd_type = 0;
static int8_t  rx_data1 = 0;   
static int8_t  rx_data2 = 0;   
static uint8_t rx_checksum = 0;



/* =========================================================================
 * 2. 硬件初始化函数
 * ========================================================================= */
void HC05_Init(void) {
    BT_RxQueue = xQueueCreate(64, sizeof(uint8_t));
    
    if (BT_RxQueue != NULL) {
        NVIC_EnableIRQ(UART_3_INST_INT_IRQN); 
    }
}

/* =========================================================================
 * 3. 协议解析状态机 
 * ========================================================================= */
static void HC05_Parse_Byte(uint8_t rx_data) {
    switch(current_state) {
        case STATE_WAIT_HEADER1:
            if(rx_data == 0xAA) {
                current_state = STATE_WAIT_HEADER2;
                rx_checksum = 0; // 收到包头，立刻重置校验和计算器
            }
            break;
            
        case STATE_WAIT_HEADER2:
            if(rx_data == 0x55) {
                current_state = STATE_WAIT_CMD;
            } else {
                current_state = STATE_WAIT_HEADER1; // 误判，退回初始状态
            }
            break;
            
        case STATE_WAIT_CMD:
            rx_cmd_type = rx_data;
            rx_checksum += rx_data; // 累加校验和
            current_state = STATE_WAIT_DATA1;
            break;
            
        case STATE_WAIT_DATA1:
            rx_data1 = (int8_t)rx_data; // <--- 存入通用变量 1
            rx_checksum += rx_data;
            current_state = STATE_WAIT_DATA2;
            break;
            
        case STATE_WAIT_DATA2:
            rx_data2 = (int8_t)rx_data; // <--- 存入通用变量 2
            rx_checksum += rx_data;
            current_state = STATE_WAIT_CHECKSUM;
            break;
            
        case STATE_WAIT_CHECKSUM:
            // 校验和比对 (手机发来的 vs 本地累加计算的)
            if(rx_data == rx_checksum) {
                current_state = STATE_WAIT_TAIL;
            } else {
                // 校验失败，说明数据受干扰了，抛弃整包数据
                current_state = STATE_WAIT_HEADER1;
            }
            break;
            
        case STATE_WAIT_TAIL:
            if(rx_data == 0xFF) {
                // 成功接收一包完整且合法的数据
                g_bt_cmd.cmd_type = rx_cmd_type;
                g_bt_cmd.D1       = rx_data1;    // <--- 传递通用数据 1
                g_bt_cmd.D2       = rx_data2;    // <--- 传递通用数据 2
            }
            // 一包流程结束，清零状态，准备吃下一包
            current_state = STATE_WAIT_HEADER1;
            break;
            
        default:
            current_state = STATE_WAIT_HEADER1;
            break;
    }
}

/* =========================================================================
 * 4. FreeRTOS 蓝牙处理任务
 * ========================================================================= */
void Bluetooth_Task(void *pvParameters) {
    uint8_t rx_byte;
    
    // 初始化外设与队列
    HC05_Init();

    while(1) {
        // xQueueReceive 会让任务进入死等状态 (portMAX_DELAY)，完全不占 CPU
        // 一旦中断向队列里塞入数据，任务立刻苏醒并拿走数据
        if (xQueueReceive(BT_RxQueue, &rx_byte, portMAX_DELAY) == pdPASS) {
            
            // 将收到的 1 个字节送入状态机
            HC05_Parse_Byte(rx_byte);
            
            
        }
    }
}

/* =========================================================================
 * 5. 极简中断服务函数 
 * ========================================================================= */
void UART_3_INST_IRQHandler(void) {

    switch(DL_UART_Main_getPendingInterrupt(UART_3_INST)) {
        case DL_UART_MAIN_IIDX_RX:
        {
            // 从底层寄存器读取收到的 1 个字节 
            uint8_t received_data = DL_UART_Main_receiveData(UART_3_INST);
            
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            
            // 将数据塞进消息队列
            xQueueSendFromISR(BT_RxQueue, &received_data, &xHigherPriorityTaskWoken);
            
            // 触发上下文切换
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;
        }
        default:
            break;
    }
}
#else
/**
 * @brief BT 禁用时的哑变量: 保证 control_test_task 和 OLED 不出现链接错误
 *        cmd_type 恒为 0 → 控制任务不会处理任何指令, 无副作用
 */
BT_Ctrl_Cmd_t g_bt_cmd = {0, 0, 0};

/**
 * @brief 哑函数 ISR：防止孤儿中断触发 Default Handler 死机
 */
void UART_3_INST_IRQHandler(void) {
    DL_UART_Main_clearInterruptStatus(UART_3_INST, DL_UART_Main_getPendingInterrupt(UART_3_INST));
    DL_UART_Main_receiveData(UART_3_INST); // 读空 RX 寄存器
}
#endif // USE_BLUETOOTH