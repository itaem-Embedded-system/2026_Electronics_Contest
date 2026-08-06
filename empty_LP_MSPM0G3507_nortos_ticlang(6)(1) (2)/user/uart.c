#include "alldata.h"

// ================= 视觉通信全局变量 =================
volatile uint32_t g_rx_pulse = 0;  // 接收字节计数器
volatile int16_t g_vision_x_offset = 0;
volatile uint8_t g_vision_ready_flag = 0; // 收到一行有效视觉数据

// 兼容旧变量：当前协议只提供 x_offset，不提供 y 或激光坐标。
int16_t g_target_x = 0, g_target_y = -1;
int16_t g_laser_x = -1, g_laser_y = -1;

// ================= 字符串指令全局变量 =================
char Serial_RxPacket[100];      // 存储类似 "[SPEED100*]" 的字符串
uint8_t Serial_RxFlag = 0;      // 字符串接收完成标志


// ================= 发送与 printf 重定向 =================

void VOFA_SendString(char *str) {
    while (*str != '\0') {
        DL_UART_Main_transmitDataBlocking(UART_2_INST, *str);
        str++;
    }
}

// printf 重定向
int fputc(int c, FILE* stream) {
    DL_UART_Main_transmitDataBlocking(UART_2_INST, c);
    return c;
}

// ================= UART_1: 视觉模块按行接收 =================
// 协议：ASCII 有符号十进制整数，以 '\n' 结束（'\r' 被忽略），表示 x_offset（像素）。
void UART_1_INST_IRQHandler(void)
{
    enum { VISION_LINE_MAX = 15 };
    static char line_buffer[VISION_LINE_MAX + 1U];
    static uint8_t line_index = 0;
    static uint8_t line_has_digit = 0;
    uint8_t rx_data;

    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            rx_data = DL_UART_Main_receiveData(UART_1_INST);
            g_rx_pulse++;

            if (rx_data == '\r') {
                break;
            }

            if (rx_data == '\n') {
                if (line_has_digit != 0U) {
                    int32_t value = 0;
                    int8_t line_sign = 1;
                    uint8_t i = 0;

                    if (line_buffer[0] == '-') {
                        line_sign = -1;
                        i = 1;
                    } else if (line_buffer[0] == '+') {
                        i = 1;
                    }

                    for (; i < line_index; i++) {
                        value = value * 10 + (int32_t)(line_buffer[i] - '0');
                    }
                    value *= line_sign;

                    if (value > INT16_MAX) {
                        value = INT16_MAX;
                    } else if (value < INT16_MIN) {
                        value = INT16_MIN;
                    }

                    g_vision_x_offset = (int16_t)value;
                    g_target_x = (int16_t)value;
                    g_vision_ready_flag = 1U;
                }

                line_index = 0;
                line_has_digit = 0U;
                break;
            }

            if (line_index == 0U && (rx_data == '-' || rx_data == '+')) {
                line_buffer[line_index++] = (char)rx_data;
                break;
            }

            if (rx_data >= '0' && rx_data <= '9') {
                if (line_index < VISION_LINE_MAX) {
                    line_buffer[line_index++] = (char)rx_data;
                    line_has_digit = 1U;
                } else {
                    line_index = 0;
                    line_has_digit = 0U;
                }
            } else {
                line_index = 0;
                line_has_digit = 0U;
            }
            break;

        default:
            break;
    }
}

// ================= UART_2: PC 数据与指令接收 =================
void UART_2_INST_IRQHandler(void) 
{
    static uint8_t RxState = 0;
    static uint8_t pRxPacket = 0;
    uint8_t RxData;
    
    switch (DL_UART_Main_getPendingInterrupt(UART_2_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            RxData = DL_UART_Main_receiveData(UART_2_INST);

            // 状态机解析 [指令*] 格式的数据
            if (RxState == 0) {
                if (RxData == '[' && Serial_RxFlag == 0) {
                    RxState = 1; pRxPacket = 0; 
                }
            }
            else if (RxState == 1) {
                if (RxData == '*') RxState = 2; 
                else if (pRxPacket < 99) Serial_RxPacket[pRxPacket++] = RxData;
                else RxState = 0;
            }
            else if (RxState == 2) {
                if (RxData == ']') {
                    RxState = 0; 
                    Serial_RxPacket[pRxPacket] = '\0';  
                    Serial_RxFlag = 1; 
                }
            }
            break;
        default: break;
    }
}