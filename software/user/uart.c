#include "alldata.h"

// ================= 视觉通信全局变量 =================
volatile uint32_t g_rx_pulse = 0;  // 脉搏计数器
int16_t g_target_x = 0, g_target_y = 0;
int16_t g_laser_x = 0, g_laser_y = 0;
uint8_t g_vision_ready_flag = 0; // 视觉帧更新标志

// ================= 字符串指令全局变量 =================
char Serial_RxPacket[100];      // 存储类似 "[SPEED100*]" 的字符串
uint8_t Serial_RxFlag = 0;      // 字符串接收完成标志


// ================= 发送与 printf 重定向 =================

void VOFA_SendString(char *str) {
    while (*str != '\0') {
        DL_UART_Main_transmitDataBlocking(UART_1_INST, *str);
        str++;
    }
}

// printf 重定向
int fputc(int c, FILE* stream) {
    DL_UART_Main_transmitDataBlocking(UART_1_INST, c);
    return c;
}

// ================= UART_1: PC 数据与指令接收 =================
void UART_1_INST_IRQHandler(void) 
{
    static uint8_t RxState = 0;
    static uint8_t pRxPacket = 0;
    uint8_t RxData;
    
    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            RxData = DL_UART_Main_receiveData(UART_1_INST);

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

// ================= UART_2: 视觉模块 (预留) =================
void UART_2_INST_IRQHandler(void) 
{
    static uint8_t rx_state = 0;
    static uint8_t obj_count = 0;
    static uint8_t rx_buffer[100];
    static uint16_t rx_index = 0;
    uint8_t rx_data;

    switch (DL_UART_Main_getPendingInterrupt(UART_2_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            rx_data = DL_UART_Main_receiveData(UART_2_INST);

            g_rx_pulse++; 

            if (rx_state == 0 && rx_data == 0xAA) { rx_state = 1; }
            else if (rx_state == 1 && rx_data == 0xBB) { rx_state = 2; }
            else if (rx_state == 2) { 
                obj_count = rx_data;
                if (obj_count > 0 && obj_count <= 5) { rx_index = 0; rx_state = 3; }
                else rx_state = 0;
            }
            else if (rx_state == 3) { 
                rx_buffer[rx_index++] = rx_data;
                if (rx_index >= obj_count * sizeof(VisionObj_t)) rx_state = 4;
            }
            else if (rx_state == 4 && rx_data == 0xCC) { rx_state = 5; }
            else if (rx_state == 5 && rx_data == 0xDD) {
                VisionObj_t *objs = (VisionObj_t *)rx_buffer;
                // 默认置为 -1，代表“本帧没看到”
                int16_t tx = -1, ty = -1, lx = -1, ly = -1;
                for (int i = 0; i < obj_count; i++) {
                    if (objs[i].cid == 0) { tx = objs[i].x; ty = objs[i].y; }
                    if (objs[i].cid == 1) { lx = objs[i].x; ly = objs[i].y; }
                }
                
                g_target_x = tx; g_target_y = ty;
                g_laser_x  = lx; g_laser_y  = ly;
                g_vision_ready_flag = 1; // 新一帧到了
                
                rx_state = 0; 
            }
            break;
        default: break;
    }
}