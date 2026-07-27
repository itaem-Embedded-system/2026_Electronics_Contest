#ifndef __UART_H
#define __UART_H

#include "ti_msp_dl_config.h" 
#include <stdint.h>           
#include <stdio.h>            

// ================= 视觉通信相关数据结构 =================
// 补充视觉对象结构体（防止编译报错）
typedef struct {
    uint16_t cid;
    int16_t x;
    int16_t y;
} VisionObj_t;

extern volatile uint32_t g_rx_pulse;
extern int16_t g_target_x, g_target_y;
extern int16_t g_laser_x, g_laser_y;
extern uint8_t g_vision_ready_flag;

/* ================= 字符串指令全局变量 ================= */
extern char Serial_RxPacket[100]; 
extern uint8_t Serial_RxFlag;     

/* ================= 外部调用函数声明 ================= */

/**
 * @brief  VOFA+ 专用字符串发送函数
 */
void VOFA_SendString(char *str);

#endif // __UART_H