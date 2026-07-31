#ifndef __UART_H
#define __UART_H

#include "ti_msp_dl_config.h" 
#include <stdint.h>           
#include <stdio.h>            

// ================= UART1 视觉通信数据 =================
// 视觉模块接到 UART1，按行发送目标相对画面中心的 x 偏差，单位为像素。
// 例如：12\r\n、-46\r\n、0\r\n。
extern volatile uint32_t g_rx_pulse;
extern volatile int16_t g_vision_x_offset;
extern volatile uint8_t g_vision_ready_flag;

// 兼容旧代码：g_target_x 保存当前 x_offset，y 和激光坐标不再由该协议提供。
extern int16_t g_target_x, g_target_y;
extern int16_t g_laser_x, g_laser_y;

/* ================= 字符串指令全局变量 ================= */
extern char Serial_RxPacket[100]; 
extern uint8_t Serial_RxFlag;     

/* ================= 外部调用函数声明 ================= */

/**
 * @brief  VOFA+ 专用字符串发送函数，现在通过 UART2 输出。
 */
void VOFA_SendString(char *str);

#endif // __UART_H