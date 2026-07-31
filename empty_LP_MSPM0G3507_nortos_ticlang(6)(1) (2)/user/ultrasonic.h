#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include "alldata.h"

#if USE_ULTRASONIC
// 超声波底层硬件初始化
void Ultrasonic_Init(void);

// 供硬件中断服务函数调用的回调
void Ultrasonic_ISR_Handler(void);

// RTOS 独立测距任务
void ultrasonic_task(void *pvParameters);

// 供其他任务随时获取最新距离的非阻塞接口 (返回 cm，-1代表超时/超出量程)
float Ultrasonic_GetDistance(void);
#endif // USE_ULTRASONIC

#endif /* ULTRASONIC_H_ */