#ifndef __SENSOR_H
#define __SENSOR_H

#include <stdint.h>            
#include "ti_msp_dl_config.h"  

#if USE_GRAY_SENSOR
// === 变量声明 ===
extern volatile uint8_t g_gray_raw_data;
extern volatile int32_t g_gray_error;

// === 底层与任务声明 ===
uint8_t Sensor_GetRawData(void);
int32_t Sensor_GetError(void);        // 改为返回 int32_t
void Gray_Task(void *pvParameters);   // 巡线后台任务
#endif // USE_GRAY_SENSOR

#endif