#ifndef __ENCODER_H
#define __ENCODER_H

#include "ti_msp_dl_config.h" 


extern volatile int32_t encoder_L;
extern volatile int32_t encoder_R;

void Encoder_Init(void);
int16_t Encoder_Get(uint8_t n);

#endif