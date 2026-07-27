#ifndef __PID_CTRL_H__
#define __PID_CTRL_H__

#include <stdint.h>

// PID 结构体定义
typedef struct {
    float Target;  float Actual;  float Out;
    float Error0;  float Error1;  float ErrorInt;
    float Kp;      float Ki;      float Kd;
    float ErrorIntMax; float ErrorIntMin;
    float OutMax;      float OutMin;      float OutOffset;
} PID_t;

void PID_Init(PID_t *p);
void PID_Update(PID_t *p);

#endif // __PID_CTRL_H__