#ifndef __KEY_H__
#define __KEY_H__

// 引入标准整数类型 (为了识别 uint8_t)
#include <stdint.h>

// ================= 函数声明 =================

// 初始化按键 
void Key_Init(void);

// 获取按键键码 (主循环里调用，读取后会自动清零)
// 返回值: 0=无按键按下, 2=S2被按下
uint8_t Key_GetNum(void);

// 获取按键当前瞬时状态 (底层函数，一般给 Key_Tick 用)
uint8_t Key_GetState(void);

// 按键状态机扫描函数
void Key_Tick(void);

#endif /* __KEY_H__ */