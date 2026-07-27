#include "alldata.h"

uint8_t Key_Num = 0;

uint8_t Key_GetNum(void)
{
    uint8_t Temp;
    if (Key_Num)
    {
        Temp = Key_Num;
        Key_Num = 0;
        return Temp;
    }
    return 0;
}

uint8_t Key_GetState(void)
{
    
    if (DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_S4_PIN) == 0)
    {
        return 4; // S4 按下返回 4
    }
    return 0; 
}

void Key_Tick(void)
{
    static uint16_t lock_time = 0;
    if (lock_time > 0) {
        lock_time--; 
        return; 
    }

    uint8_t CurrState = Key_GetState();
    if (CurrState != 0) 
    {
        Key_Num = CurrState;
        lock_time = 30; // 30 * 10ms = 300ms 冷却时间[cite: 3]
    }
}