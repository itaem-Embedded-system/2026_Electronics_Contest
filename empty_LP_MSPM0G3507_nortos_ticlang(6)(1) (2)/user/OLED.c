/***************************************************************************************
  * MSPM0G3507 移植精简版 OLED 驱动程序
  * 移除了冗余的 2D 绘图与中文字库依赖，保留最核心的 Printf 极速输出能力
  *
  * WARNING: 本驱动使用软件 GPIO 模拟 I2C (非硬件 I2C 外设), 不可重入.
  *          当前仅 Task_OLED_Display (pri 1) 单一任务访问, 无并发保护.
  *          如需多任务访问, 必须添加互斥量保护所有 OLED_* 函数入口.
  ***************************************************************************************/

#include "alldata.h"
#include "rtos_tasks.h"

#if USE_OLED_DISPLAY

/* 全局变量 *********************/
uint8_t OLED_DisplayBuf[8][128];


/* =========================================================================
 * 1. 硬件引脚抽象层
 * 负责单片机与 OLED 模块之间最基础的电平交互
 * ========================================================================= */

/**
  * @brief  OLED写SCL高低电平
  * @param  BitValue: 期望输出的电平状态。传入 1 输出高电平，传入 0 输出低电平。
  */
void OLED_W_SCL(uint8_t BitValue)
{
    if (BitValue) {
        DL_GPIO_setPins(GPIO_OLED_PORT, GPIO_OLED_SCL_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_OLED_PORT, GPIO_OLED_SCL_PIN);
    }
    Delay_us(1); 
}

/**
  * @brief  OLED写SDA高低电平
  * @param  BitValue: 期望输出的电平状态。传入 1 输出高电平，传入 0 输出低电平。
  */
void OLED_W_SDA(uint8_t BitValue)
{
    if (BitValue) {
        DL_GPIO_setPins(GPIO_OLED_PORT, GPIO_OLED_SDA_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_OLED_PORT, GPIO_OLED_SDA_PIN);
    }
    Delay_us(1); 
}

/**
  * @brief  OLED软件I2C引脚初始化
  * @note   底层时钟和开漏配置已在 SysConfig 的 SYSCFG_DL_init() 中完成，
  *         此处仅作总线空闲释放并提供供电缓冲。
  */
void OLED_GPIO_Init(void)
{
    OLED_W_SCL(1);
    OLED_W_SDA(1);
    
    Delay_ms(1); 
}


/* =========================================================================
 * 2. 软件 I2C 通信层
 * 通过按顺序翻转引脚，模拟出标准的 I2C 通信波形
 * ========================================================================= */

/**
  * @brief  产生 I2C 通信的起始信号
  */
void OLED_I2C_Start(void)
{
    OLED_W_SDA(1);
    OLED_W_SCL(1);
    OLED_W_SDA(0);
    OLED_W_SCL(0);
}

/**
  * @brief  产生 I2C 通信的停止信号
  */
void OLED_I2C_Stop(void)
{
    OLED_W_SDA(0);
    OLED_W_SCL(1);
    OLED_W_SDA(1);
}

/**
  * @brief  向总线上发送连续的 8 位数据 (1 个字节)
  * @param  Byte: 准备发送的 8 位无符号整数数据
  */
void OLED_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        OLED_W_SDA(!!(Byte & (0x80 >> i)));
        OLED_W_SCL(1);
        OLED_W_SCL(0);
    }
    OLED_W_SCL(1);
    OLED_W_SCL(0);
}

/**
  * @brief  向 OLED 屏幕发送“控制指令”
  * @param  Command: 具体的 OLED 硬件寄存器指令代码 (如 0xAE 关显示)
  */
void OLED_WriteCommand(uint8_t Command)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);
    OLED_I2C_SendByte(0x00); // 0x00 代表接下来发送的是命令
    OLED_I2C_SendByte(Command);
    OLED_I2C_Stop();
}

/**
  * @brief  向 OLED 屏幕的显存中连续发送“图像数据”
  * @param  Data: 指向要发送的数据数组的指针
  * @param  Count: 本次连续发送的数据字节数量
  */
void OLED_WriteData(uint8_t *Data, uint8_t Count)
{
    uint8_t i;
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);
    OLED_I2C_SendByte(0x40); // 0x40 代表接下来发送的是数据
    for (i = 0; i < Count; i ++)
    {
        OLED_I2C_SendByte(Data[i]);
    }
    OLED_I2C_Stop();
}


/* =========================================================================
 * 3. OLED 硬件配置层
 * 负责驱动 OLED 屏幕的核心控制芯片
 * ========================================================================= */

/**
  * @brief  OLED 屏幕的上电初始化总函数 (配置寄存器序列并点亮屏幕)
  */
void OLED_Init(void)
{
    OLED_GPIO_Init();
    
    OLED_WriteCommand(0xAE);	//关闭显示
    OLED_WriteCommand(0xD5);	//设置显示时钟分频比/振荡器频率
    OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8);	//设置多路复用率
    OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xD3);	//设置显示偏移
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40);	//设置显示开始行
    OLED_WriteCommand(0xA1);	//设置左右方向，0xA1正常，0xA0左右反置
    OLED_WriteCommand(0xC8);	//设置上下方向，0xC8正常，0xC0上下反置
    OLED_WriteCommand(0xDA);	//设置COM引脚硬件配置
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81);	//设置对比度
    OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9);	//设置预充电周期
    OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB);	//设置VCOMH取消选择级别
    OLED_WriteCommand(0x30);
    OLED_WriteCommand(0xA4);	//设置整个显示打开/关闭
    OLED_WriteCommand(0xA6);	//设置正常/反色显示，0xA6正常，0xA7反色
    OLED_WriteCommand(0x8D);	//设置充电泵
    OLED_WriteCommand(0x14);
    OLED_WriteCommand(0xAF);	//开启显示
    
    OLED_Clear();
    OLED_Update();
}

/**
  * @brief  设置 OLED 硬件写入指针的物理坐标位置
  * @param  Page: 页地址 (Y轴)，范围 0~7，代表屏幕的 8 个水平条带
  * @param  X: 列地址 (X轴)，范围 0~127，代表横向的像素坐标
  */
void OLED_SetCursor(uint8_t Page, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Page);
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));
    OLED_WriteCommand(0x00 | (X & 0x0F));
}


/* =========================================================================
 * 4. 显存管理与刷新层 
 * 所有的显示操作都在单片机内存中进行，统一推送到屏幕
 * ========================================================================= */

/**
  * @brief  将单片机内存中的缓冲区彻底推送到 OLED 硬件屏幕上
  * @note   每次调用下面的显示函数后，必须调用此函数，屏幕才会真正刷新！
  */
void OLED_Update(void)
{
    uint8_t j;
    for (j = 0; j < 8; j ++)
    {
        OLED_SetCursor(j, 0);
        OLED_WriteData(OLED_DisplayBuf[j], 128);
    }
}

/**
  * @brief  将内存缓冲区全部填 0 (全局清屏)
  */
void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j ++)
    {
        for (i = 0; i < 128; i ++)
        {
            OLED_DisplayBuf[j][i] = 0x00;
        }
    }
}

/**
  * @brief  将内存缓冲区中指定的矩形区域清零 (局部擦除)
  * @param  X: 擦除区域左上角的 X 坐标
  * @param  Y: 擦除区域左上角的 Y 坐标
  * @param  Width: 需要擦除的区域宽度
  * @param  Height: 需要擦除的区域高度
  */
void OLED_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    int16_t i, j;
    for (j = Y; j < Y + Height; j ++)
    {
        for (i = X; i < X + Width; i ++)
        {
            if (i >= 0 && i <= 127 && j >=0 && j <= 63)
            {
                OLED_DisplayBuf[j / 8][i] &= ~(0x01 << (j % 8));
            }
        }
    }
}


/* =========================================================================
 * 5. 用户 API 接口层 (User API Layer)
 * 用于写业务逻辑时直接调用的高层函数
 * ========================================================================= */

/**
  * @brief  在缓冲区的指定位置绘制一幅二维点阵图像 (内部供贴字模使用)
  * @param  X: 图像左上角的 X 坐标
  * @param  Y: 图像左上角的 Y 坐标
  * @param  Width: 图像的像素宽度
  * @param  Height: 图像的像素高度
  * @param  Image: 指向图像点阵字模数组的指针
  */
void OLED_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image)
{
    uint8_t i = 0, j = 0;
    int16_t Page, Shift;
    
    OLED_ClearArea(X, Y, Width, Height);
    
    for (j = 0; j < (Height - 1) / 8 + 1; j ++)
    {
        for (i = 0; i < Width; i ++)
        {
            if (X + i >= 0 && X + i <= 127)
            {
                Page = Y / 8;
                Shift = Y % 8;
                if (Y < 0) { Page -= 1; Shift += 8; }
                
                if (Page + j >= 0 && Page + j <= 7)
                {
                    OLED_DisplayBuf[Page + j][X + i] |= Image[j * Width + i] << (Shift);
                }
                if (Page + j + 1 >= 0 && Page + j + 1 <= 7)
                {					
                    OLED_DisplayBuf[Page + j + 1][X + i] |= Image[j * Width + i] >> (8 - Shift);
                }
            }
        }
    }
}

/**
  * @brief  在指定位置显示单个 ASCII 字符
  * @param  X: 字符左上角的 X 坐标
  * @param  Y: 字符左上角的 Y 坐标
  * @param  Char: 要显示的英文字母或符号 (如 'A')
  * @param  FontSize: 字体大小 (如 OLED_8X16 或 OLED_6X8)
  */
void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize)
{
    if (FontSize == OLED_8X16)
    {
        OLED_ShowImage(X, Y, 8, 16, OLED_F8x16[Char - ' ']);
    }
    else if(FontSize == OLED_6X8)
    {
        OLED_ShowImage(X, Y, 6, 8, OLED_F6x8[Char - ' ']);
    }
}

/**
  * @brief  在指定位置显示一串纯英文/符号字符串
  * @param  X: 首字母左上角的 X 坐标
  * @param  Y: 首字母左上角的 Y 坐标
  * @param  String: 要显示的字符串 (如 "Hello")
  * @param  FontSize: 字体大小 (如 OLED_8X16 或 OLED_6X8)
  */
void OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize)
{
    uint16_t i = 0;
    while (String[i] != '\0')
    {
        OLED_ShowChar(X + i * FontSize, Y, String[i], FontSize);
        i++;
    }
}

/**
  * @brief  【极速排版神技】类似于 C 语言标准的 printf，将变量混合格式化输出到屏幕
  * @param  X: 文本起始 X 坐标
  * @param  Y: 文本起始 Y 坐标
  * @param  FontSize: 字体大小 (如 OLED_8X16 或 OLED_6X8)
  * @param  format: 带有 %d, %f 等占位符的格式化字符串模板 (如 "Speed:%d")
  * @param  ... : 对应占位符的变量列表
  */
void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, const char *format, ...)
{
    char String[64];
    va_list arg;
    va_start(arg, format);
    vsnprintf(String, sizeof(String), format, arg);
    va_end(arg);
    OLED_ShowString(X, Y, String, FontSize);
}

#endif // USE_OLED_DISPLAY