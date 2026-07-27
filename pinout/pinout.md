# MCU 及模块引脚分配

本表根据 `software/CLAUDE.md` 中的 SysConfig 外设映射整理，用于维护 MSPM0G3507 与各传感器、执行器、通信模块的统一引脚分配。

## SysConfig 外设映射

| 功能 | 外设 / 配置 | 信号 | MCU 引脚 | 说明 |
|------|-------------|------|----------|------|
| IMU SPI | SPI1, MOTO3, 4MHz | SCK | PB9 | LSM6DSR SPI 时钟 |
| IMU SPI | SPI1, MOTO3, 4MHz | MOSI | PB8 | LSM6DSR SPI 主出从入 |
| IMU SPI | SPI1, MOTO3, 4MHz | MISO | PB7 | LSM6DSR SPI 主入从出 |
| IMU SPI | SPI1, MOTO3, 4MHz | CS | PB6 | LSM6DSR 片选 |
| IMU INT1 | GPIO 中断, 上升沿, pri 3 | INT1 | PB14 | Data Ready 中断 |
| 左编码器 AB | GPIO 中断, 上升沿 | A | PB13 | 左轮编码器 A 相 |
| 左编码器 AB | GPIO 输入 | B | PB12 | 左轮编码器 B 相, 用于判向 |
| 右编码器 AB | GPIO 中断, 上升沿 | A | PA9 | 右轮编码器 A 相 |
| 右编码器 AB | GPIO 输入 | B | PA8 | 右轮编码器 B 相, 用于判向 |
| 左电机 PWM | TIMG6, CCP0/1 | PWM0 | PB26 | 左电机 PWM 输出 |
| 左电机 PWM | TIMG6, CCP0/1 | PWM1 | PB27 | 左电机 PWM 输出 |
| 右电机 PWM | TIMG8, CCP0/1 | PWM0 | PB10 | 右电机 PWM 输出 |
| 右电机 PWM | TIMG8, CCP0/1 | PWM1 | PB11 | 右电机 PWM 输出 |
| 超声波 Trig | GPIO 输出 | TRIG | PB3 | HC-SR04 触发信号 |
| 超声波 Echo | GPIO 中断, 上升沿 | ECHO | PB23 | HC-SR04 回波信号 |
| 超声波 Timer | TIMA0, ONE_SHOT_UP, 25ms | - | - | 测量 Echo 脉宽, 无外部引脚 |
| 七路灰度 | GPIO 输入, 7 通道 | OUT1 | PA27 | 灰度传感器通道 1 |
| 七路灰度 | GPIO 输入, 7 通道 | OUT2 | PA26 | 灰度传感器通道 2 |
| 七路灰度 | GPIO 输入, 7 通道 | OUT3 | PA25 | 灰度传感器通道 3 |
| 七路灰度 | GPIO 输入, 7 通道 | OUT4 | PA24 | 灰度传感器通道 4 |
| 七路灰度 | GPIO 输入, 7 通道 | OUT5 | PB25 | 灰度传感器通道 5 |
| 七路灰度 | GPIO 输入, 7 通道 | OUT6 | PB24 | 灰度传感器通道 6 |
| 七路灰度 | GPIO 输入, 7 通道 | OUT7 | PB20 | 灰度传感器通道 7 |
| OLED I2C | GPIO OD 开漏, 软件模拟 | SDA | PA0 | OLED 软件 I2C 数据线 |
| OLED I2C | GPIO OD 开漏, 软件模拟 | SCL | PA1 | OLED 软件 I2C 时钟线 |
| HC-05 控制 | GPIO | EN | PA15 | 蓝牙模块 EN |
| HC-05 状态 | GPIO | STATE | PA16 | 蓝牙模块连接状态 |
| HC-05 UART | UART2, 115200, RX 中断 | TX | PB15 | MCU UART2 TX |
| HC-05 UART | UART2, 115200, RX 中断 | RX | PB16 | MCU UART2 RX |
| VOFA UART | UART1, 115200, RX 中断 | TX | PB4 | MCU UART1 TX |
| VOFA UART | UART1, 115200, RX 中断 | RX | PB5 | MCU UART1 RX |
| 视觉 UART | UART3, RX 中断 | TX | PA14 | MCU UART3 TX |
| 视觉 UART | UART3, RX 中断 | RX | PA13 | MCU UART3 RX |
| ZDT UART | UART0, 115200, RX 中断 | TX | PA10 | MCU UART0 TX |
| ZDT UART | UART0, 115200, RX 中断 | RX | PB1 | MCU UART0 RX |
| 按键 S4 | GPIO 输入, 上拉 | S4 | PB21 | 功能按键 |
| LED | GPIO 输出 | LED | PB22 | 运行状态指示 |
| 蜂鸣器 | GPIO 输出 | BUZZER | PB0 | 蜂鸣器控制 |
| 运行统计 Timer | TIMG12, PERIODIC_UP, /8 | - | - | FreeRTOS 运行统计计时, 无外部引脚 |

## 按 MCU 引脚排序

| MCU 引脚 | 功能 | 信号 |
|----------|------|------|
| PA0 | OLED I2C | SDA |
| PA1 | OLED I2C | SCL |
| PA8 | 右编码器 AB | B |
| PA9 | 右编码器 AB | A |
| PA10 | ZDT UART | TX |
| PA13 | 视觉 UART | RX |
| PA14 | 视觉 UART | TX |
| PA15 | HC-05 控制 | EN |
| PA16 | HC-05 状态 | STATE |
| PA24 | 七路灰度 | OUT4 |
| PA25 | 七路灰度 | OUT3 |
| PA26 | 七路灰度 | OUT2 |
| PA27 | 七路灰度 | OUT1 |
| PB0 | 蜂鸣器 | BUZZER |
| PB1 | ZDT UART | RX |
| PB3 | 超声波 Trig | TRIG |
| PB4 | VOFA UART | TX |
| PB5 | VOFA UART | RX |
| PB6 | IMU SPI | CS |
| PB7 | IMU SPI | MISO |
| PB8 | IMU SPI | MOSI |
| PB9 | IMU SPI | SCK |
| PB10 | 右电机 PWM | PWM0 |
| PB11 | 右电机 PWM | PWM1 |
| PB12 | 左编码器 AB | B |
| PB13 | 左编码器 AB | A |
| PB14 | IMU INT1 | INT1 |
| PB15 | HC-05 UART | TX |
| PB16 | HC-05 UART | RX |
| PB20 | 七路灰度 | OUT7 |
| PB21 | 按键 S4 | S4 |
| PB22 | LED | LED |
| PB23 | 超声波 Echo | ECHO |
| PB24 | 七路灰度 | OUT6 |
| PB25 | 七路灰度 | OUT5 |
| PB26 | 左电机 PWM | PWM0 |
| PB27 | 左电机 PWM | PWM1 |
