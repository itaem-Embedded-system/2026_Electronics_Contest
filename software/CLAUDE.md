# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

电子设计大赛小车项目 — 基于 MSPM0G3507 的差速驱动小车，运行 FreeRTOS 多任务系统。集成了 IMU 姿态解算、双电机 PID 速度/转向闭环控制、多传感器采集（超声波、七路灰度巡线、编码器）、OLED 显示、蓝牙遥控和闭环步进电机驱动。

仓库中另有 `Test/` 子项目，是纯粹的 IMU 测试平台（裸机、无 RTOS），用于 LSM6DSR 陀螺仪数据采集和 Mahony 算法调校。两个项目独立构建，共享部分硬件驱动概念。

## Build System

- **IDE**: TI Code Composer Studio Theia (CCS 20.x, Theia-based)
- **Compiler**: TI Arm Clang 5.1.1.LTS (TICLANG), `thumbv6m`, `cortex-m0plus`
- **SDK**: MSPM0-SDK v2.10.00.04 at `C:/ti/mspm0_sdk_2_10_00_04/`
- **SysConfig**: v1.27.0+, `empty.syscfg` 定义所有外设/引脚配置，自动生成 `Debug/ti_msp_dl_config.c/h`
- **RTOS**: FreeRTOS Kernel v11.2.0, 源码包含在 `Freertos/` 目录
- **Debug Probe**: XDS110 via SWD (PA19/SWDIO, PA20/SWCLK)
- **Output**: `Debug/empty_LP_MSPM0G3507_nortos_ticlang.out`

编译和烧录**必须通过 CCS Theia GUI 操作**，没有独立的命令行构建脚本。`empty.syscfg` 修改后需在 CCS 中重新生成 `ti_msp_dl_config.c/h`。

### Build Constraints

- TI Clang 不支持 ARM Compiler 6 inline assembly，移植代码时注意兼容
- Cortex-M0+ 不支持硬件浮点，所有浮点运算为软件模拟（慎用 `sqrtf`、`sinf` 等）
- malloc/newlib 不可用，必须用 FreeRTOS heap (`pvPortMalloc`) 或静态分配
- `configTOTAL_HEAP_SIZE` = 25KB (共 32KB SRAM)

## Architecture

### 启动流程

```
main()  [empty.c]
  ├── SYSCFG_DL_init()           — SysConfig 自动生成的硬件初始化 (时钟/GPIO/UART/SPI/PWM/Timer)
  ├── Encoder_Init()             — 编码器 GPIO 中断注册
  ├── RTOS_Tasks_Init()          — 创建 11 个 FreeRTOS 任务 (见下文)
  └── vTaskStartScheduler()      — 启动调度器, 永不返回
```

### FreeRTOS 任务总览 (按创建顺序)

| 任务名 | 优先级 | 堆栈 | 功能 |
|--------|-------|------|------|
| **IMU_Task** | 3 | 1024w | IMU 初始化→零偏校准→Mahony 姿态解算, INT1 中断触发运行 |
| **Ctrl_Task** | 2 | 2048w | 50ms 周期: 读蓝牙指令→编码器轮速→并联 PID→PWM 输出 |
| **CPU_Monitor** | 1 | 256w | 每秒统计各任务 CPU 占用率和堆栈水位 |
| **KeyScan** | 2 | 128w | 10ms 按键扫描 + S4 功能键状态机 |
| **Sonar_Task** | 2 | 512w | 超声波 Trig/Echo 时序, 距离计算, `global_distance_cm` |
| **GrayTask** | 2 | 256w | 七路灰度传感器读取 + 偏差计算 |
| **OLED_Task** | 1 | 512w | OLED 显示刷新, 200ms 周期 |
| **BT_Task** | 3 | 256w | 蓝牙 HC-05 命令帧解析, 填充 `g_bt_cmd` |
| **Heartbeat** | 1 | 128w | LED 500ms 翻转, 系统存活指示 |
| **Buzzer** | 2 | 256w | 蜂鸣器调度 (短鸣/长鸣), 标志位 `g_bt_beep_cmd` 驱动 |
| **ZDT_Test** | 1 | 256w | 步进电机演示动作序列 (速度→停止→归零→相对位移) |

### 电机控制架构 (Ctrl_Task 核心)

```
target_speed (±100)  ─→ [速度PID: Kp=1.6 Ki=0.25]  ─→ ave_pwm ─┐
target_turn  (±100)  ─→ [转向PID: Kp=1.2 Ki=0.55]  ─→ dif_pwm ─┤
                                                                  ├─→ PWM_L = ave + dif/2
轮速反馈: Encoder_Get(1/2) 归一化到 ±100 (每50ms MAX_PULSE_50MS=185)  │   PWM_R = ave - dif/2
                                                                  ↓
                                                            Motor_SetPWM(1/2, pwm)
```

- 速度环+转向环**并联**运行 (各自独立 PID 计算后合成)
- 电子手刹: 目标=0 且实际接近零时清零积分项, 防止死区震颤
- PWM 输出范围限制在 ±100, 物理电平由 SysConfig TIMG6/TIMG8 PWM 生成

### 中断服务 (GROUP1_IRQHandler)

单一中断组 (`rtos_tasks.c:GROUP1_IRQHandler`) 同时处理:
1. **左编码器** A 相 (PB13, LA): 根据 B 相 (PB12, LB) 电平判向, `±encoder_L`
2. **右编码器** A 相 (PA9, RA): 根据 B 相 (PA8, RB) 电平判向, `±encoder_R`
3. **IMU INT1** (PB14): 发出 `vTaskNotifyGiveFromISR` 唤醒 IMU_Task
4. **超声波 Echo** (PB23): 上升沿→启动 Timer, 下降沿→停止并计算距离

### SysConfig 外设映射

| 功能 | 外设 | 引脚 |
|------|------|------|
| SPI0 (IMU) | SPI1, MOTO3, 4MHz | SCK=PB9, MOSI=PB8, MISO=PB7, CS=PB6 |
| IMU INT1 | GPIO 中断, 上升沿, pri 3 | PB14 |
| 左编码器 AB | GPIO 中断, 上升沿 | A=PB13, B=PB12 |
| 右编码器 AB | GPIO 中断, 上升沿 | A=PA9, B=PA8 |
| 左电机 PWM | TIMG6, CCP0/1 | PB26, PB27 |
| 右电机 PWM | TIMG8, CCP0/1 | PB10, PB11 |
| 超声波 Trig | GPIO 输出 | PB3 |
| 超声波 Echo | GPIO 中断, 上升沿 | PB23 |
| 超声波 Timer | TIMA0, ONE_SHOT_UP, 25ms | — |
| 七路灰度 | GPIO 输入, 7 通道 | PA27/26/25/24, PB25/24/20 |
| OLED I2C | GPIO OD 开漏, 软件模拟 | SDA=PA0, SCL=PA1 |
| HC-05 EN/STATE | GPIO | EN=PA15, STATE=PA16 |
| HC-05 UART | UART2, 115200, RX 中断 | TX=PB15, RX=PB16 |
| VOFA UART | UART1, 115200, RX 中断 | TX=PB4, RX=PB5 |
| 视觉 UART | UART3, RX 中断 | TX=PA14, RX=PA13 |
| ZDT UART | UART0, 115200, RX 中断 | TX=PA10, RX=PB1 |
| 按键 S4 | GPIO 输入, 上拉 | PB21 |
| LED | GPIO 输出 | PB22 |
| 蜂鸣器 | GPIO 输出 | PB0 |
| 运行统计 Timer | TIMG12, PERIODIC_UP, ÷8 | — |

### 用户模块头文件体系

所有模块通过 `alldata.h` 汇聚引用，工程中任何 `.c` 文件只需 `#include "alldata.h"` 即可获取:
- 所有标准库 (`stdint`, `stdbool`, `math`, `stdio` 等)
- `ti_msp_dl_config.h` (SysConfig 自动生成)
- FreeRTOS (`FreeRTOS.h`, `task.h`, `queue.h`)
- 所有用户驱动模块头文件
- 全局共享变量 extern 声明 (`roll`, `pitch`, `yaw`)

### Test/ 子项目 (bare-metal IMU testbed)

位置: `D:\MSPM0Project\Test\`

裸机 Mahony 滤波器测试平台, 专门用于:
- LSM6DSR 数据采集和 SPI 驱动验证
- Mahony 参数调校 (Kp, Ki)
- 陀螺校准算法验证
- VOFA+ JustFloat 实时数据流

与主项目的区别: 无 RTOS, 无电机控制, 无传感器融合。是主项目中 IMU 子系统的"先行验证平台"。共用相同的 Mahony/LSM6DSR 驱动核心逻辑但有独立副本。

## Shared Data Flow

关键数据通过 `volatile extern` 全局变量在任务间共享 (`alldata.h` 声明, `rtos_tasks.c` 定义):

```
IMU_Task          → roll/pitch/yaw (±180°)     — 供 PID 或上层策略使用
Encoder ISR       → encoder_L/R (累积脉冲)      — Ctrl_Task 读取并换算轮速
Ctrl_Task         → target_pwm_L/R (±100)       — motor.c 转换为 PWM 占空比
HC-05 UART ISR    → g_bt_cmd (cmd_type, D1, D2)— Ctrl_Task 提取目标速度/转向
Ultrasonic ISR    → global_distance_cm          — 超声波测距结果
Gray_Task         → g_gray_raw_data / g_gray_error — 巡线偏差
Buzzer_Task       ← g_bt_beep_cmd (0/1/2)       — 蓝牙指令→提示音
```

## Hardware Constraints

- **MSPM0G3507**: Cortex-M0+, 80MHz, 128KB Flash, 32KB SRAM
- **LSM6DSR**: SPI 通信, CS=PB6, INT1=PB14 (Data Ready 信号, 1.04kHz 采样)
- **ZDT-X42S**: 闭环步进电机, UART0 驱动, 自定义二进制协议 (`ZDT_X42S.c/h`)
- **HC-05**: 蓝牙串口, 自定义帧格式 (0xAA 0x55 ... 0xFF)
- **七路灰度**: 权重映射 OUT1=-300 到 OUT7=+300, 黑线检测位图
- **超声波**: HC-SR04 兼容, Trig+Echo 时序, Timer 捕获脉宽

## PC 端分析工具

项目根目录 `D:\MSPM0Project\` 下的 Python 脚本 (针对 Test/ 子项目的 VOFA 数据流):

- **`data_logger.py`**: COM9 JustFloat 数据记录→CSV
- **`analyze_imu.py`**: 静止数据统计分析, 对标 LSM6DSR 数据手册
- **`run_logger.bat`**: Windows 一键启动
- CSV 格式: `timestamp_ms, frame, gyro_x/y/z_dps, accel_x/y/z_g, roll/pitch/yaw_deg`

## 已知 Bug 与隐患 (Known Issues)

### 🟡 仍存在: 时钟常量硬编码 vs SysConfig 分离

`IMU_DT_FREQ_HZ (39063)` 和 `IMU_DT_WRAP (39063)` 在 `rtos_tasks.c` 中硬编码, 但定时器实际频率由 SysConfig `TIMER3` 配置 (prescale=256, div=8) 决定。如果有人在 SysConfig 中修改了定时器配置, 这些常量不会自动更新, 导致 dt 计算错误。

同理 `configCPU_CLOCK_HZ (80000000)` 在 `FreeRTOSConfig.h` 中硬编码, 必须手动与 SysConfig 时钟树保持一致。

**计算验证** (当前配置):
- BUSCLK = 80 MHz → timer_clk = 80M / (256 × 8) = 39062.5 Hz
- IMU_DT_FREQ_HZ 近似为 39063 (误差 ~12.8 ppm, 可忽略)
- FreeRTOS tick = 80M / 1000 = 80000 → 1ms per tick ✅

### 🟢 Info: IMU_dt 回绕处理(rtos_tasks.c:326)

```c
if (delta < 0) delta += IMU_DT_WRAP;   /* 1 秒回绕补偿 */
```
仅补偿一次回绕。由于采样率 104Hz, 单次中断处理不可能超过 1 秒, 所以实际上不会有问题。

## 主持修改所需信息

如果需要我主持这个项目的修改, 除上述已分析的内容外, 还需要分析需要什么信息才能修改

