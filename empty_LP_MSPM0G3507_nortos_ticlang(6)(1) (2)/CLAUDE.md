# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

电子设计大赛小车项目 — 基于 MSPM0G3507 的差速驱动小车，运行 FreeRTOS 多任务系统。当前代码核心是“题目菜单 + 底盘闭环 + 视觉/摆杆执行器”三条主线：

- **底盘执行链路**：编码器采样 → 50ms 控制周期 → 速度环/转向环并联 PID → 左右轮 PWM。
- **题目菜单链路**：按键 S2/S3/S4 在 OLED 上选择题目 2~6，不同题目触发循迹、位置环循迹纠偏或题目 3 视觉闭环。
- **感知链路**：LSM6DSR IMU 中断驱动 Mahony 姿态解算，八路灰度 10ms 后台采样，视觉 UART1 接收球位置偏差。
- **执行器链路**：直流电机驱动底盘，ZDT-X42S 闭环步进电机作为题目 3 摆杆执行器。

仓库根目录当前只有该 CCS 工程和压缩包副本；旧文档中提到的外部 `Test/` 子项目和 `D:\MSPM0Project\` PC 工具不在当前工作区内。

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
  ├── NVIC_EnableIRQ(UART_1/2)   — 开启视觉 UART1 与 UART2 CPU 中断响应
  ├── Encoder_Init()             — 开启编码器/多路 GPIO 中断与全局中断
  ├── RTOS_Tasks_Init()          — 创建互斥锁和 FreeRTOS 任务
  └── vTaskStartScheduler()      — 启动调度器, 正常不返回
```

`RTOS_Tasks_Init()` 是运行期核心装配点：先创建 `target_ctrl_Mutex`，再按功能宏条件创建 IMU (`USE_IMU_SENSOR`)、底盘控制、CPU 监控、菜单按键、灰度采样、OLED、蓝牙 (`USE_BLUETOOTH`)、蜂鸣器和 ZDT 摆杆 (`USE_ZDT_STEPPER`) 任务。当前 `USE_IMU_SENSOR=0`、`USE_BLUETOOTH=0`、`USE_VOFA_DEBUG=0`，IMU 任务、蓝牙任务和 ZDT_Test 中的 VOFA/CSV 调试输出均未启用。若关键任务创建失败，代码会关中断、停电机、蜂鸣器/LED 报警并软复位。

### FreeRTOS 任务总览 (按创建顺序)

| 任务名 | 优先级 | 堆栈 | 功能 |
|--------|-------|------|------|
| **IMU_Task** | 4 | 768w | ⚠️ 已禁用 (USE_IMU_SENSOR=0) — LSM6DSR 初始化→热稳定→零偏校准→Mahony 姿态解算，GPIO INT 通知触发运行 |
| **Ctrl_Task** | 2 | 512w | 50ms 周期底盘主控制：蓝牙/yaw/位置/循迹目标 → 编码器反馈 → 并联 PID → PWM 输出 |
| **CPU_Monitor** | 1 | 256w | 每秒统计各任务 CPU 占用率、堆栈水位和剩余堆 |
| **KeyScan** | 2 | 128w | 10ms 按键扫描；OLED 菜单选择题目 2~6；S4 确定，S2 返回/停止 |
| **Sonar_Task** | 2 | 256w | 条件启用；超声波 Trig/Echo 测距，当前 `USE_ULTRASONIC=0` 不创建 |
| **GrayTask** | 2 | 128w | 10ms 读取八路灰度，计算 `g_gray_raw_data/g_gray_error` 并锁存停止线 |
| **OLED_Task** | 1 | 768w | OLED 菜单/运行界面刷新，20ms 周期 |
| **BT_Task** | 3 | 128w | ⚠️ 已禁用 (USE_BLUETOOTH=0) — UART3 蓝牙接收队列 + 帧解析，填充 `g_bt_cmd` |
| **Heartbeat** | 1 | 128w | LED 500ms 翻转，系统存活指示 |
| **Buzzer** | 2 | 128w | 按 `g_bt_beep_cmd` 执行短鸣/长鸣提示 |
| **ZDT_Test** | 2 | 512w | ZDT-X42S 初始化、题目 3 视觉闭环摆杆控制（VOFA 调试输出已禁用: USE_VOFA_DEBUG=0） |

### 电机控制架构 (Ctrl_Task 核心)

```
外层目标来源：蓝牙指令 / yaw 相对转向 / 位置环 / 循迹 / 题目菜单
      ↓ target_ctrl_Mutex 保护 target_speed、target_turn
Ctrl_Task 每 50ms:
  ├── Encoder_Get(1/2) 读取并清零左右编码器脉冲
  ├── speed_L/R = raw * 100 / MAX_PULSE_50MS
  ├── ave_speed = (L+R)/2, dif_speed = L-R 并低通/限速
  ├── [速度PID] Target=target_speed, Actual=ave_speed → ave_pwm
  ├── [转向PID] Target=target_turn,  Actual=dif_speed_filter → dif_pwm
  ├── PWM_L = ave_pwm + dif_pwm/2, PWM_R = ave_pwm - dif_pwm/2
  └── Motor_SetPWM(1/2, pwm) → TIMG6/TIMG8 双路 PWM
```

当前 PID 参数：
- 速度环：`Kp=3.0, Ki=0.5, Kd=0.0, Out=±70, I=±130`
- 转向环：`Kp=1.2, Ki=0.0, Kd=0.0, Out=±45, I=±60`

核心特性：
- 速度环 + 转向环**并联**运行，各自独立 PID 计算后差速合成。
- `target_speed` 先被限制到 ±30，`target_turn` 先被限制到 ±24，小转向目标 `±9.9` 内归零。
- 电子手刹：目标为 0 且反馈接近停稳时清空 PID 积分和输出，防止摩擦死区引起抽搐。
- `Motor_SetPWM()` 再做最终 ±100 物理限幅，并按快衰减/慢衰减模式写入左右电机 PWM 比较值。

### 题目菜单与外层策略逻辑

OLED 菜单由 `Key_Scan_Task` 和 `Task_OLED_Display` 共同驱动：

```
开机: g_question_ui_state=0, g_selected_question=2
菜单态:
  S2 → 上一个题目, 2 以下回到 6
  S3 → 下一个题目, 6 以上回到 2
  S4 → 确认并进入运行态
运行态:
  S2 → Chassis_StopAll(), 停止循迹/位置环/目标速度/秒表, 返回题目2菜单
```

题目入口：
- **题目 2**：`LineTrace_Start(&g_line_trace_q2_config)`，独立循迹参数，直线 16、弯道 12。
- **题目 3**：菜单进入运行态；`ZDT_Test` 任务检测到题目 3 后接管视觉闭环摆杆，当前目标为中心 `0px` 稳定。
- **题目 4**：`Chassis_MoveRelativeCmLineTrace(165cm, 11, &g_line_trace_s2_config)`，位置环决定终点，灰度循迹只负责转向纠偏。
- **题目 5/6**：`LineTrace_StartSmoothWithStep(&g_line_trace_q56_config, 0.5f)`，共用循迹参数，直线/弯道均 10，起步每 50ms 增加 0.5。

### 循迹核心逻辑

灰度传感器任务每 10ms 更新：
- `g_gray_raw_data`：8 位黑线位图，Bit0~Bit7 对应 OUT1~OUT8。
- `g_gray_error`：按物理位置加权的偏差，权重 `{-550,-350,-120,-50,50,120,350,550}`。
- `g_gray_stop_line_latched`：停止线短脉冲锁存 5 个采样周期，避免 Ctrl_Task 50ms 周期漏采。

`LineTrace_Update()` 在 Ctrl_Task 中运行：
1. 根据黑点数量、侧边三连黑或锁存标志判断停止线。
2. 启动后先忽略 `ignore_cycles`，防止起点黑线误停。
3. `LineTrace_CalcTurn()` 对误差做 0.2/0.8 低通，根据迟滞阈值进入/退出弯道模式。
4. 弯道模式下使用 `LINE_DEFAULT_TURN_KP_SCALE=0.40` 降低 KP，并用 `LINE_CURVE_ERROR_HOLD=120` 保持同向小跳变误差，降低相邻探头交替造成的摆头。
5. 速度在直线/弯道目标之间按 `g_line_trace_smooth_step` 平滑变化；最终写入 `target_speed/target_turn`。

### 位置环与循迹纠偏

`Chassis_MoveRelativeCmInternal()` 把距离换算为编码器脉冲目标：

```
g_pos_target_pulse = distance_cm * POSITION_PULSE_PER_CM
POSITION_PULSE_PER_CM = 15.4
POSITION_KP = 0.12
POSITION_SPEED_MIN = 5
POSITION_TOLERANCE_PULSE = 8
```

Ctrl_Task 每周期用左右轮原始脉冲平均值累加 `g_pos_current_pulse`，位置误差经 P 控制得到 `target_speed`。题目 4 会开启 `g_pos_line_trace_assist`，此时 `target_turn = LineTrace_CalcTurn()`，所以“前进距离/停车”由位置环控制，“走直/沿线”由灰度纠偏控制。

### 题目 3 视觉闭环摆杆逻辑

`UART_1_INST_IRQHandler()` 接收视觉模块发送的 ASCII 有符号整数行，以 `\n` 结束，解析为 `g_vision_x_offset` 并置位 `g_vision_ready_flag`。

`ZDT_Test` 任务初始化 ZDT-X42S 和摆杆执行器后循环：
- 只有 `g_question_ui_state==1 && g_selected_question==3` 时进入比赛控制。
- 首帧视觉数据建立零位偏置，后续按 `Q3_ZERO_BIAS_PX`、`Q3_ZERO_DEADBAND_PX` 做补偿和死区。
- 位置低通 `Q3_POS_FILTER_ALPHA=0.35`，速度低通 `Q3_VEL_FILTER_ALPHA=0.25`。
- 当前目标位置固定为 `target_pos_px = 0.0f`，用于中心稳定；虽然代码保留了 `Q3_TARGET_5CM_PX`、`Q3_TARGET_STEP_PX`、到达确认和保持周期等参数，但尚未接入 O→+5cm→-5cm 目标序列。
- 控制律是视觉 PD：`rod_cmd = Q3_KP_PULSE_PER_PX * pos_error - Q3_KD_PULSE_PER_PX * ball_vel`，没有使用通用 `PID_Update()`，也没有积分项。
- 输出经摆杆脉冲限幅/步进限速后发送 `ZDT_MoveAbsolute()`；视觉丢失超过 `Q3_VISION_LOST_CYCLES` 则回中。

### IMU 姿态解算逻辑

IMU_Task 由 GPIO 中断通知驱动，核心流程：
1. `LSM6DSR_Init()`：SPI 配置 104Hz 加速度/陀螺、BDU、INT 数据就绪、LPF 等。
2. WHO_AM_I 自检失败则 LED 快闪并挂起。
3. 上电热稳定 3 秒，再进行陀螺零偏校准；校准用 P-P 阈值检测静止，最多重试 5 次。
4. 每次 INT 通知后 burst read 12 字节，换算为 g 和 dps，扣除零偏。
5. 按芯片面朝下、绕 X 轴 180° 的安装方式变换到小车坐标系。
6. 静止锁定时陀螺清零、Mahony 积分关闭，运动后恢复积分，减少静止 yaw 漂移。
7. 使用 `IMU_dt` 定时器计算真实 dt，调用 Mahony 更新四元数并输出连续 yaw。

### 中断服务

`GROUP1_IRQHandler` 同时处理：
1. **左编码器** A 相 (PB13, LA)：根据 B 相 (PB12, LB) 电平判向，更新 `encoder_L`。
2. **右编码器** A 相 (PA9, RA)：根据 B 相 (PA8, RB) 电平判向，更新 `encoder_R`。
3. **IMU GPIO 中断**：发出 `vTaskNotifyGiveFromISR` 唤醒 IMU_Task。
4. **超声波 Echo**：当前 `USE_ULTRASONIC=0`，代码保留 ISR 转发入口。

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
| 八路灰度 | GPIO 输入, 8 通道 | OUT1-OUT8 = PA27/26/25/24, PB25/24/20, PA22 |
| OLED I2C | GPIO OD 开漏, 软件模拟 | SDA=PA0, SCL=PA1 |
| HC-05 EN/STATE | GPIO | EN=PA15, STATE=PA16 |
| UART1 视觉输入 | UART1, RX 中断 | 接收 ASCII `x_offset\n`, `UART_1_INST_IRQHandler` 解析 |
| UART2 字符串/printf/题目3调试 | UART2, 115200, RX 中断/阻塞发送 | `[...*]` 字符串协议、`printf` 重定向与题目 3 CSV 调试输出 |
| UART3 蓝牙 | UART3, RX 中断 | `hc05.c` 蓝牙接收；已移除旧 UART3 VOFA JustFloat 调试任务 |
| ZDT UART | UART0, 115200, RX 中断 | TX=PA10, RX=PB1 |
| 按键 S4 | GPIO 输入, 上拉 | PB21 |
| LED | GPIO 输出 | PB22 |
| 蜂鸣器 | GPIO 输出 | PB0 |
| 运行统计 Timer | TIMG12, PERIODIC_UP, ÷8 | — |

### 用户模块头文件体系

所有模块通过 `alldata.h` 汇聚引用，工程中任何 `.c` 文件只需 `#include "alldata.h"` 即可获取:
- 所有标准库 (`stdint`, `stdbool`, `math`, `stdio` 等)
- `ti_msp_dl_config.h` (SysConfig 自动生成)
- FreeRTOS (`FreeRTOS.h`, `task.h`, `queue.h`, `semphr.h`)
- 所有用户驱动模块头文件
- 全局共享变量 extern 声明 (`roll`, `pitch`, `yaw`, `g_ctrl_mode` 和 debug 变量)

### 核心源码分工

| 文件 | 核心职责 |
|------|----------|
| `empty.c` | 程序入口；硬件初始化、UART 中断开启、编码器初始化、任务创建和 RTOS 启动；FreeRTOS malloc/栈溢出钩子负责紧急停车复位 |
| `user/rtos_tasks.c` | 项目主逻辑集中地；任务创建、底盘控制、循迹、位置环、OLED 菜单、IMU 任务、ZDT/题目3任务、GROUP1 ISR |
| `user/Sensor.c` | 八路灰度采样、误差计算、停止线锁存 |
| `user/encoder.c` | 编码器计数读取与清零；实际计数累加在 `GROUP1_IRQHandler` 中完成 |
| `user/motor.c` | 将 ±100 抽象 PWM 转成左右电机双输入比较值，支持快衰减/慢衰减 |
| `user/uart.c` | UART1 视觉整数行解析；UART2 `[...*]` 字符串指令解析和 printf 重定向 |
| `user/hc05.c` | UART3 蓝牙字节队列与 `0xAA 0x55 ... 0xFF` 控制帧解析 |
| `user/lsm6dsr.c` | IMU SPI 寄存器配置、burst read、单位换算、WHO_AM_I 检测 |
| `user/mahony.c` | Mahony IMU 融合、积分管理、欧拉角和连续 yaw 输出 |
| `user/ZDT_X42S.c` | ZDT-X42S 串口协议、反馈帧接收队列和基础运动命令 |
| `user/vofa.c` | VOFA FireWater/JustFloat 数据打包发送与接收缓冲框架 |

## Coding Conventions

- 修改代码时必须写修改日志（原因 + 修改内容 + 仍需实车验证），参考本文档修改日志部分的格式
- 浮点运算在 M0+ 上为软件模拟，避免在中断或高频循环中使用 `sqrtf`/`sinf`
- PID 参数调整建议小步进行（如 KP 变化 ≤0.05），每次调整后需实车验证
- 循迹参数分题目独立配置，修改某题参数不得影响其他题目
- 正式测试期间蓝牙 (HC-05) 必须禁用或确保不参与控制

## Shared Data Flow

关键数据通过 `volatile extern` 全局变量在任务间共享 (`alldata.h` 声明, `rtos_tasks.c` 定义):

```
IMU_Task            → roll/pitch/yaw             — 供 yaw 相对转向和调试观察使用
Encoder ISR         → encoder_L/R 累积脉冲        — Ctrl_Task 每 50ms 读取、清零并换算轮速/位置
GrayTask            → g_gray_raw_data/error/latched — 循迹偏差和停止线判断
KeyScan             → g_question_ui_state/selected — OLED 菜单状态和题目启动入口
UART1 ISR           → g_vision_x_offset/ready    — 题目 3 摆杆视觉闭环输入
BT_Task             → g_bt_cmd                   — Ctrl_Task 消费蓝牙控制帧
Ctrl_Task           → target_pwm_L/R             — motor.c 转换为 PWM 占空比
ZDT_Test            → ZDT_MoveAbsolute           — 题目 3 摆杆位置命令
Buzzer_Task         ← g_bt_beep_cmd (0/1/2)      — 短鸣/长鸣提示
OLED_Task           ← 菜单/秒表/视觉状态          — 屏幕显示
```

## Hardware Constraints

- **MSPM0G3507**: Cortex-M0+, 80MHz, 128KB Flash, 32KB SRAM
- **LSM6DSR**: SPI 通信, CS=PB6, INT1=PB14 (Data Ready 信号, 1.04kHz 采样)
- **ZDT-X42S**: 闭环步进电机, UART0 驱动, 自定义二进制协议 (`ZDT_X42S.c/h`)
- **HC-05**: 蓝牙串口, 自定义帧格式 (0xAA 0x55 ... 0xFF)
- **八路灰度**: 原 12 路去掉第 2/4/9/11 路后保留 8 路，权重 `{-550,-350,-120,-50,50,120,350,550}`，Bit0~Bit7 对应 OUT1~OUT8
- **超声波**: HC-SR04 兼容, Trig+Echo 时序, Timer 捕获脉宽

## 已知 Bug 与隐患 (Known Issues)

### 🟡 仍存在: 时钟常量硬编码 vs SysConfig 分离

`IMU_DT_FREQ_HZ (50000.0f)` 和 `IMU_DT_WRAP_VALUE (50000)` 在 `rtos_tasks.c` 中硬编码，必须与 SysConfig 中 `IMU_dt` 定时器保持一致。当前代码注释要求 `IMU_dt` 为 BUSCLK，经 `/8` 和预分频 `99` 得到 50kHz，周期 49999，1 秒回绕。

同理 `configCPU_CLOCK_HZ (80000000)` 在 `FreeRTOSConfig.h` 中硬编码，必须手动与 SysConfig 时钟树保持一致。

### 🟡 注意: 题目 3 串口文本调试输出

题目 3 的 VOFA 文本 CSV 调试输出由 `#if USE_VOFA_DEBUG` 条件编译保护，当前 `USE_VOFA_DEBUG=0`，因此 `zdt_motor_test_task()` 中的 `VOFA_SendString()` 调用未被编译，ZDT_Test 任务不输出任何调试数据。需要调试时，将 `user/rtos_tasks.h` 中 `USE_VOFA_DEBUG` 改为 `1` 并重新编译。UART2 仍负责 `printf` 重定向和字符串命令接收。VOFA 字段、分析脚本和诊断方法统一维护在根目录 `.trae/documents/vofa分析脚本编写指导.md`。

### 🟡 注意: 题目 3 钢球控制仍是基础视觉 PD

`zdt_motor_test_task()` 当前只做中心 `0px` 稳定，控制律为位置误差 P 项 + 视觉速度 D 项。尚未接入 O→+5cm→-5cm 自动目标序列，也未结合小车速度、加速度或转弯状态做行驶中稳球前馈。任务 4/5 的行驶中稳球和任务 6 的指定位置保持仍需要继续完善。

### ✅ 已修复: 题目 3 视觉数据处理消费结构 (2026-08-01)

`zdt_motor_test_task()` 原先在循环前半段和题目 3 分支内两处检查 `g_vision_ready_flag`，存在双消费和两套 `raw_ball_pos_px` 公式不一致问题。已修复为：PRIMASK 临界区内唯一快照入口 → 统一视觉处理块（含 `Q3_ZERO_BIAS_PX`）→ 题目 3 分支只消费 `ball_pos_px/ball_vel_px/vision_lost_count` 状态量。视觉丢失回中时间恢复为设计的 `Q3_VISION_LOST_CYCLES × 50ms`。

## 修改日志

### 2026-08-01: 题目3小球卡死检测与突破脉冲

**修改背景**: 题目3视觉闭环时, 小球可能在非中心位置发生机械卡死。此时视觉仍持续输出偏移值, 但普通 PD 控制不足以让小球重新移动, 需要在检测到非中心静止后给摆杆一次更大的突破动作。

**修改内容**:
- `user/rtos_tasks.c`: 新增 `Q3_STUCK_POS_THRESHOLD_PX`、`Q3_STUCK_VEL_THRESHOLD_PX`、`Q3_STUCK_CONFIRM_COUNT`、`Q3_BREAKTHROUGH_PULSE` 和 `Q3_BREAKTHROUGH_COOLDOWN_COUNT`。原因是用视觉位置偏离、滤波速度和连续周期数定义卡死, 并给突破动作设置独立幅度和冷却窗口。
- `user/rtos_tasks.c`: 新增 `RodActuator_SetTargetPulseFast()`。原因是突破脉冲需要绕过普通 `RodActuator_SetTargetPulse()` 的单周期限步, 但仍必须经过绝对行程限幅并保持 `g_rod_target_pulse` 与下发位置一致。
- `user/rtos_tasks.c`: 在 `zdt_motor_test_task()` 中增加 `stuck_count` 和 `breakthrough_cooldown`。原因是在题目3运行态内检测“视觉持续有效 + 非中心 + 低速近似静止”的连续状态, 触发后发送一次 ±70 脉冲突破目标, 并在冷却期间避免连续冲击。
- `user/rtos_tasks.c`: VOFA CSV 调试字段末尾追加 `stuck_count` 和 `breakthrough_cooldown`。原因是实车调试时需要观察卡死计数是否误触发以及突破冷却是否按预期递减。

**仍需实车验证**:
- 小球停在非中心位置约 100ms 后应触发一次突破, 中心附近不应误触发。
- 突破脉冲方向必须确认: 若触发后小球更卡或远离中心, 需要反转突破方向逻辑或重新校准视觉/控制符号。
- `Q3_BREAKTHROUGH_PULSE=70`、`Q3_STUCK_CONFIRM_COUNT=5U`、`Q3_STUCK_VEL_THRESHOLD_PX=2.0f` 需要根据机构摩擦和视觉抖动实车微调。

### 2026-07-31: 题目5/6专属降速和平稳起步

**修改背景**: 用户要求题目5和题目6采用平稳起步, 同时题目5/6直行和转向速度都为 `10.0f`。后续明确本次更改不能影响题目4, 因此题目4的位置环参数保持原值。

**修改内容**:
- `user/rtos_tasks.c`: 将题目5/6共用配置 `g_line_trace_q56_config` 的 `straight_speed` 和 `turn_speed` 都改为 `10.0f`。原因是题目5/6需要以相同的较低速度进行直线和转弯。
- `user/rtos_tasks.c`: 保持 `POSITION_S2_SPEED_MAX=11.0f` 和 `POSITION_SMOOTH_STEP=0.20f` 不变。原因是这两个参数属于题目4的位置环直行动作, 本次题目5/6调参不能影响题目4。
- `user/rtos_tasks.c`: 新增 `g_line_trace_smooth_step` 和 `LineTrace_StartSmoothWithStep()`。原因是题目5/6需要使用 `0.5f` 的循迹速度爬升步进, 但不能改变其他循迹动作原有的 `LINE_SPEED_STEP`。
- `user/rtos_tasks.c`: 题目5/6确定后调用 `LineTrace_StartSmoothWithStep(&g_line_trace_q56_config, 0.5f)`。原因是让两题以独立参数启动, 并从 0 开始每 50ms 平滑爬升 0.5 到目标速度。

**仍需实车验证**:
- 题目4应保持原位置环速度和起步表现不变。
- 题目5/6应使用直行 10、转向 10, 且起步每 50ms 增加 0.5。

### 2026-07-31: OLED 题目菜单控制循迹动作

**修改背景**: 用户不再使用按键直接控制小车运行, 改为通过 OLED 选择题目并由按键完成菜单操作。题目2对应原 S4 循迹, 题目4对应原 S2 位置环加循迹纠偏, 题目3暂未配置电机动作, 题目5和题目6共用一套但独立于题目2和原 S4 的循迹参数。

**修改内容**:
- `user/rtos_tasks.c`: 新增题目2独立配置 `g_line_trace_q2_config`。原因是题目2虽然初始复制原 S4 参数, 后续调参不能影响题目5/6或原 S4。
- `user/rtos_tasks.c`: 新增题目5/6共用配置 `g_line_trace_q56_config`。原因是题目5和题目6要求共用一套循迹参数, 同时与题目2和原 S4 参数分开。
- `user/rtos_tasks.c`: 重写 `Key_Scan_Task`, S2 在菜单中返回, S3 切换下一题并循环, S4 确定当前题目; 运行界面按 S2 停止动作并返回菜单。
- `user/rtos_tasks.c`: 重写 `Task_OLED_Display`, 初始显示题目2到题目6并用 `>` 标记选中项, 确定后显示题目编号和计时。
- `user/rtos_tasks.c`: 题目2启动 `g_line_trace_q2_config`, 题目3仅启动计时不启动电机, 题目4复用 S2 位置环终点和循迹纠偏, 题目5/6启动 `g_line_trace_q56_config`。
- `user/rtos_tasks.c`: 新增 `Chassis_StopAll`, 确保从计时运行界面返回菜单时清除循迹、位置环、目标速度、目标转向和秒表状态。

**仍需实车验证**:
- 开机 OLED 默认选中题目2; S3 可按题目2→3→4→5→6→2循环, S4 确定。
- 题目3确定后应保持电机停止, S2 返回菜单。
- 题目2、5、6使用各自映射的循迹参数, 修改 `g_line_trace_q56_config` 应同时影响题目5和题目6, 不影响题目2和原 S4。

### 2026-07-31: S2 位置循迹降速

**修改背景**: 用户实车将 S2 距离调到 `165.0f` 后距离达到要求, 但起步过程和直行速度偏快, 需要在保持当前距离不变的前提下降低速度。

**修改内容**:
- `user/rtos_tasks.c`: 保持 `POSITION_S2_DISTANCE_CM=165.0f` 不变。原因是该距离已经满足实车要求。
- `user/rtos_tasks.c`: 将 `POSITION_S2_SPEED_MAX` 从 `18.75f` 降为 `15.0f`。原因是降低 S2 位置环直行阶段的最高目标速度, 让直行过程更慢。
- `user/rtos_tasks.c`: 将 `POSITION_SMOOTH_STEP` 从 `1.25f` 降为 `0.75f`。原因是降低每 50ms 的起步速度爬升幅度, 让电机起步更柔和。

**仍需实车验证**:
- 若 S2 仍偏快, 优先继续降低 `POSITION_S2_SPEED_MAX` 到 `13.0f~14.0f`。
- 若起步仍冲, 继续降低 `POSITION_SMOOTH_STEP`; 若起步太慢, 可小幅回调到 `1.0f`。

### 2026-07-31: S2 位置循迹增加计时并加长距离

**修改背景**: 用户要求 S2 按下后也能计时, 到达终点后停止, 同时让小车多走一点距离。当前 S2 已由位置环控制终点并由循迹负责纠偏, 到达终点时已有清目标停车逻辑, 但未接入秒表计时。

**修改内容**:
- `user/rtos_tasks.c`: 将 `POSITION_S2_DISTANCE_CM` 从 `150.0f` 调为 `155.0f`。原因是让 S2 位置环目标小幅加长, 小车能比原先多走约 5cm。
- `user/rtos_tasks.c`: 新增 `g_pos_stopwatch_active`。原因是只让 S2 这种位置循迹组合动作启动/停止秒表, 避免普通位置运动误触发计时。
- `user/rtos_tasks.c`: 在 `Chassis_MoveRelativeCmLineTrace()` 成功启动位置运动后调用 `Stopwatch_Start()`。原因是 S2 按键按下并确实进入位置运动后开始计时。
- `user/rtos_tasks.c`: 在位置环到达终点 `fabsf(pos_error) <= POSITION_TOLERANCE_PULSE` 时调用 `Chassis_ClearTarget()` 停车, 并在 `g_pos_stopwatch_active` 有效时调用 `Stopwatch_Stop()`。原因是终点到达后同时停止电机目标和计时。

**仍需实车验证**:
- S2 应从按键启动开始计时, 到达 155cm 位置终点后停车并停止计时。
- 若实车距离仍偏短或偏长, 优先继续微调 `POSITION_S2_DISTANCE_CM`, 其次重新标定 `POSITION_PULSE_PER_CM`。

### 2026-07-31: S2 位置环直行增加循迹纠偏

**修改背景**: 用户反馈 S2 仅靠位置环直行容易走歪, 需要在不改变 1.5 米终点的前提下加入循迹纠偏。由于终点必须保持由位置环决定, 循迹逻辑只应用于转向修正, 不能重新接管前进距离。

**修改内容**:
- `user/rtos_tasks.c`: 恢复 `g_line_trace_s2_config`, 但用途改为 S2 位置环直行时的循迹纠偏参数。原因是 S2 仍需要灰度误差、KP、弯道判断等循迹参数计算 `target_turn`, 但不再用循迹速度决定路程终点。
- `user/rtos_tasks.c`: 新增 `g_pos_line_trace_assist` 状态。原因是区分普通位置运动和 S2 的“位置环 + 循迹纠偏”运动, 避免普通 `Chassis_MoveRelativeCm()` 被循迹转向影响。
- `user/rtos_tasks.c`: 将循迹转向计算拆为 `LineTrace_CalcTurn()`, 普通 `LineTrace_Update()` 继续使用该函数计算循迹转向。原因是让 S2 位置环可以复用同一套灰度纠偏计算, 同时保留 S4/S1 原循迹行为。
- `user/rtos_tasks.c`: 新增 `Chassis_MoveRelativeCmLineTrace(distance_cm, speed_max, line_config)`, 启动位置环时加载 S2 循迹配置并开启纠偏。原因是 S2 的 `target_speed` 仍由位置环和 150cm 目标决定, `target_turn` 则由循迹误差决定, 从而减少走歪且不改变终点。
- `user/rtos_tasks.c`: 将 S2 按键入口改为调用 `Chassis_MoveRelativeCmLineTrace(POSITION_S2_DISTANCE_CM, POSITION_S2_SPEED_MAX, &g_line_trace_s2_config)`。原因是 S2 现在执行“平滑起步 + 位置环 1.5 米 + 循迹纠偏”的组合动作。
- `user/rtos_tasks.c`: 删除不再使用的 `Chassis_MoveRelativeCmSmooth()` 静态函数。原因是 S2 已切换到带循迹纠偏的位置运动入口, 保留旧静态函数可能再次触发未使用 warning。

**仍需实车验证**:
- S2 应按 1.5 米位置环终点停车, 同时沿线纠偏减少走歪。
- 若 S2 纠偏过猛导致摆头, 优先降低 `g_line_trace_s2_config` 的 `kp` 或提高 `turn_deadband`。
- 若 S2 仍走歪, 优先检查灰度线是否连续, 再小幅提高 `g_line_trace_s2_config` 的 `kp`。

### 2026-07-31: 清理 S2 未使用循迹配置

**修改背景**: S2 按键已切换为位置环直行 1.5 米, 原 `g_line_trace_s2_config` 不再被引用, TI Arm Clang 编译时报 `unused variable 'g_line_trace_s2_config'` warning。

**修改内容**:
- `user/rtos_tasks.c`: 删除未使用的 `g_line_trace_s2_config` 配置块及其注释。原因是 S2 当前不再启动循迹路线, 保留该静态 const 会触发 `-Wunused-const-variable` 编译警告。

**仍需实车验证**:
- S2 应继续执行位置环直行 1.5 米; S1/S4 循迹入口不受影响。

### 2026-07-31: S2 平滑位置环直行 1.5 米

**修改背景**: 用户要求 S2 按键按下后不再按循迹路线运行, 而是在平滑启动的情况下, 使用已经封装好的位置环在 8 秒内走完 1.5 米直线。

**修改内容**:
- `user/rtos_tasks.c`: 新增 `POSITION_S2_DISTANCE_CM=150.0f` 和 `POSITION_S2_SPEED_MAX=18.75f`。原因是 S2 专属目标为 150cm, 8 秒完成对应平均速度约 18.75cm/s, 不修改普通位置环默认速度上限。
- `user/rtos_tasks.c`: 将 `Chassis_MoveRelativeCm()` 拆为内部 `Chassis_MoveRelativeCmInternal(distance_cm, smooth_start, speed_max)`, 普通接口仍使用原 `POSITION_SPEED_MAX`。原因是复用已有位置环计算, 同时给 S2 单独传入平滑启动和速度上限。
- `user/rtos_tasks.c`: 新增 `g_pos_smooth_start/g_pos_speed_filter/g_pos_speed_max`, 在位置环输出 `pos_speed` 后按 `POSITION_SMOOTH_STEP=1.25f` 逐 50ms 爬升。原因是让 S2 起步阶段不从 0 直接跳到位置环目标速度, 降低电机速度突变。
- `user/rtos_tasks.c`: 将 S2 按键分支改为调用 `Chassis_MoveRelativeCmSmooth(POSITION_S2_DISTANCE_CM, POSITION_S2_SPEED_MAX)`。原因是 S2 现在执行位置环直行 1.5 米, 不再启动 S2 循迹配置。

**仍需实车验证**:
- 若 8 秒内走不完 1.5 米, 优先上调 `POSITION_S2_SPEED_MAX` 或重新标定 `POSITION_PULSE_PER_CM`。
- 若起步仍冲, 降低 `POSITION_SMOOTH_STEP`; 若起步太慢影响 8 秒目标, 提高 `POSITION_SMOOTH_STEP`。

### 2026-07-31: S1/S2 循迹平滑起步

**修改背景**: 用户希望 S1、S2 仍保持当前循迹参数不变, 但这两个按键启动循迹时电机不要从 0 直接跳到目标速度, 需要专属平滑起步逻辑降低速度突变。

**修改内容**:
- `user/rtos_tasks.c`: 新增 `g_line_trace_smooth_start` 状态和 `LineTrace_StartInternal(config, smooth_start)`。原因是区分普通循迹启动和 S1/S2 专属平滑启动, 不改变 `LineTrace_Config_t` 参数结构和 S1/S2 参数值。
- `user/rtos_tasks.c`: 将速度滤波首次赋值逻辑改为: 普通启动仍直接进入目标速度, 平滑启动则从 `0.0f` 开始按 `LINE_SPEED_STEP=1.0f` 逐周期爬升到目标速度。原因是只限制 S1/S2 起步阶段速度变化, 不影响后续弯道/直线速度切换逻辑。
- `user/rtos_tasks.c`: 将 S1、S2 按键分支改为调用带步进参数的平滑启动入口, S4 仍调用 `LineTrace_Start()`。原因是平滑起步只作用于 S1/S2, S4 的现有起步行为保持不变。

**仍需实车验证**:
- 若 S1/S2 起步仍偏冲, 可降低 `LINE_SPEED_STEP` 或增加单独的起步步进值。
- 若 S1/S2 起步太慢影响路线时序, 可提高 `LINE_SPEED_STEP` 或给平滑起步设置更高初始速度。

### 2026-07-31: S3 循迹切换到 S1

**修改背景**: 用户当前不再使用 S3 作为循迹启动按键, 需要把原路线 2 的循迹入口切换到 S1。

**修改内容**:
- `user/rtos_tasks.c`: 将原 `g_line_trace_s3_config` 重命名为 `g_line_trace_s1_config`, 并把对应注释从 S3 改为 S1。原因是路线 2 参数现在由 S1 按键使用。
- `user/rtos_tasks.c`: 将按键扫描任务中路线 2 的触发条件从 `key_val == 3` 改为 `key_val == 1`, 并调用 `g_line_trace_s1_config`。原因是 `Key_GetNum()` 返回值 1 对应 S1, 3 对应 S3。

**仍需实车验证**:
- 按下 S1 应启动路线 2 循迹, 按下 S3 不应再启动该循迹路线。

### 2026-07-31: S4 直行速度调整为 16

**修改背景**: 用户需要提高 S4 路线循迹直线段速度, 同时不影响 S3/S2 预留路线参数。

**修改内容**:
- `user/rtos_tasks.c`: 将 `g_line_trace_s4_config` 的 `straight_speed` 从 `LINE_DEFAULT_STRAIGHT_SPEED` 改为 `16.0f`, 并同步更新 S4 参数注释。原因是只让 S4 使用更高直行速度, 不修改 `LINE_DEFAULT_STRAIGHT_SPEED`, 避免 S3/S2 跟随变化。

**仍需实车验证**:
- 若直线速度提升后停车线漏停或入弯过冲增加, 优先降低 S4 直行速度或增加停车线保持周期。

### 2026-07-31: 停止线 10ms 峰值保持

**修改背景**: 实车高速通过较窄停止线时, 灰度任务每 10ms 能采到停止线, 但循迹停车判断在 `LineTrace_Update()` 中随控制任务运行, 控制周期较慢时可能刚好错过停止线最黑的一帧, 导致到停止线不停止。

**修改内容**:
- `user/Sensor.c`: 新增 `g_gray_stop_line_latched` 和 `GRAY_STOP_LINE_HOLD_CYCLES=5`, 在 `Gray_Task()` 中每 10ms 判断停止线形态, 命中后保持 5 个采样周期。原因是把停止线短脉冲保持约 50ms, 让循迹控制周期能读到最近经过过停止线。
- `user/Sensor.h`: 新增 `g_gray_stop_line_latched` 的 extern 声明。原因是让 `rtos_tasks.c` 能读取灰度任务保持的停止线标志。
- `user/rtos_tasks.c`: 在 `LineTrace_Update()` 中把 `g_gray_stop_line_latched` 纳入停车条件, 并在启动忽略周期、停车触发、循迹状态重置时清零该标志。原因是保留原有启动忽略逻辑, 同时避免历史停止线标志残留造成下一次循迹误停。

**仍需实车验证**:
- 若仍有漏停, 可把 `GRAY_STOP_LINE_HOLD_CYCLES` 从 `5` 调到 `6~8`, 让保持时间略长。
- 若出现误停, 优先把 `GRAY_STOP_LINE_HOLD_CYCLES` 从 `5` 降到 `3~4`, 或提高 `stop_line_black_min`。

### 2026-07-30: 回退弯道输出限速并改用误差保持

**修改背景**: 上一版增加弯道转向输出限速和反向修正限制后, 实车表现为转弯动力明显不足。用户询问是否需要调整 `Kd`。分析认为当前主要干扰来自灰度传感器 8/10 交替触发造成的误差跳变, 直接增加 `Kd` 会对误差变化率敏感, 反而可能放大传感器闪烁带来的抖动, 因此本次不修改 `pid_turn.Kd`。

**修改内容**:
- `user/rtos_tasks.c`: 移除 `LINE_TURN_OUTPUT_STEP`、`g_line_trace_turn_output`、`g_line_trace_turn_dir` 以及弯道反向修正限制逻辑。原因是这些限制直接压制最终转向输出, 实车已验证会削弱转弯动力。
- `user/rtos_tasks.c`: 新增 `LINE_CURVE_ERROR_HOLD=90` 和 `g_line_trace_curve_error`。原因是把抗抖位置从“输出端”前移到“误差端”, 在弯道中如果新误差与上一弯道误差同向且变化小于阈值, 沿用上一误差, 减少相邻探头交替造成的转向量跳变。
- `user/rtos_tasks.c`: 在退出弯道或重置循迹状态时清零 `g_line_trace_curve_error`。原因是误差保持只应作用于当前弯道, 不能影响下一段直线或下一次循迹启动。

**仍需实车验证**:
- 若弯道动力恢复但仍小幅摆头, 可把 `LINE_CURVE_ERROR_HOLD` 从 `90` 调到 `120`, 让相邻探头造成的小跳变更难改变控制误差。
- 若弯道跟随变迟钝, 可把 `LINE_CURVE_ERROR_HOLD` 从 `90` 降到 `60`。
- 暂不建议给 `pid_turn.Kd` 加值；若后续要试, 也应从很小值开始, 如 `0.05f`, 并且只在传感器状态稳定后再验证。

### 2026-07-30: 弯道相邻探头交替防摆优化

**修改背景**: 小车从直线进入弯道时, 传感器状态会从 6/7 都灭逐渐变为 8 和 10 轮着灭, 导致灰度误差在相邻权重之间跳变。虽然弯道 KP 已降低到 `LINE_DEFAULT_TURN_KP_SCALE=0.40f`, 但转向输出仍会跟随误差跳变, 在圆弧入口产生左右晃动。

**修改内容**:
- `user/rtos_tasks.c`: 新增 `LINE_TURN_OUTPUT_STEP=3.0f`。原因是弯道模式下限制每个 50ms 控制周期的转向输出变化量, 避免相邻探头交替时转向量突然跳变。
- `user/rtos_tasks.c`: 新增 `g_line_trace_turn_output` 保存上一周期转向输出。原因是需要用上一周期输出计算变化量, 实现弯道转向输出限速。
- `user/rtos_tasks.c`: 新增 `g_line_trace_turn_dir` 记录当前弯道方向。原因是进入弯道后锁定本次弯道的转向方向, 若相邻探头交替造成短暂反向转向量, 先压为 0, 防止小车在弯道内左右反打。
- `user/rtos_tasks.c`: 在 `LineTrace_ResetState()` 中清零弯道方向和上一周期转向输出。原因是每次重新开始循迹时都应重新判断弯道方向, 避免沿用上一段路线的方向记忆。

**仍需实车验证**:
- 若弯道入口仍有轻微晃动, 可将 `LINE_TURN_OUTPUT_STEP` 从 `3.0f` 降到 `2.0f`。
- 若弯道跟随变钝或入弯转不过来, 可将 `LINE_TURN_OUTPUT_STEP` 从 `3.0f` 调到 `4.0f`, 或把 `LINE_DEFAULT_TURN_KP_SCALE` 从 `0.40f` 微调到 `0.45f`。

### 2026-07-30: 循迹速度切换抖动优化

**修改背景**: 用户实车将 `LINE_DEFAULT_TURN_KP_SCALE` 调为 `0.40f` 后, 圆弧甩头明显减轻, 但仍有小幅摆头。进一步现象是直线行驶时中间两路循迹有一路会时亮时灭, 导致 `g_line_trace_turn_mode` 在直线/弯道之间切换, 进而让速度在 `LINE_DEFAULT_STRAIGHT_SPEED=14.0f` 和 `LINE_DEFAULT_TURN_SPEED=10.0f` 之间反复硬切换, 造成抖动。

**修改内容**:
- `user/rtos_tasks.c`: 保留用户已验证的 `LINE_DEFAULT_TURN_KP_SCALE=0.40f`。原因是该值已能明显抑制圆弧甩头, 不再回调。
- `user/rtos_tasks.c`: 新增 `LINE_TURN_IN_CONFIRM=2` 和 `LINE_TURN_OUT_CONFIRM=4`, 并增加 `g_line_trace_turn_in_count/g_line_trace_turn_out_count`。原因是进入/退出弯道必须连续确认, 避免中间循迹单次闪烁就触发直线/弯道模式切换。
- `user/rtos_tasks.c`: 新增 `g_line_trace_speed_filter` 和 `LINE_SPEED_STEP=1.0f`, 将循迹速度从硬切换改为每 50ms 最多变化 1 个速度单位。原因是即使弯道状态发生切换, 速度也从 14 平滑过渡到 10 或从 10 平滑回 14, 减少电机速度突变引起的车身抖动。
- `user/rtos_tasks.c`: 在 `LineTrace_ResetState()` 中同步清零速度滤波和弯道确认计数。原因是每次启动/停止循迹后重新从干净状态开始, 避免沿用上一次的速度和模式判断。

**仍需实车验证**:
- 若直线仍因中间传感器闪烁出现速度抖动, 优先把 `LINE_TURN_IN_CONFIRM` 从 `2` 调到 `3`。
- 若出弯后恢复直行速度太慢, 可把 `LINE_TURN_OUT_CONFIRM` 从 `4` 调到 `3`, 或把 `LINE_SPEED_STEP` 从 `1.0f` 调到 `1.5f`。
- 若圆弧小幅摆头仍存在但速度已不抖, 再微调 `LINE_DEFAULT_TURN_KP_SCALE`, 建议只在 `0.35f~0.45f` 范围内小步调整。

### 2026-07-30: 循迹转弯平滑性优化二次调整

**修改背景**: 上一版平滑方案实车后出现更明显晃动, 且原本稳定的直行也变成左右摇摆。判断上一版同时改动误差低通、输出低通、转向死区和迟滞阈值, 改动范围过大, 其中强滤波和降低 `target_turn` 死区会破坏原本直行时的小误差处理节奏。

**修改内容**:
- `user/rtos_tasks.c`: 恢复 `LINE_DEFAULT_TURN_IN_ERROR=50`, `LINE_DEFAULT_TURN_OUT_ERROR=25`, `LINE_DEFAULT_TURN_DEADBAND=0.0f`, `LINE_DEFAULT_KP=0.12f`。原因是这些参数在原代码下直行稳定, 不再改动直行循迹主响应。
- `user/rtos_tasks.c`: 恢复循迹误差低通为 `旧值*0.2 + 新值*0.8`, 删除上一版新增的 `g_line_trace_turn_filter` 输出低通。原因是实车表现说明额外滤波引入滞后, 车体到线边后修正变晚, 反而导致左右摇摆。
- `user/rtos_tasks.c`: 恢复 `Chassis_LimitTarget()` 中 `target_turn` 死区为 `±9.9f`。原因是原直行稳定依赖该死区抑制小幅噪声修正, 降到 `±3.0f` 后灰度小抖动也会驱动车轮修正。
- `user/rtos_tasks.c`: 保留转弯速度降低为 `LINE_DEFAULT_TURN_SPEED=10.0f`, 并新增 `LINE_DEFAULT_TURN_KP_SCALE=0.75f`, 只在 `g_line_trace_turn_mode` 进入弯道状态时把循迹 KP 降为 75%。原因是尽量不影响直线, 只减弱弯道中的过猛修正。

**仍需实车验证**:
- 若直行已恢复稳定但弯道仍摆头, 优先把 `LINE_DEFAULT_TURN_KP_SCALE` 从 `0.75f` 降到 `0.65f`。
- 若弯道转不过来或压外线, 把 `LINE_DEFAULT_TURN_KP_SCALE` 从 `0.75f` 回调到 `0.85f`, 或将 `LINE_DEFAULT_TURN_SPEED` 从 `10.0f` 调到 `12.0f`。

### 2026-07-30: 循迹转弯平滑性优化

**修改背景**: 小车循迹转弯时会时不时摆头。检查发现循迹误差滤波较轻、转弯速度与直线速度相同、转向 KP 偏激进, 且目标转向小量修正会被较大的死区直接清零, 容易形成“不修正→大修正→过冲→反向修正”的摆头现象。

**修改内容**:
- `user/rtos_tasks.c`: 将 `LINE_DEFAULT_TURN_SPEED` 从 `14.0f` 降为 `10.0f`。原因是转弯时适当降速可减少车体惯性过冲, 让弯道修正更平稳。
- `user/rtos_tasks.c`: 将 `LINE_DEFAULT_KP` 从 `0.12f` 降为 `0.08f`, 并将 `LINE_DEFAULT_TURN_DEADBAND` 从 `0.0f` 调为 `10.0f`。原因是降低循迹转向输出对灰度误差的敏感度, 避免黑线边缘采样抖动直接放大成左右摆头。
- `user/rtos_tasks.c`: 将 `LINE_DEFAULT_TURN_IN_ERROR` 从 `50` 调为 `80`, `LINE_DEFAULT_TURN_OUT_ERROR` 从 `25` 调为 `35`。原因是增大转弯状态切换迟滞, 减少直线/弯道状态在临界误差附近频繁切换。
- `user/rtos_tasks.c`: 将循迹误差低通从 `旧值*0.2 + 新值*0.8` 改为 `旧值*0.65 + 新值*0.35`, 并新增 `g_line_trace_turn_filter` 对最终 `line_turn` 做低通。原因是让目标转向变化更连续, 减少转弯时突然反向修正。
- `user/rtos_tasks.c`: 将 `Chassis_LimitTarget()` 中的 `target_turn` 死区从 `±9.9f` 降为 `±3.0f`。原因是允许小角度连续修正生效, 避免误差积累到较大后才突然修正。

**仍需实车验证**:
- 若转弯仍有摆头, 优先继续降低 `LINE_DEFAULT_KP` 到 `0.06f` 或将 `LINE_DEFAULT_TURN_SPEED` 降到 `8.0f`。
- 若转弯变得太钝、压线外侧过多, 可将 `LINE_DEFAULT_KP` 从 `0.08f` 小幅调回 `0.09f~0.10f`, 或把误差滤波中新值权重从 `0.35f` 提到 `0.45f`。

### 2026-07-30: 12 路灰度改 8 路后的循迹逻辑优化

**修改背景**: 原硬件为 12 路灰度循迹, 现在因硬件问题去掉从 1 开始编号的第 2、4、9、11 路, 实际保留第 1、3、5、6、7、8、10、12 路作为 8 路输入。检查发现 SysConfig 和 GPIO 采样已经是 8 路, 但误差权重仍按 8 路等间距处理, 未体现原 12 路删除后的非等距物理位置。

**修改内容**:
- `user/Sensor.c`: 将灰度误差权重从等距 `{-350,-240,-140,-45,45,140,240,350}` 改为按原 12 路保留位置映射的 `{-550,-350,-150,-50,50,150,350,550}`。原因是删除 2/4/9/11 后, 相邻保留探头间距不再完全一致, 等距权重会压缩外侧偏差并影响弯道判断。
- `user/Sensor.c`: 新增内部函数 `Sensor_CalcErrorFromRaw(uint8_t status)`, 让 `Gray_Task` 使用同一次 `raw` 采样计算 `g_gray_raw_data` 和 `g_gray_error`。原因是原逻辑每轮任务调用 `Sensor_GetRawData()` 和 `Sensor_GetError()` 会读取两次 GPIO, 快速经过黑线边缘时可能出现 raw 与 error 不对应。
- `user/rtos_tasks.c`: 将 `LINE_DEFAULT_STOP_CONFIRM` 从 1 调为 2。原因是 8 路硬件缺少部分通道后, 单次采样触发停止线的抗抖余量偏小, 连续 2 次确认能降低误停概率。

**仍需实车验证**:
- 若实际 OUT1-OUT8 并不是从左到右对应原 12 路的 1、3、5、6、7、8、10、12, 需要按真实安装顺序调整 `Sensor.c` 中的权重数组。
- 修改后最大误差从 ±350 变为 ±550, 当前 `LINE_DEFAULT_KP=0.12f` 会让最外侧单点偏差输出约 ±66 转向量, 未超过 ±100 限幅, 但直道/弯道速度和 KP 仍建议按实车微调。

## 主持修改所需信息

如果需要我主持这个项目的修改, 除上述已分析的内容外, 还需要分析需要什么信息才能修改

要求：更改我的代码时需写修改日志，告诉我为什么原因修改了什么代码。
