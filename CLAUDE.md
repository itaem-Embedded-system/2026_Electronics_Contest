# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

2026 年全国大学生电子设计竞赛 H 题《车载平衡滚球运动控制系统》— 基于 MSPM0G3507 (Cortex-M0+, 80MHz, 128KB Flash, 32KB SRAM) 的差速驱动小车主控工程，运行 FreeRTOS v11.2.0 多任务系统。

**职责边界**：本工程负责底盘循迹、编码器测速、电机 PID、题目菜单/按键、OLED 秒表、IMU 姿态、闭环步进电机驱动、滚球位置闭环。**不负责**无线图传、录像回放、摄像头图像识别（由独立摄像模块完成，本工程仅通过 UART1 接收钢球偏移值）。

## 文档索引

| 文档 | 内容 |
|------|------|
| `empty_LP.../CLAUDE.md` | **工程详细手册**：完整启动流程、架构设计、外设映射、数据流、已知 Bug、500+ 行修改日志 |
| `本工程目的以及相关限制.md` | 项目目标、6 道题目拆解、职责边界、合规风险清单 |
| `.trae/documents/vofa分析脚本编写指导.md` | VOFA 调试字段定义、分析脚本规范、诊断方法 |
| `.trae/documents/q3_first_round_cycle_vofa_zdt_plan.md` | 题目 3 控制周期优化计划（待实施） |
| `26电赛问题解答2607302112.txt` | 官方 H 题 FAQ（66 条），含循迹传感器、图传、无线通信等合规要求 |

## Repository Structure

```
D:\26diansaidisanti\
├── empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)/   ← CCS Theia 工程 (主工作区)
│   ├── empty.c / empty.syscfg                       ← 程序入口 + SysConfig 外设配置
│   ├── user/                                         ← 所有用户驱动和业务逻辑
│   ├── Freertos/                                     ← FreeRTOS Kernel 源码
│   ├── Debug/                                        ← 构建输出 (gitignored)
│   ├── targetConfigs/                                ← 调试探针配置 (XDS110)
│   ├── tools/                                        ← Python 辅助脚本
│   └── CLAUDE.md                                     ← ★ 工程详细手册 (500+ 行)
├── 本工程目的以及相关限制.md                          ← 项目目标、题目拆解、合规风险
├── H题_车载平衡滚球运动控制系统.pdf                   ← 赛题 PDF
└── .trae/documents/                                   ← 技术文档
```

## Build System

- **IDE**: TI Code Composer Studio Theia (CCS 20.x) — **编译和烧录必须通过 CCS Theia GUI 操作**，无独立命令行构建脚本
- **Compiler**: TI Arm Clang 5.1.1.LTS, `thumbv6m`, `cortex-m0plus`
- **SDK**: MSPM0-SDK v2.10.00.04, **SysConfig**: v1.27.0+
- **Debug Probe**: XDS110 via SWD (PA19/SWDIO, PA20/SWCLK)
- Build constraints (Cortex-M0+ 无硬件浮点、malloc 不可用、堆 25KB 等) 详见工程 CLAUDE.md

## FreeRTOS Tasks

| Task | Pri | Stack | Function |
|------|-----|-------|----------|
| IMU_Task | 4 | 768w | ⚠️ 已禁用 (USE_IMU_SENSOR=0) |
| BT_Task | 3 | 128w | ⚠️ 已禁用 (USE_BLUETOOTH=0) |
| Ctrl_Task | 2 | 512w | 50ms 底盘主控制：速度环+转向环并联 PID → PWM |
| KeyScan | 2 | 128w | 10ms 按键扫描 + OLED 题目菜单 (S2/S3/S4) |
| GrayTask | 2 | 128w | 10ms 八路灰度采样 + 停止线锁存 |
| ZDT_Test | 2 | 512w | 题目 3 视觉闭环摆杆控制（VOFA 已禁用: USE_VOFA_DEBUG=0） |
| Buzzer | 2 | 128w | 蜂鸣器提示 |
| CPU_Monitor | 1 | 256w | 每秒 CPU/堆栈统计 |
| OLED_Task | 1 | 768w | OLED 显示刷新 20ms |
| Heartbeat | 1 | 128w | LED 500ms 翻转 |

功能宏定义在 `user/rtos_tasks.h`：`USE_IMU_SENSOR=0`, `USE_BLUETOOTH=0`, `USE_ULTRASONIC=0`, `USE_VOFA_DEBUG=0`, `USE_GRAY_SENSOR=1`, `USE_OLED_DISPLAY=1`, `USE_ZDT_STEPPER=1`。

## Coding Conventions

- 修改代码时必须写修改日志（原因 + 修改内容 + 仍需实车验证），参考工程 CLAUDE.md 修改日志部分的格式
- 浮点运算在 M0+ 上为软件模拟，避免在中断或高频循环中使用 `sqrtf`/`sinf`
- PID 参数调整建议小步进行（如 KP 变化 ≤0.05），每次调整后需实车验证
- 循迹参数分题目独立配置，修改某题参数不得影响其他题目
- 正式测试期间蓝牙 (HC-05) 必须禁用或确保不参与控制

## 当前待完善项

1. 题目 3 O→+5cm→-5cm 自动目标序列（当前仅中心 0px 稳定）
2. 题目 4/5 行驶中滚球稳定（需结合小车速度/加速度/转弯前馈补偿）
3. 题目 6 评委指定位置输入与保持逻辑
4. ✅ 已修复：题目 3 视觉数据消费结构（唯一快照入口 + 统一公式，2026-08-01）
5. 实车标定：摄像模块偏移值符号/比例、A 点停车测点、循迹传感器合规确认
