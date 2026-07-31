# COM13 VOFA 数据自动抓取与分析脚本执行计划

## Summary

目标是为当前工程增加一个“傻瓜式”的 Python 脚本，用于自动打开电脑端串口 `COM13`，抓取单片机通过 VOFA/串口发来的数据，自动识别并解析当前发送格式，输出可读结果，并保存日志文件，方便检查单片机发送的数据是否正确。

当前代码中已经存在 VOFA 向上位机发送数据的模块：

- VOFA 协议库：`user/vofa.c`、`user/vofa.h`
- 实际周期发送任务：`user/rtos_tasks.c` 中的 `vofa_debug_task`
- 当前实际发送串口：`UART_3_INST`
- 当前实际数据格式：`VOFA_FMT_JUSTFLOAT`
- 当前实际发送内容：3 个 `float`
- 当前波特率：115200
- 当前周期：10 ms，大约 100 Hz

计划实现一个独立 Python 工具，不改动现有嵌入式 C 代码。用户只需要双击/运行一个批处理文件或按提示运行脚本，就能开始采集。

## Current State Analysis

### 1. 已有 VOFA 发送模块

代码库中已有完整 VOFA 模块：

- `user/vofa.h`
  - 定义 VOFA 数据格式：FireWater、JustFloat、RawData。
  - 定义波特率枚举，包含 115200。
  - 声明 `VOFA_Init`、`VOFA_SendData` 等接口。
- `user/vofa.c`
  - `VOFA_Init(...)` 初始化 VOFA 句柄。
  - `VOFA_SendData(...)` 根据格式发送数据。
  - JustFloat 帧尾固定为：`00 00 80 7F`。

### 2. 当前实际 VOFA 输出入口

`user/rtos_tasks.c` 中有 `vofa_debug_task`，在满足以下宏条件时创建：

- `USE_IMU_SENSOR == 1`
- `USE_VOFA_DEBUG == 1`

这两个宏当前在 `user/rtos_tasks.h` 中均为 1，因此 VOFA 调试任务当前应会参与编译和运行。

当前初始化代码等价于：

```c
VOFA_Init(&hvofa, NULL, VOFA_UART_SendCallback,
          VOFA_FMT_JUSTFLOAT, VOFA_BAUD_115200);
```

当前发送回调中实际使用：

```c
DL_UART_Main_transmitDataBlocking(UART_3_INST, data[i]);
```

所以当前 VOFA 数据通过 `UART_3_INST` 发出。

### 3. 当前实际数据内容

当前 `vofa_debug_task` 每 10 ms 发送 3 个 `float`：

```text
通道 0：g_pos_target_pulse - g_pos_current_pulse
通道 1：g_pos_current_pulse
通道 2：target_speed
```

JustFloat 二进制帧格式为：

```text
float0 小端 4 字节
float1 小端 4 字节
float2 小端 4 字节
帧尾 00 00 80 7F
```

因此当前一帧长度应为：

```text
3 * 4 + 4 = 16 字节
```

### 4. 注释与实际代码存在差异

`user/rtos_tasks.c` 中 VOFA 注释提到 9 通道 IMU 数据，但实际代码当前只发送 3 个 float。因此脚本不能只按注释里的 9 通道写死，应默认按当前实际 3 通道解析，同时预留命令行参数允许切换到 9 通道。

### 5. 其他串口发送线索

`user/uart.c` 中还有：

- `VOFA_SendString(char *str)`：通过 `UART_2_INST` 发送字符串。
- `fputc(...)`：将 `printf` 重定向到 `UART_2_INST`。

这说明如果 COM13 实际接的是 UART2，而不是 UART3，Python 可能抓到的是普通文本，而不是 JustFloat 二进制帧。

因此脚本应支持两种模式：

1. 默认模式：JustFloat 二进制解析，适配当前 `vofa_debug_task`。
2. 文本模式：按行读取 ASCII 文本，适配 `printf` 或 FireWater 文本格式。

## Proposed Changes

### 1. 新增 Python 抓取脚本

新增文件：

- `tools/capture_com13_vofa.py`

用途：

- 自动打开 `COM13`。
- 默认使用 `115200, 8N1, timeout=1`。
- 默认按 JustFloat 3 通道解析。
- 自动搜索帧尾 `00 00 80 7F`。
- 解析每帧前 12 字节为 3 个小端 float。
- 在终端实时显示：帧序号、时间戳、通道值、原始帧十六进制。
- 自动统计：有效帧数、错误字节数、解析频率、丢帧/错帧提示。
- 自动保存 CSV 日志，便于用户用 Excel 打开。

建议默认输出 CSV 字段：

```text
time_s,frame_index,ch0_pos_error,ch1_current_pulse,ch2_target_speed,raw_hex
```

命令行参数设计成傻瓜式，但保留可配置能力：

```text
--port COM13               默认 COM13
--baud 115200              默认 115200
--channels 3               默认 3
--mode justfloat           默认 justfloat，可选 text/firewater/raw
--duration 0               默认一直采集，0 表示直到 Ctrl+C
--csv vofa_capture.csv     默认自动生成带时间戳文件名
```

用户最简单用法：

```powershell
python tools/capture_com13_vofa.py
```

如果未来恢复 9 通道：

```powershell
python tools/capture_com13_vofa.py --channels 9
```

如果 COM13 抓到的是文本：

```powershell
python tools/capture_com13_vofa.py --mode text
```

实现要点：

- 使用 `pyserial` 读取串口。
- 使用 `struct.unpack('<fff', payload)` 解析 3 通道 float。
- 使用缓冲区处理半帧、粘包、错位数据。
- 遇到异常时给出中文提示，例如：
  - COM13 打不开：提示检查端口是否被 VOFA/串口助手占用。
  - 没有数据：提示检查单片机是否运行、接线是否为 UART3 TX、GND 是否共地。
  - 数据不像 JustFloat：提示尝试 `--mode text` 或确认当前接的是 UART2/printf。

### 2. 新增 Windows 一键运行脚本

新增文件：

- `tools/run_capture_com13.bat`

用途：

- 面向不懂 Python 的用户。
- 双击后自动检查是否能运行 Python。
- 自动尝试安装/提示安装 `pyserial`。
- 自动运行 `capture_com13_vofa.py`。
- 运行结束后不立刻关闭窗口，方便用户看到错误提示。

批处理逻辑：

```text
1. 切换到脚本所在目录。
2. 检查 python 是否可用。
3. 如果缺少 serial 模块，执行 python -m pip install pyserial。
4. 运行 python capture_com13_vofa.py --port COM13 --baud 115200 --channels 3 --mode justfloat。
5. pause 等待用户查看结果。
```

### 3. 新增简短使用说明

新增文件：

- `tools/COM13_VOFA_使用说明.txt`

用途：

- 用中文说明最少操作步骤。
- 不假设用户懂 Python。
- 内容保持短小，不写复杂教程。

建议内容包括：

```text
1. 确认硬件已经接到电脑，设备管理器里端口是 COM13。
2. 不要同时打开 VOFA/串口助手占用 COM13。
3. 双击 run_capture_com13.bat。
4. 正常情况下会看到 ch0/ch1/ch2 三个数实时变化。
5. CSV 文件会自动保存到 tools/logs 目录。
6. 如果提示端口打不开，关闭其他串口软件后重试。
7. 如果一直没有数据，检查 TX/RX/GND 和单片机程序是否正在运行。
```

### 4. 不修改嵌入式代码

本计划不改动以下文件：

- `user/vofa.c`
- `user/vofa.h`
- `user/rtos_tasks.c`
- `user/rtos_tasks.h`
- `user/uart.c`
- `empty.syscfg`

原因：

- 当前需求是抓取并分析 COM13 数据，不是修改单片机发送逻辑。
- 现有代码已经具备 VOFA 输出能力。
- 先通过 Python 抓包确认实际输出，再决定是否需要后续调整固件。

## Assumptions & Decisions

1. 默认认为 COM13 对应当前 VOFA 输出串口，即 `UART_3_INST` 的 TX。
2. 默认波特率使用 115200，因为 UART1/UART2/UART3/UART_ZDT 配置均为 115200。
3. 默认解析格式使用 JustFloat 3 通道，因为当前 `vofa_debug_task` 实际发送 3 个 float。
4. 脚本要能处理用户不懂 Python 的情况，因此提供 `.bat` 一键运行入口。
5. 如果 COM13 实际接的是 UART2，则脚本可通过 `--mode text` 查看 printf/字符串输出。
6. CSV 日志默认保存，方便后续把数据发给我分析或用 Excel 查看。
7. 不主动安装大型依赖，只使用 Python 标准库和 `pyserial`。
8. 不引入图形界面，避免复杂化；终端实时显示 + CSV 文件足够满足当前检查需求。

## Verification Steps

执行阶段完成后，按以下步骤验证：

### 1. 静态检查

- 检查 `tools/capture_com13_vofa.py` 能被 Python 解释器正常解析。
- 检查命令行参数帮助可用：

```powershell
python tools/capture_com13_vofa.py --help
```

### 2. 依赖检查

运行：

```powershell
python -c "import serial; print(serial.__version__)"
```

如果失败，通过一键脚本或手动执行：

```powershell
python -m pip install pyserial
```

### 3. 串口连接验证

运行默认命令：

```powershell
python tools/capture_com13_vofa.py
```

期望结果：

- 能打开 `COM13`。
- 能持续显示帧计数。
- 能看到 3 个通道数值。
- 有效帧数持续增加。
- CSV 文件被写入。

### 4. 数据格式验证

如果默认模式成功，应看到类似：

```text
frame=1 t=0.010s ch0=... ch1=... ch2=...
frame=2 t=0.020s ch0=... ch1=... ch2=...
```

如果默认模式没有有效帧，执行：

```powershell
python tools/capture_com13_vofa.py --mode text
```

用于判断 COM13 是否接到了 UART2/printf 文本输出。

### 5. 9 通道兼容验证

如果后续固件恢复 IMU 9 通道 JustFloat，运行：

```powershell
python tools/capture_com13_vofa.py --channels 9
```

期望每帧解析 9 个 float。

## 执行顺序

1. 新建 `tools` 目录。
2. 编写 `tools/capture_com13_vofa.py`。
3. 编写 `tools/run_capture_com13.bat`。
4. 编写 `tools/COM13_VOFA_使用说明.txt`。
5. 做 Python 语法检查。
6. 如环境允许，运行 `--help` 检查参数显示。
7. 用户连接硬件后运行一键脚本进行实际采集。

## 最终交付

执行完成后，用户将得到：

- 一个可直接采集 COM13 的 Python 脚本。
- 一个双击运行的 Windows 批处理文件。
- 一个中文傻瓜式使用说明。
- 自动生成的 CSV 日志能力。
- 对当前固件 VOFA JustFloat 3 通道数据格式的实时解析与检查能力。
