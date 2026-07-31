# VOFA 分析脚本编写指导

## 目标

编写一个电脑端串口分析脚本，用于采集和分析当前固件发给 VOFA/串口工具的数据。

当前工程里有两类 VOFA 相关发送能力：

1. `user/uart.c` 中的 `VOFA_SendString()`：直接通过 `UART_2_INST` 发送字符串。
2. `user/vofa.c` 中的 `VOFA_SendData()`：通用 VOFA 打包库，支持 FireWater、JustFloat、RawData 三种格式。

当前实际题目 3 调试输出使用第 1 类，也就是 `VOFA_SendString()` 走 UART2 文本 CSV。

## 当前实际发送入口

当前题目 3 输出位置在 `user/rtos_tasks.c` 的 `zdt_motor_test_task()` 中：

```c
#if USE_VOFA_DEBUG
        snprintf(vofa_buf, sizeof(vofa_buf), "%ld,%ld,%ld\n",
                 (long)ball_pos_px,
                 (long)target_pos_px,
                 (long)last_target_pulse);
        VOFA_SendString(vofa_buf);
#endif
```

底层发送函数在 `user/uart.c`：

```c
void VOFA_SendString(char *str) {
    while (*str != '\0') {
        DL_UART_Main_transmitDataBlocking(UART_2_INST, *str);
        str++;
    }
}
```

因此当前分析脚本应默认按 UART2 文本行解析，而不是默认按 JustFloat 二进制解析。

## 当前 CSV 字段

当前每行 3 个整数：

```text
ball,target,cmd
```

含义：

| 字段 | 来源 | 含义 |
|---|---|---|
| `ball` | `ball_pos_px` | 题目 3 处理后的球位置 |
| `target` | `target_pos_px` | 题目 3 目标位置，当前通常为 0 |
| `cmd` | `last_target_pulse` | 本周期计算出的摆杆目标脉冲 |

示例：

```text
-12,0,30
-10,0,25
0,0,0
```

## 脚本默认设计

### 1. 串口参数

默认参数建议：

```text
port = COM13
baud = 115200
mode = text
encoding = ascii / utf-8 ignore
line ending = \n
```

脚本应允许命令行覆盖：

```text
--port COM13
--baud 115200
--csv logs/q3_vofa.csv
--duration 0
```

### 2. 文本解析逻辑

推荐流程：

1. 打开串口。
2. 按行读取，直到遇到 `\n`。
3. 去掉 `\r\n` 和空白字符。
4. 跳过空行。
5. 用英文逗号分割。
6. 当前期望 3 列。
7. 每列转为整数或浮点。
8. 增加电脑端时间戳和行号。
9. 实时打印并写入 CSV。

CSV 保存字段建议：

```text
time_s,line_index,ball,target,cmd,raw_text
```

### 3. 异常处理

脚本需要处理这些情况：

- 串口打不开：提示检查端口号、VOFA/串口助手是否占用。
- 长时间无数据：提示检查 UART2 TX/RX/GND、单片机是否运行、题目 3 是否启动、`USE_VOFA_DEBUG` 是否为 1。
- 行格式错误：保留 `raw_text`，计入 bad line，不让脚本退出。
- 列数不是 3：提示当前固件字段可能已经扩展，需要同步更新字段表。
- 数值转换失败：保存原始行，继续采集下一行。

## 推荐 Python 结构

脚本可以按以下结构写：

```python
import argparse
import csv
import time
from pathlib import Path

import serial

FIELD_NAMES = ["ball", "target", "cmd"]


def parse_line(line):
    text = line.strip()
    if not text:
        return None
    parts = text.split(",")
    if len(parts) != len(FIELD_NAMES):
        raise ValueError(f"字段数量不匹配: {text}")
    return [float(item) for item in parts]
```

主循环建议：

```python
start = time.time()
line_index = 0
bad_lines = 0

while duration <= 0 or time.time() - start < duration:
    raw = ser.readline()
    if not raw:
        continue

    text = raw.decode("utf-8", errors="ignore").strip()
    now = time.time() - start

    try:
        values = parse_line(text)
        if values is None:
            continue
        line_index += 1
        row = [now, line_index, *values, text]
        writer.writerow(row)
        print(f"#{line_index} t={now:.3f}s ball={values[0]} target={values[1]} cmd={values[2]}")
    except ValueError as exc:
        bad_lines += 1
        print(f"坏行 {bad_lines}: {text} ({exc})")
```

## 如果固件扩展为 9 字段

如果后续题目 3 输出扩展为：

```text
rx,raw,ball,target,vel,cmd,rod,lost,run
```

只需要把脚本字段表改为：

```python
FIELD_NAMES = ["rx", "raw", "ball", "target", "vel", "cmd", "rod", "lost", "run"]
```

并同步更新打印格式即可。解析框架不需要改变。

## VOFA_SendData 通用库兼容说明

虽然当前实际输出是 UART2 文本，但 `user/vofa.c` 仍保留通用打包函数 `VOFA_SendData()`。

### FireWater

`VOFA_FMT_FIREFIREWATER` 发送格式是文本行：

```text
v0,v1,v2,...\n
```

脚本可以复用当前 text/csv 解析逻辑。

### JustFloat

`VOFA_FMT_JUSTFLOAT` 发送格式是：

```text
N 个 float 小端二进制 + 帧尾 00 00 80 7F
```

只有当固件重新创建 VOFA 句柄、绑定发送回调，并实际调用 `VOFA_SendData(..., VOFA_FMT_JUSTFLOAT, ...)` 时，脚本才需要启用 JustFloat 解析。

### RawData

`VOFA_FMT_RAWDATA` 是原始字节发送，脚本必须知道每个字段的数据类型和顺序，否则无法可靠解析。

## 当前不建议默认使用 JustFloat 的原因

当前工程已经移除旧 UART3 JustFloat 调试任务，题目 3 实际输出是 UART2 文本 CSV。如果脚本默认按 JustFloat 解析，会表现为：

- 找不到帧尾 `00 00 80 7F`
- 有效帧数为 0
- 缓冲区不断丢弃文本字节

所以当前脚本默认模式必须是文本行解析。

## 验证清单

1. 串口能打开。
2. 题目 3 启动后能看到连续文本行。
3. 每行能解析出 3 个字段。
4. CSV 文件能保存：`time_s,line_index,ball,target,cmd,raw_text`。
5. `ball` 随视觉输入变化。
6. `target` 当前通常为 0。
7. `cmd` 随摆杆控制输出变化。
8. 如果无数据，先检查题目 3 是否运行和 `USE_VOFA_DEBUG` 是否开启。

## 文件命名建议

建议脚本命名为：

```text
tools/analyze_vofa_text.py
```

日志目录建议：

```text
tools/logs/
```

默认日志文件名建议带时间戳：

```text
q3_vofa_YYYYMMDD_HHMMSS.csv
```
