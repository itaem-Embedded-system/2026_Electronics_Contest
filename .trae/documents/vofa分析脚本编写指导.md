# VOFA 文档与分析脚本编写指导

## 1. 文档定位

这是当前工程中 VOFA 相关内容的统一维护文档，用于说明：

- 当前固件实际通过哪个串口发送 VOFA/调试数据。
- 当前题目 3 输出了哪些字段。
- 分析脚本应该如何解析、保存和辅助判断问题。
- `VOFA_SendString()` 与 `VOFA_SendData()` 的区别。

当前结论：**题目 3 调试输出走 UART2 文本 CSV，不走旧 UART3 JustFloat 调试任务。**

---

## 2. 当前 VOFA 发送模块现状

当前工程里有两类 VOFA 相关发送能力。

### 2.1 当前实际使用：`VOFA_SendString()`

位置：`user/uart.c`

```c
void VOFA_SendString(char *str) {
    while (*str != '\0') {
        DL_UART_Main_transmitDataBlocking(UART_2_INST, *str);
        str++;
    }
}
```

特性：

- 通过 `UART_2_INST` 阻塞逐字节发送。
- 适合发送 CSV / FireWater 风格文本行。
- 当前题目 3 调试输出实际使用这一路。
- 当前物理连接是 UART2 接 VOFA/串口工具，因此分析脚本默认文本解析。

### 2.2 通用库保留：`VOFA_SendData()`

位置：`user/vofa.c` / `user/vofa.h`

`VOFA_SendData()` 是通用 VOFA 打包库，支持：

| 格式 | 说明 | 当前题目 3 是否使用 |
|---|---|---|
| FireWater | 文本 CSV 行，形如 `v0,v1,v2\n` | 否，题目 3 直接用 `VOFA_SendString()` |
| JustFloat | 多个 float 小端二进制 + 帧尾 `00 00 80 7F` | 否，旧 UART3 调试任务已移除 |
| RawData | 按字段类型发送原始字节 | 否 |

当前不需要修改 `vofa.c` / `vofa.h`。

---

## 3. 当前题目 3 实际输出

当前题目 3 输出位置在 `user/rtos_tasks.c` 的 `zdt_motor_test_task()` 中。

当前已经升级为 9 字段，并且只在题目 3 运行时输出，每 2 个 50ms 控制周期输出 1 次，即约 10Hz。

当前固件输出代码等价于：

```c
#if USE_VOFA_DEBUG
        if (question3_running) {
            vofa_debug_div++;
            if (vofa_debug_div >= 2U) {
                vofa_debug_div = 0U;
                snprintf(vofa_buf, sizeof(vofa_buf),
                         "%lu,%d,%ld,%ld,%ld,%ld,%ld,%u,%u\n",
                         (unsigned long)g_rx_pulse,
                         (int)g_vision_x_offset,
                         (long)ball_pos_px,
                         (long)target_pos_px,
                         (long)ball_vel_px,
                         (long)last_target_pulse,
                         (long)RodActuator_GetTargetPulse(),
                         (unsigned int)vision_lost_count,
                         (unsigned int)question3_running);
                VOFA_SendString(vofa_buf);
            }
        } else {
            vofa_debug_div = 0U;
        }
#endif
```

当前 CSV 格式：

```text
rx,raw,ball,target,vel,cmd,rod,lost,run
```

字段含义：

| 字段 | 来源 | 诊断用途 |
|---|---|---|
| `rx` | `g_rx_pulse` | 判断 UART1 视觉串口是否持续收到数据 |
| `raw` | `g_vision_x_offset` | 判断视觉原始方向、零位、跳变 |
| `ball` | `ball_pos_px` | 判断滤波后的钢珠位置 |
| `target` | `target_pos_px` | 判断目标位置 |
| `vel` | `ball_vel_px` | 判断速度估计噪声和 Kd 是否放大噪声 |
| `cmd` | `last_target_pulse` | 控制律输出 |
| `rod` | `RodActuator_GetTargetPulse()` | 限幅/限速后的实际摆杆目标 |
| `lost` | `vision_lost_count` | 判断视觉丢帧/更新不稳定 |
| `run` | `question3_running` | 判断当前是否处于题目 3 运行状态 |

设计约束：

- 不修改 `VOFA_SendString()`。
- 不修改 `vofa.c` / `vofa.h`。
- 不修改 UART1 视觉接收 ISR。
- 不修改 ZDT 底层协议。
- 输出只在题目 3 运行时发送。
- 默认约 10Hz，降低 UART2 阻塞影响。

---

## 4. 如何通过 9 字段诊断问题

| 问题 | 主要观察字段 | 判断方法 |
|---|---|---|
| 控制方向错误 | `raw,ball,target,cmd,rod` | `ball` 偏离 `target` 时，`cmd/rod` 应让球回中心；若越控越远，优先检查 `Q3_VISION_POS_SIGN` 或 `Q3_KP_PULSE_PER_PX` 符号 |
| Kp/Kd 不匹配 | `ball,target,vel,cmd` | 接近目标时 `cmd` 仍很大，多为 Kp 过大/Kd 不足；`vel` 抖动带动 `cmd` 抖动，说明 Kd 放大噪声 |
| 视觉/滤波延迟 | `raw,ball,cmd,rod` | `raw` 先变，`ball` 晚几帧跟上，说明滤波/视觉延迟；`cmd/rod` 再滞后则执行链路也慢 |
| 速度估计噪声 | `ball,vel,cmd` | `ball` 小抖但 `vel` 大跳，且 `cmd` 高频反打，说明速度估计噪声进入 Kd 项 |
| 机械死区/非线性 | `cmd,rod,ball` | `cmd/rod` 已变化但 `ball` 长时间不动，随后突然冲出，可能是死区、摩擦或间隙 |
| 输出限幅/限速 | `cmd,rod` | `cmd` 很大但 `rod` 每次只变一段，说明受 `ROD_DEFAULT_MAX_STEP` 限制；`cmd` 长期卡 ±上限说明饱和 |
| 零位偏置不准 | `raw,ball,target,cmd` | 球在机械中心但 `ball` 不接近 `target`，且 `cmd` 长期非 0，需要检查 `Q3_ZERO_BIAS_PX` 和首帧零位 |
| 视觉消费异常 | `rx,raw,ball,lost` | `rx` 增长但 `ball` 更新不稳、`lost` 偶尔上升，说明视觉接收/消费节奏需要整理 |

---

## 5. 分析脚本维护要求

当前脚本：`empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)/tools/analyze_vofa_text.py`

脚本字段表必须与固件 CSV 严格对齐：

```python
FIELD_NAMES = ["rx", "raw", "ball", "target", "vel", "cmd", "rod", "lost", "run"]
```

CSV 保存字段：

```text
timestamp,time_s,line_index,rx,raw,ball,target,vel,cmd,rod,lost,run,raw_text
```

解析策略：

1. 打开串口。
2. 按行读取。
3. 去掉 `\r\n`。
4. 英文逗号分割。
5. 检查列数等于 `len(FIELD_NAMES)`。
6. 转成数值。
7. 添加电脑端时间戳。
8. 写入 CSV。
9. 可选实时打印。

异常提示要求：

- 串口打不开：检查端口号和 VOFA/串口助手占用。
- 10 秒无数据：检查 UART2 接线、单片机是否运行、题目 3 是否启动、`USE_VOFA_DEBUG` 是否为 1。
- 字段数量不匹配：提示固件字段和 `FIELD_NAMES` 不一致。
- 数值转换失败：保留原始行并继续采集。

---

## 6. 当前不建议默认使用 JustFloat

当前工程已经移除旧 UART3 JustFloat 调试任务，题目 3 实际输出是 UART2 文本 CSV。

如果脚本默认按 JustFloat 解析，会出现：

- 找不到帧尾 `00 00 80 7F`。
- 有效帧数为 0。
- 文本字节被当作二进制垃圾丢弃。

因此当前分析脚本默认必须是 `text/csv` 模式。JustFloat 解析只作为以后重新启用 `VOFA_SendData(..., VOFA_FMT_JUSTFLOAT, ...)` 时的兼容分支。

---

## 7. 文档维护规则

后续只维护这一份 VOFA 主文档。若出现新的 VOFA 抓包/分析计划，应优先合并到本文档对应章节，而不是新开重复文档。

需要同步维护的位置：

| 位置 | 维护要求 |
|---|---|
| 本文档 | 记录当前实际发送入口、字段、脚本设计和诊断方法 |
| `tools/analyze_vofa_text.py` | 字段表、顶部注释、实时打印格式必须与固件 CSV 对齐 |
| `CLAUDE.md` | 只保留高层说明，避免重复大量字段细节 |
| `user/rtos_tasks.c` | 注释只说明输出目的、字段顺序和节流策略，不写过时 UART3/JustFloat 内容 |

不再维护旧 UART3 JustFloat 抓包文档；如需恢复 JustFloat，应在本文档新增“JustFloat 恢复方案”章节。