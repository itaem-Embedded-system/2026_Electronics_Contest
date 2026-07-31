# 第三问视觉闭环编号 1/2 问题修复计划

## Summary

本计划用于修复第三问视觉闭环中的两个已确认问题：

1. `zdt_motor_test_task()` 中 `g_vision_ready_flag` 在同一个 50ms 控制周期内存在两处消费点，导致题目 3 分支的新帧判断失真，并可能让 `vision_lost_count` 在同周期递增两次。
2. 两条 `raw_ball_pos_px` 计算路径不一致：第一条路径包含 `Q3_ZERO_BIAS_PX`，第二条路径漏掉该零位补偿，导致偶发控制坐标跳变。

计划的修复目标是：在 `user/rtos_tasks.c` 中将视觉数据处理收敛为单一入口，确保每个控制周期最多消费一次视觉帧、最多递增一次丢帧计数，并统一使用包含 `Q3_ZERO_BIAS_PX` 的坐标变换公式。修复范围不包含 Kp/Kd 参数调优、速度按 dt 归一化、VOFA 通道扩展或 UART 复用调整。

---

## Current State Analysis

### 1. 相关代码结构

当前工程主目录：

- `d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)`

第三问视觉闭环相关文件：

- `user/rtos_tasks.c`
  - 第三问控制任务 `zdt_motor_test_task()` 位于约 `1334-1488` 行。
  - 题目 3 运行判定：`g_question_ui_state == 1U && g_selected_question == 3U`。
  - 摆杆控制输出通过 `RodActuator_SetTargetPulse()` 下发给 ZDT-X42S。
- `user/uart.c`
  - `UART_1_INST_IRQHandler()` 解析视觉模块发送的 ASCII 有符号整数行。
  - 成功解析后写入 `g_vision_x_offset`，并置位 `g_vision_ready_flag = 1U`。
- `user/rtos_tasks.h`
  - 当前 `USE_ZDT_STEPPER = 1`，`USE_VOFA_DEBUG = 1`，第三问任务会创建。
- `user/ZDT_X42S.c`
  - `ZDT_MoveAbsolute()` 负责发送 ZDT 绝对位置运动指令。
- `user/vofa.c` / `user/uart.c`
  - 当前第三问文本调试输出通过 `VOFA_SendString()` 走 `UART_2_INST`。

### 2. 当前视觉输入数据流

```text
UART1 视觉模块
  -> UART_1_INST_IRQHandler()
  -> g_vision_x_offset = value
  -> g_vision_ready_flag = 1U
  -> zdt_motor_test_task()
  -> ball_pos_px / ball_vel_px
  -> rod_cmd
  -> last_target_pulse
  -> RodActuator_SetTargetPulse()
  -> ZDT_MoveAbsolute()
```

### 3. 当前问题 1：`g_vision_ready_flag` 双消费

在 `user/rtos_tasks.c` 的 `zdt_motor_test_task()` 中，当前存在两段视觉数据消费逻辑：

第一段位于循环前半段，约 `1375-1407` 行：

```c
if (g_vision_ready_flag != 0U) {
    ...
    g_vision_ready_flag = 0U;
    ...
    vision_lost_count = 0;
} else if (vision_lost_count < UINT16_MAX) {
    vision_lost_count++;
}
```

第二段位于题目 3 分支内，约 `1435-1458` 行：

```c
if (g_vision_ready_flag != 0U) {
    ...
    g_vision_ready_flag = 0U;
    ...
    vision_lost_count = 0;
} else if (vision_lost_count < UINT16_MAX) {
    vision_lost_count++;
}
```

当前影响：

- 如果视觉帧在本轮循环开始前到达，会被第一段先消费并清零。
- 题目 3 分支内通常看不到同周期新帧。
- 如果本轮没有新帧，`vision_lost_count` 可能在第一段和第二段各递增一次。
- 如果第一段刚处理新帧，第二段可能又因为 flag 已清零把 `vision_lost_count` 加到 1。
- 视觉正常时可能仍然出现丢帧计数异常，影响 `Q3_VISION_LOST_CYCLES` 判断。

### 4. 当前问题 2：零位补偿计算路径不一致

第一段公式约在 `user/rtos_tasks.c:1388`：

```c
raw_ball_pos_px = ((float)g_vision_x_offset - vision_zero_offset_px) * Q3_VISION_POS_SIGN + Q3_ZERO_BIAS_PX;
```

第二段公式约在 `user/rtos_tasks.c:1436`：

```c
float raw_ball_pos_px = ((float)g_vision_x_offset - vision_zero_offset_px) * Q3_VISION_POS_SIGN;
```

当前 `Q3_ZERO_BIAS_PX` 定义在约 `user/rtos_tasks.c:177`：

```c
#define Q3_ZERO_BIAS_PX 17.0f
```

当前影响：

- 两条路径对同一个视觉输入使用不同坐标定义。
- 若 UART ISR 在第一段消费之后、第二段检查之前到达新帧，第二段会漏掉 `Q3_ZERO_BIAS_PX`。
- `ball_pos_px` 与 `ball_vel_px` 可能出现固定偏置跳变。
- 控制输出 `rod_cmd` 可能突然变化，现场表现类似过冲、反打或中心保持不稳。

### 5. 影响程度评估

| 维度 | 等级 | 说明 |
|---|---|---|
| 业务影响 | 高 | 第三问中心保持闭环可能无法稳定完成。 |
| 用户影响 | 高 | 现场表现为球无法稳定在中心、偶发过冲、误回中、调参不可复现。 |
| 调试影响 | 高 | VOFA/串口采集到的数据可能无法准确反映单一控制路径。 |
| 影响范围 | 中 | 主要影响题目 3；循迹题目 2/4/5/6 基本不受影响。 |
| 修复优先级 | 高 | 应优先于 Kp/Kd 参数调节。 |

---

## Proposed Changes

### 文件 1：`user/rtos_tasks.c`

#### Change 1：在 `zdt_motor_test_task()` 中建立单一视觉帧快照入口

**位置：** `zdt_motor_test_task()` 的 `while (1)` 循环内，建议放在 ZDT 反馈队列清空之后、`if (!question3_running)` 判断之前。

**What：**

引入局部变量：

```c
bool has_new_vision = false;
int16_t vision_x_snapshot = 0;
```

每个 50ms 周期只在一个位置读取并清零 `g_vision_ready_flag`：

```c
taskENTER_CRITICAL();
if (g_vision_ready_flag != 0U) {
    vision_x_snapshot = g_vision_x_offset;
    g_vision_ready_flag = 0U;
    has_new_vision = true;
}
taskEXIT_CRITICAL();
```

**Why：**

- 保证 ISR 生产的视觉事件每周期只有一个消费入口。
- 避免第一段和题目 3 分支重复清零同一个 flag。
- 避免读取 `g_vision_x_offset` 与清零 flag 之间被 ISR 插入造成当前周期数据不一致。

**How：**

- 使用 FreeRTOS 临界区宏保护 `g_vision_x_offset` 和 `g_vision_ready_flag` 的快照读取。
- 临界区只包含读取 16 位 offset、清 8 位 flag、设置本地 bool，保持极短。
- 后续所有视觉处理只使用 `vision_x_snapshot`，不再直接读取 `g_vision_x_offset`。

#### Change 2：将视觉位置与速度更新收敛成唯一代码块

**位置：** `zdt_motor_test_task()` 中原第一段视觉处理位置，即约 `1375-1407` 行。

**What：**

将原逻辑改为：

```c
if (has_new_vision) {
    float raw_ball_pos_px;
    float new_ball_pos_px;

    if (!vision_zero_ready) {
        vision_zero_offset_px = (float)vision_x_snapshot;
        vision_zero_ready = true;
        ball_pos_px = 0.0f;
        last_ball_pos_px = 0.0f;
        ball_vel_px = 0.0f;
    }

    raw_ball_pos_px = ((float)vision_x_snapshot - vision_zero_offset_px)
                    * Q3_VISION_POS_SIGN
                    + Q3_ZERO_BIAS_PX;

    if (fabsf(raw_ball_pos_px) <= Q3_ZERO_DEADBAND_PX) {
        raw_ball_pos_px = 0.0f;
    }

    new_ball_pos_px = ball_pos_px * (1.0f - Q3_POS_FILTER_ALPHA)
                    + raw_ball_pos_px * Q3_POS_FILTER_ALPHA;

    if (fabsf(new_ball_pos_px) <= Q3_ZERO_DEADBAND_PX) {
        new_ball_pos_px = 0.0f;
    }

    ball_vel_px = ball_vel_px * (1.0f - Q3_VEL_FILTER_ALPHA)
                + (new_ball_pos_px - last_ball_pos_px) * Q3_VEL_FILTER_ALPHA;
    last_ball_pos_px = new_ball_pos_px;
    ball_pos_px = new_ball_pos_px;
    vision_lost_count = 0;
} else if (vision_lost_count < UINT16_MAX) {
    vision_lost_count++;
}
```

**Why：**

- 统一 `raw_ball_pos_px` 坐标定义。
- 保证 `Q3_ZERO_BIAS_PX` 永远生效。
- 保证 `ball_pos_px`、`ball_vel_px` 只有一个更新入口。
- 保证 `vision_lost_count` 每周期最多递增一次。

**How：**

- 使用 `vision_x_snapshot` 替代所有视觉处理中的 `g_vision_x_offset`。
- 保留现有死区、低通、速度估计、丢帧计数策略。
- 不修改 `Q3_KP_PULSE_PER_PX`、`Q3_KD_PULSE_PER_PX` 和滤波参数。

#### Change 3：删除题目 3 分支内第二套视觉消费逻辑

**位置：** `user/rtos_tasks.c` 约 `1435-1458` 行。

**What：**

删除题目 3 分支内部的：

```c
if (g_vision_ready_flag != 0U) {
    ...
} else if (vision_lost_count < UINT16_MAX) {
    vision_lost_count++;
}
```

题目 3 分支只保留：

- 首次进入题目 3 时初始化 `contest_started`、`ball_pos_px`、`ball_vel_px`、`last_target_pulse`、`vision_lost_count`。
- 根据统一视觉处理后的 `vision_lost_count` 判断是否回中。
- 根据统一视觉处理后的 `ball_pos_px` 和 `ball_vel_px` 计算 `rod_cmd`。

**Why：**

- 消除双消费根因。
- 消除两套公式不一致根因。
- 让题目 3 控制分支只消费“处理后的状态”，而不是再次消费 ISR flag。

**How：**

- 保留 `if (vision_lost_count >= Q3_VISION_LOST_CYCLES)` 分支。
- 保留 `rod_cmd` 计算和限幅。
- 保留 `RodActuator_SetTargetPulse(last_target_pulse)` 调用。

#### Change 4：退出题目 3 时对 pending 视觉帧的处理保持简单

**位置：** `user/rtos_tasks.c` 约 `1409-1423` 行。

**What：**

保留退出题目 3 时清理状态：

```c
contest_started = false;
vision_zero_offset_px = 0.0f;
ball_pos_px = 0.0f;
last_ball_pos_px = 0.0f;
ball_vel_px = 0.0f;
last_target_pulse = 0;
vision_lost_count = 0;
```

对于 `g_vision_ready_flag = 0U` 的清理，执行时可保留，但建议也放入临界区：

```c
taskENTER_CRITICAL();
g_vision_ready_flag = 0U;
taskEXIT_CRITICAL();
```

**Why：**

- 退出题目 3 后丢弃切换瞬间旧帧是可接受的。
- 但清 flag 应与 ISR 并发关系保持明确。

**How：**

- 只调整清 flag 的并发保护，不改变退出行为。

#### Change 5：保留现有 VOFA 文本输出，不扩展通道

**位置：** `user/rtos_tasks.c` 约 `1478-1484` 行。

**What：**

暂时保留当前输出：

```c
snprintf(vofa_buf, sizeof(vofa_buf), "%ld,%ld,%ld\n",
         (long)ball_pos_px,
         (long)target_pos_px,
         (long)last_target_pulse);
VOFA_SendString(vofa_buf);
```

**Why：**

- 本计划只修编号 1/2，避免把数据通道扩展、串口冲突和控制修复混在一次变更里。
- 修复后当前三通道数据已经能反映统一视觉路径下的位置、目标和输出。

**How：**

- 不修改 `VOFA_SendString()`。
- 不扩展题目 3 UART2 CSV 字段；VOFA 强化诊断计划统一维护在 `.trae/documents/vofa分析脚本编写指导.md`。
- 不调整 `USE_VOFA_DEBUG`、`USE_BLUETOOTH`、`USE_IMU_SENSOR`。

### 文件 2：`user/uart.c`

不修改。

**原因：**

- 当前 `UART_1_INST_IRQHandler()` 能正确解析 ASCII 有符号整数行。
- ISR 生产者模式保留：只负责写 `g_vision_x_offset` 并置 `g_vision_ready_flag`。
- 本次修复只调整任务侧消费逻辑。

### 文件 3：`user/rtos_tasks.h`

不修改。

**原因：**

- 当前 `USE_ZDT_STEPPER = 1`，第三问任务启用。
- 当前 `USE_VOFA_DEBUG = 1`，第三问文本输出启用。
- 本次不处理 VOFA 强化诊断字段扩展，也不调整功能宏；VOFA 相关计划统一维护在 `.trae/documents/vofa分析脚本编写指导.md`。

### 文件 4：`user/ZDT_X42S.c`

不修改。

**原因：**

- 当前问题根因在视觉数据消费路径，不在 ZDT 运动协议层。
- `ZDT_MoveAbsolute()` 和 `RodActuator_SetTargetPulse()` 现有调用关系保持不变。

---

## Assumptions & Decisions

1. 本次只修复编号 1 和编号 2。
2. 不修改控制律：

```c
rod_cmd = Q3_KP_PULSE_PER_PX * pos_error_px - Q3_KD_PULSE_PER_PX * ball_vel_px;
```

3. 不调整参数：

- `Q3_KP_PULSE_PER_PX`
- `Q3_KD_PULSE_PER_PX`
- `Q3_POS_FILTER_ALPHA`
- `Q3_VEL_FILTER_ALPHA`
- `Q3_ZERO_BIAS_PX`
- `Q3_ZERO_DEADBAND_PX`

4. 不实现 +5cm / -5cm 往返脚本。当前 `target_pos_px` 继续保持 `0.0f`，即零位保持。
5. 不修改 UART1 ISR 协议。
6. 不扩展 VOFA 通道。
7. 不处理速度按 dt 归一化问题；该问题留到视觉路径可信后再处理。
8. 不处理 VOFA 强化诊断字段扩展；该计划统一维护在 `.trae/documents/vofa分析脚本编写指导.md`。
9. 使用 `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()` 做视觉快照，临界区必须极短。
10. 修复后若现场表现发生变化，应视为数据路径被修正后的正常结果，需要重新基于可信数据调参，而不是回退到双消费结构。

---

## Implementation Steps

### Step 1：准备与基线确认

**负责人：** 嵌入式开发负责人。

**操作：**

1. 确认 Git 工作区干净。
2. 建议创建修复分支：`fix/q3-vision-consume-path`。
3. 记录当前异常现象：中心保持不稳、类似超调、是否出现误回中。
4. 记录当前关键参数值：`Q3_ZERO_BIAS_PX`、`Q3_KP_PULSE_PER_PX`、`Q3_KD_PULSE_PER_PX`、`Q3_POS_FILTER_ALPHA`、`Q3_VEL_FILTER_ALPHA`。

**交付标准：**

- 有可回滚的 Git 基线。
- 明确本次只修编号 1/2。
- 修复前异常现象有记录。

### Step 2：修改 `zdt_motor_test_task()` 的视觉快照逻辑

**负责人：** 嵌入式开发负责人。

**操作：**

1. 在 `while (1)` 循环内新增 `has_new_vision` 与 `vision_x_snapshot`。
2. 在 ZDT 反馈队列清理之后，使用临界区读取并清零 `g_vision_ready_flag`。
3. 后续本周期只使用 `vision_x_snapshot`。

**交付标准：**

- `g_vision_ready_flag` 在该循环周期内只有一个消费点。
- 不再有两个独立代码块直接读取并清零 `g_vision_ready_flag`。

### Step 3：合并视觉位置/速度计算路径

**负责人：** 嵌入式开发负责人。

**操作：**

1. 保留第一段视觉处理位置作为唯一处理入口。
2. 将 `g_vision_x_offset` 替换为 `vision_x_snapshot`。
3. 统一使用公式：

```c
raw_ball_pos_px = ((float)vision_x_snapshot - vision_zero_offset_px)
                * Q3_VISION_POS_SIGN
                + Q3_ZERO_BIAS_PX;
```

4. 保留死区处理、位置低通、速度低通。
5. 无新帧时每周期最多执行一次 `vision_lost_count++`。

**交付标准：**

- `Q3_ZERO_BIAS_PX` 只在唯一视觉坐标转换路径中应用。
- `ball_pos_px` 和 `ball_vel_px` 只在唯一视觉处理入口更新。
- `vision_lost_count` 每个 50ms 周期最多变化一次。

### Step 4：删除题目 3 分支内重复视觉处理

**负责人：** 嵌入式开发负责人。

**操作：**

1. 删除题目 3 分支内第二个 `if (g_vision_ready_flag != 0U)` 块。
2. 删除第二条不含 `Q3_ZERO_BIAS_PX` 的 `raw_ball_pos_px` 计算。
3. 保留视觉丢失后的回中逻辑。
4. 保留 `rod_cmd` 计算、限幅和 `RodActuator_SetTargetPulse()`。

**交付标准：**

- `zdt_motor_test_task()` 中搜索 `g_vision_ready_flag`，应只剩：
  - 单一快照消费入口。
  - 退出题目 3 时的清理入口。
- 不再存在不含 `Q3_ZERO_BIAS_PX` 的第二条 `raw_ball_pos_px` 公式。

### Step 5：代码审查与静态检查

**负责人：** 代码审查负责人。

**操作：**

1. 检查 `g_vision_ready_flag` 是否仍有重复消费。
2. 检查 `raw_ball_pos_px` 是否只有一条公式。
3. 检查临界区是否只包裹必要变量读取与 flag 清零。
4. 检查题目 3 退出路径是否仍能清理状态并回中。

**交付标准：**

- 无重复消费。
- 无重复公式。
- 无额外功能变更。
- 无不必要参数调整。

### Step 6：编译与集成调试

**负责人：** 嵌入式开发负责人 + 硬件验证负责人。

**操作：**

1. 使用当前 CCS/TI Clang 工程编译。
2. 烧录到板子。
3. 进入题目 3。
4. 验证视觉正常时不会误回中。
5. 遮挡视觉，验证约 `Q3_VISION_LOST_CYCLES * 50ms` 后回中。
6. 观察摆杆输出是否仍有由数据路径导致的突变。

**交付标准：**

- 工程编译通过。
- 题目 3 能正常启动。
- 视觉正常时不误判丢失。
- 遮挡视觉后按设计周期回中。
- 修复未影响题目菜单和 ZDT 初始化。

### Step 7：预发布验证

**负责人：** 测试验证负责人。

**操作：**

1. 连续运行题目 3 多轮。
2. 每轮记录 `ball_pos_px`、`target_pos_px`、`last_target_pulse`。
3. 观察是否还有突发偏置跳变。
4. 验证题目 2/4/5/6 的启动和停止不受影响。

**交付标准：**

- 视觉正常时误回中次数为 0。
- `ball_pos_px` 无由双路径造成的固定偏置跳变。
- `last_target_pulse` 无由视觉路径切换造成的反向突跳。
- 循迹相关题目维持原有正常表现。

---

## Resource Plan

### 人力资源

| 角色 | 职责 |
|---|---|
| 嵌入式开发负责人 | 修改 `user/rtos_tasks.c`，确保视觉消费路径唯一。 |
| 硬件验证负责人 | 烧录、上板、检查视觉/ZDT/摆杆运行状态。 |
| 测试记录负责人 | 记录修复前后题目 3 表现和串口数据。 |
| 代码审查负责人 | 检查是否仍存在重复消费或公式分叉。 |

### 技术工具

| 工具 | 用途 |
|---|---|
| Git | 分支、提交、回滚。 |
| CCS / TI Clang | 编译和烧录工程。 |
| 串口助手或 VOFA+ | 观察 `VOFA_SendString()` 输出。 |
| Python 采集脚本 | 后续采集串口数据用于离线分析。 |
| 逻辑分析仪/示波器 | 可选，用于确认 UART1 视觉帧率。 |

### 测试环境

| 环境 | 要求 |
|---|---|
| 桌面固定测试 | 小车固定，摆杆可安全动作。 |
| 视觉输入环境 | 光照稳定，视觉模块持续输出 `x_offset\r\n`。 |
| 题目 3 实测环境 | 允许球在摆杆上自由运动，具备安全防护。 |
| 回归测试环境 | 可切换题目 2/4/5/6 验证循迹不受影响。 |

---

## Risk Management

### 风险 1：修复后视觉丢失回中变慢

**原因：** 原来 `vision_lost_count` 可能每周期加两次，修复后恢复为每周期最多加一次。

**影响：** 实际回中时间可能从约 250ms 变为设计值约 500ms。

**预案：** 如果确实需要更快回中，后续单独降低 `Q3_VISION_LOST_CYCLES`，不恢复双递增 bug。

**回滚：** Git 回退本次修复提交。

### 风险 2：修复后现有 Kp/Kd 表现变化

**原因：** 控制器输入路径被修正，之前参数可能是在错误数据路径下调出来的。

**影响：** 球的响应可能变得更真实但与修复前不同。

**预案：** 修复后先采集数据，再重新调参数。

**回滚：** 回退本次修复提交，恢复旧固件作对比。

### 风险 3：临界区使用不当影响 UART

**原因：** 任务侧短暂关闭中断读取视觉快照。

**影响：** 若临界区过长，可能影响 UART 中断响应。

**预案：** 临界区仅包含 `g_vision_ready_flag` 判断、`g_vision_x_offset` 读取、flag 清零、本地 bool 设置。

**回滚：** 改为不使用临界区，或后续引入更正式的队列/双缓冲。

### 风险 4：仍然存在过冲

**原因：** 编号 1/2 只修数据路径，不解决所有控制问题。仍可能存在方向、Kp/Kd、机械惯性、视觉延迟、速度 dt 等问题。

**影响：** 修复后球仍可能过冲。

**预案：** 本次修复完成后，基于可信数据继续处理编号 3、VOFA 扩展、参数调节和机械调校。

**回滚：** 不建议因仍有过冲而回滚编号 1/2；除非确认修复引入新故障。

---

## Verification Steps

### 1. 静态验证

- 搜索 `zdt_motor_test_task()` 中的 `g_vision_ready_flag`。
- 验收标准：
  - 只有一个周期性消费入口。
  - 退出题目 3 时允许有一次清理入口。
- 搜索 `raw_ball_pos_px`。
- 验收标准：
  - 不存在第二条漏掉 `Q3_ZERO_BIAS_PX` 的公式。

### 2. 编译验证

- 使用 CCS/TI Clang 编译当前工程。
- 验收标准：
  - 无编译错误。
  - 无因作用域、变量未使用、宏条件导致的问题。

### 3. 功能验证

#### 用例 1：视觉正常输入

**步骤：**

1. 启动题目 3。
2. 保持视觉模块持续输出 `x_offset`。
3. 观察调试输出和摆杆动作。

**验收：**

- 不应出现视觉正常时误回中。
- `ball_pos_px` 连续变化。
- `last_target_pulse` 不应因路径切换出现突跳。

#### 用例 2：视觉遮挡

**步骤：**

1. 启动题目 3。
2. 遮挡视觉输入。
3. 观察摆杆是否回中。

**验收：**

- 大约 `Q3_VISION_LOST_CYCLES * 50ms` 后回中。
- 当前参数下约为 500ms。
- 不应提前一半时间误触发。

#### 用例 3：零位一致性

**步骤：**

1. 球放置在机械中心附近。
2. 观察 `ball_pos_px`。
3. 轻微移动球，观察位置变化。

**验收：**

- `ball_pos_px` 不出现由公式切换导致的固定偏置跳变。
- 零位补偿始终一致生效。

#### 用例 4：题目切换

**步骤：**

1. 从菜单进入题目 3。
2. 按 S2 返回菜单。
3. 再进入题目 3。

**验收：**

- 退出时摆杆回中。
- 再次进入时状态重新初始化。
- 不沿用上一次残留 `vision_zero_offset_px`、`ball_pos_px`、`ball_vel_px`。

### 4. 回归验证

| 题目 | 验证内容 | 验收标准 |
|---|---|---|
| 题目 2 | 循迹启动、停止 | 正常。 |
| 题目 4 | 位置环循迹 | 正常。 |
| 题目 5/6 | 平滑循迹启动 | 正常。 |
| OLED 菜单 | S2/S3/S4 选择题目 | 正常。 |
| ZDT 初始化 | 上电后回中、可接收位置命令 | 正常。 |

### 5. 量化验收指标

| 指标 | 目标 |
|---|---|
| 视觉正常时误回中 | 0 次。 |
| 无视觉时 `vision_lost_count` 递增 | 每 50ms 最多 +1。 |
| 零位补偿一致性 | 所有视觉帧统一经过 `Q3_ZERO_BIAS_PX`。 |
| 控制输出突跳 | 无由双路径/公式不一致造成的突跳。 |
| 循迹回归 | 题目 2/4/5/6 无行为退化。 |

---

## Deployment Plan

### 阶段 1：本地代码修复

**交付物：** 修改后的 `user/rtos_tasks.c`。

**通过标准：** 静态检查确认视觉消费入口唯一。

### 阶段 2：编译烧录

**交付物：** 可烧录固件。

**通过标准：** 编译通过并成功烧录。

### 阶段 3：桌面固定测试

**交付物：** 题目 3 静态运行记录。

**通过标准：** 视觉正常时不误回中，遮挡时按设计回中。

### 阶段 4：实车题目 3 测试

**交付物：** 第三问实际运行数据。

**通过标准：** 数据路径稳定，若仍有过冲，可进入下一轮基于 VOFA 的控制参数分析。

### 阶段 5：回归与版本固化

**交付物：** Git 提交与可回滚版本。

**通过标准：** 题目 2/4/5/6 回归正常，第三问修复无新增故障。

### 灰度策略

嵌入式项目采用功能灰度：

1. 只运行题目 3，不运行其他题目。
2. 运行题目 3 + 串口数据采集。
3. 回归题目 2/4/5/6。
4. 固化为比赛前候选固件。

### 上线后监控指标

- `ball_pos_px` 是否连续。
- `last_target_pulse` 是否连续。
- 视觉正常时是否误回中。
- 遮挡视觉时是否按设计回中。
- 球是否仍出现明显超调。
- 摆杆是否出现高频反打。

---

## Retrospective Requirements

修复完成后需要进行一次复盘，重点回答：

1. 为什么 `g_vision_ready_flag` 会出现两个消费点？
2. 为什么两段视觉位置公式没有保持一致？
3. 是否缺少“ISR flag 只能有一个消费者”的代码约定？
4. 是否需要将视觉处理封装成独立静态函数，避免后续再次复制逻辑？
5. 后续调参前，是否已确认数据路径可信？
6. 是否需要建立题目 3 的固定 VOFA 通道规范？

### 预防机制

1. 对 ISR 标志位建立规则：一个 flag 只能有一个任务消费入口。
2. 对传感器坐标变换建立规则：`raw -> zero offset -> sign -> bias -> deadband -> filter` 必须集中在一处实现。
3. 调参前必须先验证数据链路：帧率、坐标方向、零位补偿、丢帧计数、输出连续性。
4. 后续如扩展 +5cm / -5cm 脚本，应基于统一的 `ball_pos_px` 和 `ball_vel_px`，不要重新读取 `g_vision_x_offset`。
5. 每次控制逻辑修复后单独提交，避免和参数调节混在同一提交。

---

## Executor Checklist

执行修复时按以下清单逐项完成：

- [ ] 在 `zdt_motor_test_task()` 中新增单一视觉快照入口。
- [ ] 用 `vision_x_snapshot` 替代视觉处理中的直接 `g_vision_x_offset` 读取。
- [ ] 保留且统一使用 `+ Q3_ZERO_BIAS_PX`。
- [ ] 删除题目 3 分支内第二套 `g_vision_ready_flag` 消费逻辑。
- [ ] 确认 `vision_lost_count` 每周期最多递增一次。
- [ ] 保留现有 `rod_cmd` 控制律与参数。
- [ ] 保留现有 VOFA 文本输出。
- [ ] 编译通过。
- [ ] 上板验证视觉正常不误回中。
- [ ] 上板验证遮挡视觉按设计回中。
- [ ] 回归题目 2/4/5/6。
