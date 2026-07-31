# 第三问视觉闭环编号 1/2 代码修改细化计划

## Summary

本计划细化之前的 `q3_vision_issue_1_2_fix_plan.md`，用于指导后续代码修改。用户已确认本次只修编号 1/2：

1. 修复 `zdt_motor_test_task()` 中 `g_vision_ready_flag` 的双消费问题。
2. 修复两条 `raw_ball_pos_px` 计算公式不一致问题，统一使用带 `Q3_ZERO_BIAS_PX` 的公式。

本次明确不做以下内容：

- 不调 `Q3_KP_PULSE_PER_PX` / `Q3_KD_PULSE_PER_PX`。
- 不调滤波参数。
- 不实现 O → +5cm → -5cm 的完整第三问状态机。
- 不扩展 VOFA 通道；强化诊断计划统一维护在 `.trae/documents/vofa分析脚本编写指导.md`。
- 不调整蓝牙 / VOFA 功能宏。
- 不把视觉接收改成队列。
- 不使用互斥量保护 ISR 共享变量。

当前阶段继续保持 `target_pos_px = 0.0f` 的中心保持目标，并保留现有首帧零位语义：首帧建立 `vision_zero_offset_px` 后，仍继续应用 `Q3_ZERO_BIAS_PX` 参与本周期滤波和控制。

---

## Current State Analysis

### 1. 当前工程结构

主要工程目录：

- `d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)`

本次相关文件：

- `user/rtos_tasks.c`
  - 第三问视觉闭环和 ZDT 摆杆控制集中在 `zdt_motor_test_task()`。
  - 当前需要修改的唯一代码文件。
- `user/uart.c`
  - UART1 视觉接收 ISR。
  - 生产 `g_vision_x_offset` 和 `g_vision_ready_flag`。
  - 本次不修改。
- `user/uart.h`
  - 声明视觉共享变量。
  - 本次不修改。
- `user/rtos_tasks.h`
  - 功能宏和 `RodActuator_Config_t`。
  - 当前 `USE_ZDT_STEPPER=1`、`USE_VOFA_DEBUG=1`。
  - 本次不修改。
- `user/ZDT_X42S.c`
  - ZDT-X42S 协议发送与接收队列。
  - 本次不修改。
- `user/vofa.c` / `user/uart.c`
  - 当前题目 3 文本调试输出通过 `VOFA_SendString()` 走 UART2。
  - 本次不修改。

### 2. 当前视觉输入链路

视觉 ISR 在 `user/uart.c` 中解析 ASCII 有符号整数行：

```c
g_vision_x_offset = (int16_t)value;
g_target_x = (int16_t)value;
g_vision_ready_flag = 1U;
```

当前语义是“单槽 latest value”：

```text
视觉模块最新 x_offset
  -> 覆盖 g_vision_x_offset
  -> 置位 g_vision_ready_flag
```

这不是队列，不保证每一帧都被处理。对当前 50ms 控制周期的中心保持闭环来说，使用最新值比处理旧队列更合适。

### 3. 当前第三问控制链路

```text
UART1 ISR
  -> g_vision_x_offset / g_vision_ready_flag
  -> zdt_motor_test_task()
  -> ball_pos_px / ball_vel_px
  -> pos_error_px = target_pos_px - ball_pos_px
  -> rod_cmd = Kp * pos_error_px - Kd * ball_vel_px
  -> last_target_pulse
  -> RodActuator_SetTargetPulse()
  -> ZDT_MoveAbsolute()
```

当前 `target_pos_px` 固定为：

```c
const float target_pos_px = 0.0f;
```

因此当前代码实现的是“中心保持”，不是完整 O → +5cm → -5cm 状态机。用户已确认本次继续保持中心保持目标。

### 4. 当前编号 1 问题：flag 双消费

当前 `zdt_motor_test_task()` 中有两处周期性消费 `g_vision_ready_flag`。

第一处：循环前半段。

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

第二处：题目 3 分支内部。

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

这会导致：

- 同一轮 50ms 循环内，同一个 ready flag 被两个代码块检查。
- 第一段处理了新帧后，第二段通常看到 flag 已清零。
- 视觉正常时，第二段仍可能把 `vision_lost_count` 加到 1。
- 无视觉帧时，`vision_lost_count` 可能同周期加 2。
- `Q3_VISION_LOST_CYCLES=10` 的语义从“10 个 50ms 周期”变成可能约 5 个周期。

### 5. 当前编号 2 问题：两条公式不一致

第一条公式带 `Q3_ZERO_BIAS_PX`：

```c
raw_ball_pos_px = ((float)g_vision_x_offset - vision_zero_offset_px)
                * Q3_VISION_POS_SIGN
                + Q3_ZERO_BIAS_PX;
```

第二条公式不带 `Q3_ZERO_BIAS_PX`：

```c
float raw_ball_pos_px = ((float)g_vision_x_offset - vision_zero_offset_px)
                      * Q3_VISION_POS_SIGN;
```

当前 `Q3_ZERO_BIAS_PX`：

```c
#define Q3_ZERO_BIAS_PX 17.0f
```

如果 UART1 ISR 在第一段消费之后、第二段检查之前到来，新帧会走第二条公式，导致本周期坐标少 17px 软件零位补偿。位置跳变会进一步传导到 `ball_vel_px` 和 `rod_cmd`。

---

## Proposed Changes

### 文件：`user/rtos_tasks.c`

本次只修改 `zdt_motor_test_task()` 内部逻辑，不修改其他模块。

---

### Change 1：在每个 50ms 循环建立视觉快照

#### What

在 `zdt_motor_test_task()` 的 `while (1)` 循环中，每轮新增局部变量：

```c
bool has_new_vision = false;
int16_t vision_x_snapshot = 0;
```

在清空 ZDT 反馈队列之后，加入一个极短临界区：

```c
taskENTER_CRITICAL();
if (g_vision_ready_flag != 0U) {
    vision_x_snapshot = g_vision_x_offset;
    g_vision_ready_flag = 0U;
    has_new_vision = true;
}
taskEXIT_CRITICAL();
```

#### Why

- `g_vision_ready_flag` 是 ISR 生产的单槽事件标志，只应有一个消费入口。
- 任务侧用快照可以保证本周期使用同一个 `vision_x_snapshot`。
- 临界区避免“读取 offset 与清 flag 之间被 ISR 插入”的不一致。
- 不使用互斥量，因为普通 mutex 不适合 ISR 与任务共享变量保护。
- 不使用队列，因为中心保持闭环更需要最新位置，队列可能引入旧帧延迟。

#### How

- 临界区只允许包含：检查 flag、读取 offset、清 flag、设置本地 bool。
- 临界区内不得调用：`snprintf()`、`VOFA_SendString()`、`RodActuator_SetTargetPulse()`、`ZDT_*()`、任何阻塞函数。
- 后续视觉处理全部使用 `vision_x_snapshot`。

#### 对其他模块影响

| 模块 | 影响 |
|---|---|
| UART1 ISR | 不修改 ISR。仍然写 `g_vision_x_offset` 并置 flag。 |
| OLED 显示 | 不影响，OLED 仍可读取原始 `g_vision_x_offset` 显示。 |
| 蓝牙 | 不影响。 |
| 循迹 | 不影响。 |
| ZDT 协议 | 不影响。 |

#### 可能新问题

- 临界区会短暂屏蔽中断。由于只读写两个变量，耗时极短，风险低。
- 若错误地把耗时逻辑放入临界区，会影响系统中断响应。执行时必须避免。

---

### Change 2：统一视觉位置/速度更新入口

#### What

把当前第一段视觉处理改为唯一处理入口，逻辑使用：

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

#### Why

- 统一坐标转换公式。
- 保证所有视觉帧都经过 `Q3_ZERO_BIAS_PX`。
- 保证 `ball_pos_px`、`ball_vel_px` 只有一个更新入口。
- 保证 `vision_lost_count` 每 50ms 循环最多递增一次。
- 保留现有首帧零位逻辑，符合用户选择。

#### How

- 替换当前直接使用 `g_vision_x_offset` 的视觉处理逻辑。
- 不调整现有参数值。
- 不改变 `target_pos_px=0.0f`。
- 不改变 `rod_cmd` 计算。

#### 对其他模块影响

| 模块 | 影响 |
|---|---|
| 题目 3 | 正向影响：控制输入路径唯一，丢帧计数语义恢复。 |
| 题目 2/4/5/6 | 不影响，未触碰循迹和底盘控制。 |
| VOFA 文本输出 | 间接变好，输出的 `ball_pos_px` 来自唯一处理路径。 |
| ZDT 摆杆 | 输入更连续，但原参数可能需要后续重新调。 |

#### 可能新问题

- 修复后视觉丢失回中时间可能从约 250ms 恢复到设计约 500ms。如果希望更快回中，应后续调小 `Q3_VISION_LOST_CYCLES`，不要保留双递增 bug。
- 修复后控制表现可能和之前不同，因为之前参数可能是在错误数据路径下调出来的。

---

### Change 3：删除题目 3 分支内第二套视觉消费逻辑

#### What

删除题目 3 分支中的第二个：

```c
if (g_vision_ready_flag != 0U) {
    ...
} else if (vision_lost_count < UINT16_MAX) {
    vision_lost_count++;
}
```

题目 3 分支只保留：

```c
if (vision_lost_count >= Q3_VISION_LOST_CYCLES) {
    ball_vel_px = 0.0f;
    last_target_pulse = 0;
    RodActuator_ReturnCenter();
} else {
    float pos_error_px = target_pos_px - ball_pos_px;
    float rod_cmd = Q3_KP_PULSE_PER_PX * pos_error_px - Q3_KD_PULSE_PER_PX * ball_vel_px;
    ...
    last_target_pulse = (int32_t)rod_cmd;
    RodActuator_SetTargetPulse(last_target_pulse);
}
```

#### Why

- 彻底消除双消费。
- 彻底消除第二条不带 `Q3_ZERO_BIAS_PX` 的公式。
- 让题目 3 分支只使用已经处理好的状态量，而不是再次访问 ISR flag。

#### How

- 删除第二段视觉更新代码时，保留 `contest_started` 初始化逻辑。
- 保留丢帧回中逻辑。
- 保留 PD 控制和限幅逻辑。

#### 对其他模块影响

| 模块 | 影响 |
|---|---|
| 题目 3 | 控制分支更清晰，只消费状态量。 |
| UART1 | 不影响。 |
| 其他题目 | 不影响。 |

#### 可能新问题

- 如果此前第二段偶然消费到新帧，修复后这些新帧会等到下一轮循环被快照消费。这是 expected behavior，因为每 50ms 控制周期只处理一次最新视觉状态。

---

### Change 4：退出题目 3 时清理 flag 使用临界区

#### What

当前退出题目 3 时有：

```c
g_vision_ready_flag = 0U;
```

建议改为：

```c
taskENTER_CRITICAL();
g_vision_ready_flag = 0U;
taskEXIT_CRITICAL();
```

#### Why

- 退出题目 3 时丢弃 pending 视觉帧是现有行为，保留。
- 用临界区让清 flag 的语义更明确。

#### How

- 只包裹 `g_vision_ready_flag = 0U;`。
- 不在临界区内调用回中、秒表、VOFA、ZDT 等函数。

#### 对其他模块影响

- 基本无影响。
- 只影响题目 3 退出瞬间的 pending 视觉帧。

#### 可能新问题

- 如果进入/退出题目 3 的瞬间有新视觉帧，可能被丢弃。这与当前行为一致，不是新增行为。

---

### Change 5：保留当前 VOFA 文本输出

#### What

保留当前：

```c
snprintf(vofa_buf, sizeof(vofa_buf), "%ld,%ld,%ld\n",
         (long)ball_pos_px,
         (long)target_pos_px,
         (long)last_target_pulse);
VOFA_SendString(vofa_buf);
```

#### Why

- 本次只修编号 1/2，不引入调试通道变更。
- 修复后 `ball_pos_px` 更可信，现有三列输出已经能辅助判断是否仍有突跳。

#### 对其他模块影响

- 无新增影响。
- UART2 阻塞发送风险保持原状。

---

## Assumptions & Decisions

1. 用户已确认：本次只修编号 1/2。
2. 用户已确认：题目 3 当前阶段继续中心保持，`target_pos_px=0.0f` 不变。
3. 用户已确认：首帧零位逻辑保留现状，即首帧建立 `vision_zero_offset_px` 后仍应用 `Q3_ZERO_BIAS_PX`。
4. 使用 `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()` 做任务侧视觉快照。
5. 不使用互斥量保护视觉变量，因为普通 mutex 不适合