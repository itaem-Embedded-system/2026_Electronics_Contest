# 第三问视觉闭环编号 1/2 细化代码修改计划

## Summary

本计划是在既有 `.trae/documents/q3_vision_issue_1_2_fix_plan.md` 基础上的执行级细化，目标是先修复第三问视觉闭环中的两个数据路径问题：

1. `zdt_motor_test_task()` 中 `g_vision_ready_flag` 被同一个 50ms 控制周期内两处逻辑消费。
2. `raw_ball_pos_px` 存在两条计算路径，其中一条漏掉 `Q3_ZERO_BIAS_PX`，导致视觉坐标定义不一致。

用户已确认本次编号 1/2 修复范围：

* 只修编号 1/2。
* 题目 3 当前阶段继续做中心保持，`target_pos_px = 0.0f` 不变。
* 首帧零位保留现逻辑：首帧建立 `vision_zero_offset_px` 后，同一帧仍继续应用 `Q3_ZERO_BIAS_PX`。
* 不顺带做 Kp/Kd 调参、速度 dt 归一化、VOFA 通道扩展、UART 复用处理、+5cm/-5cm 状态机。

最新日志结论：`tools/logs/q3_vofa_20260801_051613.csv` 显示当前现象已不是简单小振荡，而是“测量跳变 + 输出饱和 + 执行滞后”的组合问题：`raw/ball` 多次大幅跳变，`cmd` 大量撞到当前 `ROD_DEFAULT_MIN/MAX_PULSE = ±180`，实际 `rod` 又受 `ROD_DEFAULT_MAX_STEP = 45` 的每 50ms 限步进约束而滞后追赶。

因此本计划分成两个层级：

1. **本次代码修复层级**：仍先修编号 1/2，让 `ball_pos_px`、`ball_vel_px`、`vision_lost_count` 的数据路径可信。
2. **修复后复测层级**：最高优先级调整为：第一，确认控制方向是否反了；第二，将输出限幅从当前 ±180 临时降到 ±60 或 ±80 后重测；之后再判断 D 项符号、Kp/Kd、滤波和视觉测量跳变处理。

本次计划的代码修改应集中在：

* `d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.c`

不修改：

* `user/uart.c`
* `user/rtos_tasks.h`
* `user/ZDT_X42S.c`
* `user/vofa.c`

---

## Current State Analysis

### 1. 当前视觉输入链路

视觉模块由 UART1 输入 ASCII 有符号整数行，逻辑在：

* `d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\uart.c`

关键行为：

```c
g_vision_x_offset = (int16_t)value;
g_target_x = (int16_t)value;
g_vision_ready_flag = 1U;
```

该 ISR 是唯一视觉数据生产者。它只负责写最新 `x_offset` 和置位 ready flag，不负责滤波、零位补偿或控制输出。

### 2. 当前第三问控制链路

第三问控制任务是 `zdt_motor_test_task()`，位于：

* `d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.c`

核心数据流：

```text
UART1 ISR
  -> g_vision_x_offset / g_vision_ready_flag
  -> zdt_motor_test_task()
  -> ball_pos_px / ball_vel_px
  -> rod_cmd
  -> last_target_pulse
  -> RodActuator_SetTargetPulse()
  -> ZDT_MoveAbsolute()
```

题目 3 运行判定：

```c
bool question3_running = (g_question_ui_state == 1U && g_selected_question == 3U);
```

当前目标位置：

```c
const float target_pos_px = 0.0f;
```

本次编号 1/2 修复不修改该目标，继续做中心保持。

### 3. 当前摆杆输出边界与执行滞后来源

当前摆杆默认参数在 `user/rtos_tasks.c` 中定义：

```c
#define ROD_DEFAULT_MIN_PULSE          (-180)
#define ROD_DEFAULT_MAX_PULSE          (180)
#define ROD_DEFAULT_MAX_STEP           45
#define ROD_DEFAULT_RPM                220
#define ROD_DEFAULT_ACC                50
```

当前控制律：

```c
float pos_error_px = target_pos_px - ball_pos_px;
float rod_cmd = Q3_KP_PULSE_PER_PX * pos_error_px - Q3_KD_PULSE_PER_PX * ball_vel_px;

if (rod_cmd > (float)ROD_DEFAULT_MAX_PULSE) {
    rod_cmd = (float)ROD_DEFAULT_MAX_PULSE;
} else if (rod_cmd < (float)ROD_DEFAULT_MIN_PULSE) {
    rod_cmd = (float)ROD_DEFAULT_MIN_PULSE;
}

last_target_pulse = (int32_t)rod_cmd;
RodActuator_SetTargetPulse(last_target_pulse);
```

`cmd` 与 `rod` 的层级不同：

* `cmd`：`last_target_pulse`，来自控制律并已受 `±180` 控制律限幅。
* `rod`：`RodActuator_GetTargetPulse()`，是 `RodActuator_SetTargetPulse()` 内部保存的实际目标，还会再经过绝对限幅和 `ROD_DEFAULT_MAX_STEP = 45` 的每周期限步进。

因此日志中 `cmd` 长期 ±180 表示控制律饱和；`rod` 阶梯追赶 `cmd` 表示执行器限步进导致的命令跟随滞后。

### 4. 当前问题 1：ready flag 双消费

当前 `zdt_motor_test_task()` 中存在两个周期性消费点。

第一处：循环前半段，当前代码约在 `rtos_tasks.c:1361-1393`：

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

第二处：题目 3 分支内，当前代码约在 `rtos_tasks.c:1421-1444`：

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

影响：

* 视觉帧可能在第一段被清掉，题目 3 分支内看不到同周期新帧。
* 无视觉帧时，`vision_lost_count` 可能一个 50ms 周期加两次。
* 有视觉帧时，第一段清零后，第二段可能又把 `vision_lost_count` 从 0 加到 1。
* `Q3_VISION_LOST_CYCLES = 10U` 的 500ms 语义可能被压缩到约 250ms。
* 最新日志中视觉仍在更新但 `lost` 几乎长期为 1，与该双消费路径吻合。

### 5. 当前问题 2：视觉位置公式不一致

第一条公式当前包含 `Q3_ZERO_BIAS_PX`：

```c
raw_ball_pos_px = ((float)g_vision_x_offset - vision_zero_offset_px) * Q3_VISION_POS_SIGN + Q3_ZERO_BIAS_PX;
```

第二条公式当前缺少 `Q3_ZERO_BIAS_PX`：

```c
float raw_ball_pos_px = ((float)g_vision_x_offset - vision_zero_offset_px) * Q3_VISION_POS_SIGN;
```

当前 `Q3_ZERO_BIAS_PX`：

```c
#define Q3_ZERO_BIAS_PX                17.0f
```

用户已确认保留现有零位语义，因此统一公式必须使用带 bias 的版本：

```c
raw_ball_pos_px = ((float)vision_x_snapshot - vision_zero_offset_px)
                * Q3_VISION_POS_SIGN
                + Q3_ZERO_BIAS_PX;
```

### 6. 最新 VOFA 日志现象：测量跳变、输出饱和与执行滞后

已读取最新日志：

* `d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\tools\logs\q3_vofa_20260801_051613.csv`

日志字段为：

```text
timestamp,time_s,line_index,rx,raw,ball,target,vel,cmd,rod,lost,run,raw_text
```

其中当前源码实际发送的 9 个核心字段为：

```text
rx, raw, ball, target, vel, cmd, rod, lost, run
```

关键现象：

* `target` 始终为 0，说明日志仍对应当前中心保持阶段。
* `raw/ball` 多次出现大幅跳变，不像简单连续物理运动，说明视觉测量链路存在跳变、限幅或识别异常风险。
* `cmd` 大量达到 `±180`，说明闭环输出长期撞到当前摆杆目标限幅。
* `rod` 与 `cmd` 经常不同，并以阶梯方式追赶 `cmd`，这与 `ROD_DEFAULT_MAX_STEP = 45` 的每 50ms 限步进一致，说明执行器存在明显命令跟随滞后。

因此当前问题不能再简单归类为小振荡，而是：

```text
测量跳变 + 输出饱和 + 执行滞后
```

该日志结论对计划的影响：

* 编号 1/2 仍必须先修，因为 ready flag 双消费和 bias 不一致会影响 `ball_pos_px/ball_vel_px/lost` 可信度。
* 但只修编号 1/2 不应被期待为立即消除大幅振荡。
* 修复编号 1/2 后，复测优先级调整为：
  1. 先确认控制方向是否反了，包括视觉方向、Kp 符号、摆杆正负方向与球运动方向。
  2. 将 `ROD_DEFAULT_MIN_PULSE/ROD_DEFAULT_MAX_PULSE` 从当前 `±180` 临时降到 `±60` 或 `±80`，再采集同格式日志。
  3. 如果限幅降低后仍出现同向越控越远，再优先排查方向符号。
  4. 如果方向正确但仍过冲，再进入 D 项符号、滤波、Kp/Kd 调参和视觉测量跳变处理。
* 临时降低输出限幅属于后续现场验证/调试动作，不纳入本次编号 1/2 数据路径修复提交，除非另开调试提交。

### 7. 当前工程结构适配性

当前结构适合做小范围修复，原因：

* `g_vision_ready_flag` 的生产者集中在 `uart.c`。
* 控制消费者集中在 `rtos_tasks.c` 的 `zdt_motor_test_task()`。
* ZDT 执行器已经通过 `RodActuator_SetTargetPulse()` 封装。
* 题目菜单、循迹、位置环不直接依赖 `g_vision_ready_flag`。
* 当前任务周期固定 50ms，适合将 `vision_lost_count` 定义为“每控制周期最多 +1”。

---

## Proposed Changes

### 文件：`user/rtos_tasks.c`

#### Change 1：在 `zdt_motor_test_task()` 中建立唯一视觉快照入口

**修改位置：**

`zdt_motor_test_task()` 的 `while (1)` 内，放在 ZDT 反馈队列清空之后、原第一段 `if (g_vision_ready_flag != 0U)` 之前。

目标结构：

```c
while (1) {
    bool question3_running = ...;
    bool has_new_vision = false;
    int16_t vision_x_snapshot = 0;

    if (Motor1.rxReady == true) {
        ...
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (g_vision_ready_flag != 0U) {
        vision_x_snapshot = g_vision_x_offset;
        g_vision_ready_flag = 0U;
        has_new_vision = true;
    }
    if (!primask) {
        __enable_irq();
    }

    if (has_new_vision) {
        ...
    }
```

**为什么采用 PRIMASK 风格：**

当前工程业务代码中已有多处 `__get_PRIMASK()` / `__disable_irq()` / 条件恢复中断的短临界区风格，例如编码器读取、ZDT 队列读取、蓝牙命令快照。因此实际执行时建议沿用 PRIMASK 风格，降低风格差异和嵌套中断状态恢复风险。

**临界区限制：**

临界区内只允许：

* 判断 `g_vision_ready_flag`。
* 读取 `g_vision_x_offset`。
* 清零 `g_vision_ready_flag`。
* 设置本地 `has_new_vision`。

临界区内禁止：

* 浮点计算。
* `fabsf()`。
* `snprintf()`。
* `VOFA_SendString()`。
* ZDT 发送。
* 任何可能阻塞或耗时函数。

#### Change 2：把原第一段视觉处理改成唯一位置/速度更新入口

**目标逻辑：**

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

**必须保持的语义：**

* 首帧如果 `vision_zero_ready == false`，先建立 `vision_zero_offset_px = vision_x_snapshot`。
* 首帧同一周期仍继续执行统一公式。
* 首帧仍应用 `+ Q3_ZERO_BIAS_PX`。
* 不在首帧建立零位后 `continue` 或跳过控制坐标计算。

#### Change 3：删除题目 3 分支内第二套视觉处理

**删除范围：**

题目 3 分支内的第二段：

```c
if (g_vision_ready_flag != 0U) {
    float raw_ball_pos_px = ((float)g_vision_x_offset - vision_zero_offset_px) * Q3_VISION_POS_SIGN;
    ...
    g_vision_ready_flag = 0U;
    ...
} else if (vision_lost_count < UINT16_MAX) {
    vision_lost_count++;
}
```

**删除后保留：**

题目 3 分支内仍保留：

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

题目 3 控制分支应只使用统一处理后的状态：

* `ball_pos_px`
* `ball_vel_px`
* `vision_lost_count`

不应再次读取：

* `g_vision_ready_flag`
* `g_vision_x_offset`
* 第二套 `raw_ball_pos_px` 公式

#### Change 4：退出题目 3 时清理 pending flag 保持兜底，但加短临界保护

**当前逻辑：**

非题目 3 分支中，若 `contest_started` 为 true，会清理状态并执行：

```c
g_vision_ready_flag = 0U;
```

**目标逻辑：**

保留该兜底清理，但建议改成：

```c
uint32_t primask = __get_PRIMASK();
__disable_irq();
g_vision_ready_flag = 0U;
if (!primask) {
    __enable_irq();
}
```

该清理入口不算周期性消费入口。修复后允许存在：

* 一个周期性快照消费入口。
* 一个退出题目 3 时的清理入口。

#### Change 5：本次编号 1/2 修复保留中心目标、控制律和参数

本次编号 1/2 修复必须不改：

```c
const float target_pos_px = 0.0f;
```

必须不改：

* `ROD_DEFAULT_MIN_PULSE`
* `ROD_DEFAULT_MAX_PULSE`
* `ROD_DEFAULT_MAX_STEP`
* `Q3_ZERO_BIAS_PX`
* `Q3_VISION_POS_SIGN`
* `Q3_KP_PULSE_PER_PX`
* `Q3_KD_PULSE_PER_PX`
* `Q3_POS_FILTER_ALPHA`
* `Q3_VEL_FILTER_ALPHA`
* `Q3_ZERO_DEADBAND_PX`
* `Q3_VISION_LOST_CYCLES`

**为什么：**

用户确认当前阶段继续中心保持，只修编号 1/2。参数调优、方向验证、临时限幅验证和完整第三问目标状态机后续单独处理。

#### Change 6：保留现有 VOFA 9 字段文本输出

当前题目 3 VOFA 输出为 9 字段文本 CSV：

```c
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
```

字段顺序：

```text
rx, raw, ball, target, vel, cmd, rod, lost, run
```

本次不扩展通道，不改 UART，不改 `vofa.c`。

#### Change 7：后续现场调试限幅验证单独处理

在编号 1/2 修复完成并确认数据路径一致后，再单独做“临时限幅验证”。建议优先使用 `±60`，如果动作明显太弱再试 `±80`。

临时调试改法之一：

```c
#define ROD_DEFAULT_MIN_PULSE          (-60)
#define ROD_DEFAULT_MAX_PULSE          (60)
```

或：

```c
#define ROD_DEFAULT_MIN_PULSE          (-80)
#define ROD_DEFAULT_MAX_PULSE          (80)
```

该改动不应混入编号 1/2 数据路径修复提交。它是后续复测提交或现场临时调试改动，用于降低输出饱和造成的机械大动作和执行滞后放大效应。

---

## Impact Analysis

### 1. 对题目 3 的影响

正向影响：

* `g_vision_ready_flag` 每周期只被消费一次。
* `vision_lost_count` 每 50ms 最多加 1。
* 视觉丢失判断恢复为 `Q3_VISION_LOST_CYCLES * 50ms`。
* `ball_pos_px` 不再因两套公式切换产生固定 bias 跳变。
* `ball_vel_px` 不再因位置路径突变而产生非物理尖峰。
* `rod_cmd` 输入更可信，后续判断方向、限幅、D 项和滤波才有意义。

预期行为变化：

* 若之前双递增导致约 250ms 回中，修复后会恢复到约 500ms 回中。
* 修复后控制表现可能与之前不同，因为旧参数是在错误数据路径上运行的。
* 如果仍过冲或饱和，不代表编号 1/2 修复失败，可能是控制方向、输出限幅、视觉跳变、D 项符号、机械惯性、速度 dt 等后续问题。
* 基于 `q3_vofa_20260801_051613.csv`，修复后若仍大幅振荡，不能再按“简单小振荡”处理，应先做方向确认和低限幅复测。

### 2. 对题目 2/4/5/6 的影响

预计无直接影响。

原因：

* 本次只改 `zdt_motor_test_task()` 内视觉数据处理路径。
* 不改 `control_test_task()`。
* 不改循迹 `LineTrace_Update()` / `LineTrace_CalcTurn()`。
* 不改位置环。
* 不改灰度传感器任务。
* 不改按键菜单。

仍需回归：

* 题目 2 循迹启动/停止。
* 题目 4 位置环循迹。
* 题目 5/6 平滑循迹。
* S2 退出运行态。

### 3. 对 UART1 视觉接收的影响

无代码影响。

保留特性：

* UART1 ISR 仍是单槽 latest-value 模式。
* 视觉帧率高于控制周期时，中间帧仍可能被覆盖。
* 本次不引入队列，不逐帧保留历史视觉数据。

理由：

当前中心保持闭环更需要最新球位置。队列可能让控制器处理旧帧，增加延迟和超调风险。

### 4. 对 ZDT 电机驱动的影响

无协议层影响。

保留：

* `RodActuator_SetTargetPulse()`。
* 目标限幅。
* 每周期步进限幅。
* `ZDT_MoveAbsolute()` 帧格式。
* ZDT 反馈队列读取。

可能观察到：

* 由于 `ball_pos_px/ball_vel_px` 更连续，`last_target_pulse` 变化可能更符合真实输入。
* 参数未调时仍可能出现输出饱和和物理过冲。
* 若后续临时把输出限幅降到 ±60/±80，`cmd` 饱和边界会变小，`rod` 追赶滞后和机械冲击应减弱，但响应能力也会下降。

### 5. 对 VOFA / UART2 的影响

无代码影响。

保留：

* 当前题目 3 仍输出 `rx,raw,ball,target,vel,cmd,rod,lost,run` 9 字段文本 CSV。
* `VOFA_SendString()` 仍是阻塞发送。
* 正式比赛前是否关闭 VOFA，需要后续按规则确认。

### 6. 对蓝牙/正式比赛合规性的影响

无代码影响。

当前宏仍为：

```c
#define USE_BLUETOOTH 1
#define USE_VOFA_DEBUG 1
```

本次不处理蓝牙和 VOFA 正式测试禁用策略。

---

## New Issue Risk Assessment

### 风险 1：视觉丢失回中时间变长

**原因：** 修复后 `vision_lost_count` 每周期最多 +1，不再双递增。

**影响：** 回中时间从错误的约 250ms 恢复到设计的约 500ms。

**是否新问题：** 不是新问题，是恢复设计语义。

**应对：** 若实际需要更快回中，后续单独调小 `Q3_VISION_LOST_CYCLES`，不要保留双递增 bug。

### 风险 2：首帧后位置不是严格 0

**原因：** 用户确认保留现逻辑，首帧建立 `vision_zero_offset_px` 后仍应用 `Q3_ZERO_BIAS_PX`。

**影响：** 首帧 raw 约等于 `Q3_ZERO_BIAS_PX`，经滤波进入 `ball_pos_px`。

**是否新问题：** 不是新问题，是用户确认的现有语义。

**应对：** 实施时不要跳过首帧 bias。若后续认为首帧应强制 0，需要另开计划。

### 风险 3：临界区误用导致中断延迟

**原因：** 如果实际修改时把浮点计算或串口发送放入临界区，会影响 UART/FreeRTOS 响应。

**影响：** 可能造成中断延迟、丢字节、控制抖动。

**应对：** 临界区只做快照和清 flag，代码审查必须检查。

### 风险 4：仍然不能完成完整第三问

**原因：** 本次不实现 O→+5cm→-5cm 状态机。

**影响：** 修复后仍只是中心保持。

**是否新问题：** 不是新问题，是用户确认的阶段范围。

**应对：** 后续基于可信中心保持数据，再实现完整题目 3 目标脚本。

### 风险 5：调试数据仍显示测量跳变

**原因：** 最新日志中的 `raw=-338`、大幅跳变等现象可能来自视觉识别失败、默认值、通信异常或目标误检；编号 1/2 只修消费路径，不修视觉源头。

**影响：** 即使 ready flag 和 bias 修好，`raw/ball` 仍可能因输入源跳变造成控制输出突变。

**应对：** 编号 1/2 后若 `raw` 仍异常跳变，再单独计划视觉输入有效性过滤、异常值丢弃或视觉端识别策略修正。

### 风险 6：修复编号 1/2 后仍大幅振荡

**原因：** 最新日志显示当前是“测量跳变 + 输出饱和 + 执行滞后”的组合问题。编号 1/2 只修视觉数据路径一致性，不修控制方向、不调限幅、不修控制律。

**影响：** 修复后数据更可信，但球仍可能继续过冲、饱和或极限环振荡。

**是否新问题：** 不是新问题，是当前日志已暴露的后续控制/执行问题。

**应对：** 修复编号 1/2 后按顺序做后续验证：

1. **先确认控制方向是否反了**：观察球在正/负位置时，摆杆命令方向是否会把球推回中心；同时核对 `Q3_VISION_POS_SIGN`、`Q3_KP_PULSE_PER_PX`、摆杆正负脉冲方向与球运动方向。
2. **再临时降低输出限幅**：将 `ROD_DEFAULT_MIN_PULSE/ROD_DEFAULT_MAX_PULSE` 从 `±180` 降到 `±60`，若动作太弱再试 `±80`，采集同格式 VOFA 日志。
3. 如果低限幅下仍越控越远，优先处理方向符号，不先调 D。
4. 如果方向正确但仍过冲，再单独验证 D 项符号、速度定义、Kp/Kd 和滤波。
5. 这些操作不纳入本次编号 1/2 修复提交，除非用户明确要求另开调试改动。

---

## Logic Validation Against Requirements

### 1. 是否符合当前阶段要求

符合。

当前阶段用户确认：

* 只修编号 1/2。
* 继续中心保持。
* 首帧零位保留现逻辑。

本计划不改变控制目标、不改变控制参数、不改变硬件协议，只修视觉数据路径一致性。

### 2. 是否符合第三问最终题目要求

部分符合。

题目 3 最终目标是 O→+5cm→-5cm 并稳定。本次修复只让中心保持的数据路径可信，是完成第三问前置步骤，但本身不实现 +5cm/-5cm 目标状态机。

计划中必须明确：

* 本次修复完成后，第三问仍处于中心保持阶段。
* 后续还需要 `target_pos_px` 目标状态机才能完成完整第三问。
* 当前日志暴露的问题优先级是方向确认和限幅复测，不应跳过这些直接进入复杂状态机实现。

### 3. 是否适配当前结构

适配。

当前结构本来就是：

* UART1 ISR 生产最新视觉值。
* `zdt_motor_test_task()` 周期性消费视觉值。
* `RodActuator_SetTargetPulse()` 负责输出。

本计划不改变架构，只把消费点从两个收敛为一个。

---

## Verification Steps

### 1. 静态搜索验证

修复后搜索 `g_vision_ready_flag`。

预期结果：

* `uart.c` 中保留一处生产者置位：`g_vision_ready_flag = 1U;`
* `rtos_tasks.c` 中保留一个周期性快照消费入口。
* `rtos_tasks.c` 中可保留一个退出题目 3 清理入口。
* 不再存在题目 3 分支内部第二个 `if (g_vision_ready_flag != 0U)`。

修复后搜索 `raw_ball_pos_px`。

预期结果：

* 只存在统一视觉处理块中的公式。
* 公式必须包含 `+ Q3_ZERO_BIAS_PX`。
* 不存在漏掉 bias 的公式。

修复后搜索 `target_pos_px`。

预期结果：

* 仍为 `const float target_pos_px = 0.0f;`。

### 2. 代码审查验证

检查点：

* 临界区内是否只做快照。
* 是否没有新增队列、互斥量、复杂结构体。
* 是否没有修改 `uart.c`、`rtos_tasks.h`、`ZDT_X42S.c`、`vofa.c`。
* 是否保留 `RodActuator_ReturnCenter()` 和 `Stopwatch_Stop()` 的退出逻辑。
* 是否保留现有 VOFA 9 字段输出。
* 是否没有在编号 1/2 修复中顺手修改 `ROD_DEFAULT_MIN/MAX_PULSE`、Kp、Kd 或 D 项符号。

### 3. 编译验证

使用 CCS/TI Clang 编译。

验收：

* 无编译错误。
* 无新增未使用变量。
* 无宏条件导致的编译分支问题。

### 4. 上板功能验证

#### 用例 A：视觉正常输入

步骤：

1. 启动题目 3。
2. 保持视觉模块正常输出。
3. 观察摆杆和 VOFA 输出。

验收：

* 视觉正常时不误回中。
* `lost` 应长期为 0 或仅短暂波动，不应在稳定输入时长期等于 1。
* `ball_pos_px` 连续变化。
* `last_target_pulse` 不再出现由公式路径切换导致的固定 17px bias 突跳。

#### 用例 B：视觉遮挡

步骤：

1. 启动题目 3。
2. 遮挡视觉输入。
3. 记录回中时间。

验收：

* 约 `Q3_VISION_LOST_CYCLES * 50ms` 后回中。
* 当前应约 500ms。
* 不应再约 250ms 就误判丢失。

#### 用例 C：题目切换

步骤：

1. 进入题目 3。
2. 按 S2 返回菜单。
3. 再次进入题目 3。

验收：

* 退出时摆杆回中。
* 再次进入时重新等待新帧建立零位。
* 不沿用上次残留的 `vision_zero_offset_px`、`ball_pos_px`、`ball_vel_px`。

#### 用例 D：修复后同条件 VOFA 复测

步骤：

1. 使用与 `q3_vofa_20260801_051613.csv` 相同的采集方式记录中心保持日志。
2. 保持 `target_pos_px = 0.0f`。
3. 不改 Kp/Kd、不改 D 项符号、不改输出限幅，先采集编号 1/2 修复后的基线日志。
4. 对比 `rx/raw/ball/target/vel/cmd/rod/lost/run`。

验收：

* 编号 1/2 修复不以“立即稳定”为唯一验收标准。
* 首要验收是数据路径一致：无双消费、无漏 bias 公式、`lost` 不再稳定输入时异常为 1。
* 若 `cmd` 仍长期饱和，则记录为后续“方向确认 + 临时降限幅复测”问题，不回滚编号 1/2。
* 若 `raw/ball` 仍有大幅跳变，则记录为后续视觉测量跳变问题。

#### 用例 E：方向确认实验

该用例属于编号 1/2 修复后的最高优先级复测，不混入编号 1/2 修复提交。

步骤：

1. 保持球在中心附近，手动轻推或放置到视觉正方向一侧，观察 `ball` 正负。
2. 观察控制输出 `cmd` 正负和摆杆实际运动方向。
3. 判断摆杆动作是否把球推回 `target=0`，还是把球推得更远。
4. 再在视觉负方向一侧重复一次。

验收：

* 若球在正方向时，控制动作应使球向中心返回。
* 若球在负方向时，控制动作也应使球向中心返回。
* 若两侧都越控越远，优先处理方向符号：`Q3_VISION_POS_SIGN`、`Q3_KP_PULSE_PER_PX` 或摆杆正负脉冲方向。

#### 用例 F：临时降低输出限幅复测

该用例属于方向确认后的第二优先级复测，不混入编号 1/2 修复提交。

步骤：

1. 在确认方向不反后，临时将输出限幅从 `±180` 降到 `±60`。
2. 采集同格式 VOFA 日志。
3. 如果 `±60` 动作明显不足，再试 `±80`。
4. 对比 `cmd` 饱和比例、`rod` 追赶滞后、`ball` 过冲幅度。

验收：

* `cmd` 饱和边界应从 ±180 变成 ±60 或 ±80。
* `rod` 与 `cmd` 的阶梯追赶幅度和机械冲击应降低。
* 如果低限幅后仍越控越远，则优先回到方向问题，不先调 D。
* 如果方向正确但仍过冲，再进入 D 项、Kp/Kd、滤波和测量跳变处理。

#### 用例 G：题目 2/4/5/6 回归

验收：

* 题目 2 循迹正常。
* 题目 4 位置环循迹正常。
* 题目 5/6 平滑循迹正常。
* 菜单按键切换正常。

### 5. 后续控制律验证建议，不属于本次修复

若编号 1/2 修复、方向确认、临时低限幅复测后仍出现类似当前日志的大幅振荡，下一轮按以下顺序单独验证：

1. 保持编号 1/2 修复后的数据路径不变。
2. 保持已确认正确的方向符号。
3. 保持较低输出限幅作为安全边界，例如 ±60 或 ±80。
4. 再验证 D 项符号和速度定义，必要时做“关 D”对比实验。
5. 如果 D 项方向确认无误，再调 `Q3_KP_PULSE_PER_PX`、`Q3_KD_PULSE_PER_PX`、`Q3_POS_FILTER_ALPHA`、`Q3_VEL_FILTER_ALPHA`。
6. 如果 `raw` 仍跳变，再单独做视觉异常值处理。
7. 上述控制律验证需要单独计划和提交，不混入编号 1/2 修复。

---

## Assumptions & Decisions

1. 本次只修改 `user/rtos_tasks.c`。
2. 不使用队列。
3. 不使用互斥量保护 ISR 共享变量。
4. 使用短临界区快照 `g_vision_x_offset` 和 `g_vision_ready_flag`。
5. 实际执行时推荐沿用工程现有 PRIMASK 临界区风格。
6. 统一公式必须包含 `+ Q3_ZERO_BIAS_PX`。
7. 首帧建立 `vision_zero_offset_px` 后仍应用 `Q3_ZERO_BIAS_PX`。
8. `target_pos_px = 0.0f` 不变。
9. 编号 1/2 修复不调参数。
10. 编号 1/2 修复不改 `ROD_DEFAULT_MIN_PULSE/ROD_DEFAULT_MAX_PULSE`。
11. 编号 1/2 修复不实现 +5cm/-5cm 状态机。
12. 编号 1/2 修复不扩展 VOFA 通道。
13. 编号 1/2 修复不改变 UART1 视觉协议。
14. `q3_vofa_20260801_051613.csv` 已证明当前问题是“测量跳变 + 输出饱和 + 执行滞后”的组合问题。
15. 修复编号 1/2 后，最高优先级不是直接调 D，而是先确认控制方向是否反了。
16. 方向确认后，第二优先级是临时把输出限幅从 ±180 降到 ±60 或 ±80 后重测。
17. 只有在方向正确且低限幅下仍过冲时，才进入 D 项符号、Kp/Kd、滤波和视觉跳变处理。
18. 修复后若仍过冲，不把编号 1/2 回滚为旧双消费结构。

---

## What I Still Need Before Editing

本次编号 1/2 代码修改本身已经足够明确，不再强依赖额外信息。

如果后续要做“方向确认 + 临时限幅复测”的调试改动，需要现场确认：

1. 摆杆正脉冲时，球实际会向哪一侧运动。
2. 视觉 `raw/ball` 正方向对应物理哪一侧。
3. 你希望临时限幅先用 `±60` 还是直接用 `±80`。

默认建议：先用 `±60`，若动作明显太弱再试 `±80`。

---

## Executor Checklist

### 编号 1/2 修复提交

* [ ] 只修改 `user/rtos_tasks.c`。
* [ ] 在 `zdt_motor_test_task()` 每轮循环新增 `has_new_vision` 和 `vision_x_snapshot`。
* [ ] 在 ZDT 反馈队列清理后建立唯一视觉快照入口。
* [ ] 快照临界区只读 `g_vision_x_offset`、清 `g_vision_ready_flag`、置 `has_new_vision`。
* [ ] 原第一段视觉处理改为依赖 `has_new_vision` 和 `vision_x_snapshot`。
* [ ] 统一公式包含 `+ Q3_ZERO_BIAS_PX`。
* [ ] 首帧建立零位后仍执行统一公式。
* [ ] 删除题目 3 分支内第二套视觉消费逻辑。
* [ ] 保留丢帧回中逻辑。
* [ ] 保留 `rod_cmd` 控制律。
* [ ] 保留 `target_pos_px = 0.0f`。
* [ ] 保留 VOFA 9 字段输出。
* [ ] 不修改 `ROD_DEFAULT_MIN/MAX_PULSE`。
* [ ] 不修改 Kp/Kd/D 项符号。
* [ ] 不修改 `uart.c`、`rtos_tasks.h`、`ZDT_X42S.c`、`vofa.c`。
* [ ] 静态搜索确认只有一个周期性 ready flag 消费入口。
* [ ] 静态搜索确认不存在漏 bias 的 `raw_ball_pos_px` 公式。
* [ ] 编译验证。
* [ ] 上板验证视觉正常不误回中。
* [ ] 上板验证遮挡视觉约 500ms 回中。
* [ ] 回归题目 2/4/5/6。

### 修复后复测优先级

* [ ] 采集编号 1/2 修复后的同格式 VOFA 基线日志。
* [ ] 第一优先级：确认控制方向是否反了。
* [ ] 第二优先级：方向正确后，临时把输出限幅降到 ±60 复测。
* [ ] 如果 ±60 动作太弱，再试 ±80。
* [ ] 如果低限幅仍越控越远，优先修方向符号。
* [ ] 如果方向正确但仍过冲，再进入 D 项、Kp/Kd、滤波、视觉跳变处理。
