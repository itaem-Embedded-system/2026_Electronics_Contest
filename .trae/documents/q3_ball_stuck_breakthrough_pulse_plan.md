# 题目 3 小球卡死检测与突破脉冲计划

## Summary

在题目 3 的视觉闭环步进任务中增加“小球在非中心位置停下”的卡死检测：当视觉持续有效、滤波后小球位置偏离中心、滤波速度接近 0，并连续满足若干个 50ms 控制周期后，判定为卡死。判定后发送一次更大的突破脉冲，推动摆杆尝试解除机械卡死，然后进入短暂冷却/观察窗口，避免连续重复冲击。

用户已确认：

- 判定条件采用“更敏感”策略。
- 突破动作需要“大脉冲”，允许绕过现有 `RodActuator_SetTargetPulse()` 的 45 脉冲单周期限步。

## Current State Analysis

### 视觉输入

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\uart.c`

- `UART_1_INST_IRQHandler()` 按行接收 ASCII 有符号整数，解析为 `g_vision_x_offset`。
- 收到有效一帧后置位 `g_vision_ready_flag = 1U`。
- 当前视觉变量只表达“有新视觉数据”，不表达小球是否移动或是否稳定。

### 题目 3 控制任务

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.c`

- `zdt_motor_test_task()` 周期为 50ms。
- 每周期在临界区快照 `g_vision_x_offset` 并清除 `g_vision_ready_flag`。
- 已经计算了：
  - `ball_pos_px`：滤波后小球位置。
  - `ball_vel_px`：由相邻滤波位置差估计出的滤波速度。
  - `vision_lost_count`：视觉丢失计数。
- 当前只有视觉丢失回中逻辑：`vision_lost_count >= Q3_VISION_LOST_CYCLES` 时 `RodActuator_ReturnCenter()`。
- 当前没有卡死检测、卡死状态、突破脉冲、冷却窗口。

### 摆杆发送接口

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.c`

- `RodActuator_SetTargetPulse()` 会先做绝对行程限幅，再做单周期变化限幅。
- `RodActuator_LimitStep()` 当前限制每次最多变化 `g_rod_config.max_step_per_update`，默认 45 脉冲。
- 用户要求突破动作需要大脉冲，因此突破动作不能复用 `RodActuator_SetTargetPulse()` 的普通入口，否则会被限步削弱。
- 可复用 `RodActuator_LimitPulse()` 和 `RodActuator_SendAbsolute()`：突破目标仍遵守 `ROD_DEFAULT_MIN_PULSE` 到 `ROD_DEFAULT_MAX_PULSE` 的行程限制，但绕过单周期限步。

### ZDT 底层接口

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\ZDT_X42S.c`

- `ZDT_MoveAbsolute()` 发送绝对位置命令。
- `ZDT_MoveRelative()` 发送相对当前位置命令。
- 本计划采用“绝对目标大步跳转”的突破方式，而不是相对运动：这样软件目标 `g_rod_target_pulse` 能与下发的绝对位置保持一致，减少突破后 PD 控制目标不同步的风险。

## Proposed Changes

### 1. 在 `rtos_tasks.c` 增加题目 3 卡死参数宏

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.c`

位置：现有题目 3 参数宏附近，和 `Q3_ZERO_DEADBAND_PX`、`Q3_KP_PULSE_PER_PX`、`Q3_VISION_LOST_CYCLES` 保持同一风格。

新增建议参数：

```c
#define Q3_STUCK_POS_THRESHOLD_PX      6.0f
#define Q3_STUCK_VEL_THRESHOLD_PX      2.0f
#define Q3_STUCK_CONFIRM_COUNT         5U
#define Q3_BREAKTHROUGH_PULSE          70
#define Q3_BREAKTHROUGH_COOLDOWN_COUNT 6U
```

说明：

- `Q3_STUCK_POS_THRESHOLD_PX = 6.0f`：大于当前 `Q3_ZERO_DEADBAND_PX = 4.0f`，避免中心附近误触发。
- `Q3_STUCK_VEL_THRESHOLD_PX = 2.0f`：更敏感，允许低速/近似静止时触发。
- `Q3_STUCK_CONFIRM_COUNT = 5U`：5 个 50ms 周期，约 250ms，符合用户“更敏感”的选择。
- `Q3_BREAKTHROUGH_PULSE = 70`：大于普通 45 脉冲限步，但仍低于默认绝对行程 80 脉冲。
- `Q3_BREAKTHROUGH_COOLDOWN_COUNT = 6U`：突破后约 300ms 不重复触发，让机构和视觉有时间响应。

这些参数后续需要实车微调，尤其是突破方向和脉冲大小。

### 2. 增加绕过单周期限步的大脉冲发送入口

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.c`

位置：`RodActuator_SetTargetPulse()` 附近。

新增一个 `static bool RodActuator_SetTargetPulseFast(int32_t target_pulse)`，只供本文件内部题目 3 突破动作使用。

行为：

- 检查 `g_rod_ready`。
- 调用 `RodActuator_LimitPulse()` 做绝对行程限幅。
- 不调用 `RodActuator_LimitStep()`。
- 调用 `RodActuator_SendAbsolute()`。
- 发送成功后更新 `g_rod_target_pulse = target_pulse`。

这样突破动作能绕过 45 脉冲单周期限制，但仍不会超出机械行程限制。

### 3. 在 `zdt_motor_test_task()` 增加卡死状态变量

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.c`

位置：`vision_lost_count` 附近。

新增局部变量：

```c
uint16_t stuck_count = 0;
uint16_t breakthrough_cooldown = 0;
```

重置点：

- 退出题目 3 运行态时清零。
- 刚进入题目 3 运行态时清零。
- 视觉丢失时清零。
- 小球回到中心附近或速度不满足静止条件时清零。

### 4. 在正常视觉闭环分支中加入卡死判定和突破动作

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.c`

位置：`vision_lost_count < Q3_VISION_LOST_CYCLES` 的正常控制分支内，计算普通 PD 命令前或后均可；推荐在计算普通 PD 后、发送普通目标前插入，便于突破动作覆盖本周期普通控制。

判定条件：

- `has_new_vision == true`：必须是视觉仍在持续输出，不把丢帧当卡死。
- `fabsf(ball_pos_px - target_pos_px) >= Q3_STUCK_POS_THRESHOLD_PX`：小球停在非中心位置。
- `fabsf(ball_vel_px) <= Q3_STUCK_VEL_THRESHOLD_PX`：滤波速度接近 0。
- `breakthrough_cooldown == 0U`：冷却结束。

计数逻辑：

- 连续满足条件则 `stuck_count++`，最多饱和到 `UINT16_MAX`。
- 任一条件不满足则 `stuck_count = 0`。
- `stuck_count >= Q3_STUCK_CONFIRM_COUNT` 时触发突破。

突破方向：

- 以普通 PD 方向为优先依据：`rod_cmd` 若为正，则突破目标向正方向；若为负，则向负方向。
- 若普通 `rod_cmd` 接近 0，则用位置误差方向兜底：`pos_error_px >= 0` 时向 `Q3_BREAKTHROUGH_PULSE`，否则向 `-Q3_BREAKTHROUGH_PULSE`。
- 发送目标为 `breakthrough_target_pulse = ±Q3_BREAKTHROUGH_PULSE`，不是在当前目标上继续累加，避免连续触发时顶死在一侧。
- 使用 `RodActuator_SetTargetPulseFast(breakthrough_target_pulse)` 发送。
- 发送成功后：
  - `last_target_pulse = breakthrough_target_pulse`
  - `stuck_count = 0`
  - `breakthrough_cooldown = Q3_BREAKTHROUGH_COOLDOWN_COUNT`

冷却逻辑：

- 每个正常控制周期若 `breakthrough_cooldown > 0U`，则递减。
- 冷却期间不再次触发突破，但普通 PD 控制继续运行，让系统恢复中心闭环。

### 5. 调试输出增加卡死观测字段

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.c`

位置：`#if USE_VOFA_DEBUG` 下的 `snprintf()`。

计划在现有 CSV 末尾追加：

- `stuck_count`
- `breakthrough_cooldown`

这样不改变前面字段含义，只在末尾增加观测量，便于 VOFA 或串口日志判断误触发与触发时机。

### 6. 修改日志

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\CLAUDE.md`

根据工作区规则，实施代码修改后需要追加修改日志，内容包括：

- 修改原因：小球可能在非中心机械卡死，普通 PD 输出无法脱困。
- 修改内容：题目 3 增加非中心低速连续检测、突破大脉冲、冷却窗口和 VOFA 调试字段。
- 仍需实车验证：视觉比例/方向、卡死阈值、突破脉冲大小、是否会误触发或过冲。

## Assumptions & Decisions

- 卡死定义仅基于视觉位置和视觉速度，不使用 ZDT 驱动板 `isStall`，因为当前代码尚未解析并验证 ZDT 反馈状态。
- 卡死只在题目 3 运行态生效，不影响题目 2/4/5/6 的底盘与循迹控制。
- 突破动作在任务上下文发送，ISR 中不做任何阻塞操作。
- 突破仍遵守绝对行程限制，避免超过 `ROD_DEFAULT_MIN_PULSE` 和 `ROD_DEFAULT_MAX_PULSE`。
- 采用更敏感的初始阈值：约 250ms 判定卡死。实车若误触发，可优先把 `Q3_STUCK_CONFIRM_COUNT` 增大到 8~10，或把 `Q3_STUCK_VEL_THRESHOLD_PX` 降到 1.0~1.5。
- 突破方向可能受机构方向、`Q3_VISION_POS_SIGN`、`Q3_KP_PULSE_PER_PX` 符号影响；计划以普通 PD 命令方向为准，实车若方向反了，只需要调整突破方向符号逻辑或统一修正视觉/控制符号。

## Verification Steps

1. 在 CCS Theia 中编译工程，确认无 C 语法错误、无未声明函数、无格式化字符串参数不匹配。
2. 进入题目 3，视觉正常输出且小球在中心附近时，确认 `stuck_count` 保持 0，不发送突破脉冲。
3. 手动让小球停在非中心位置，观察 `stuck_count` 连续增加并在约 250ms 后触发一次突破。
4. 突破后观察 `breakthrough_cooldown` 递减，确认冷却期间不会连续重复触发。
5. 观察摆杆目标不超过默认绝对行程 ±80 脉冲。
6. 使用 VOFA/串口日志确认新增字段能反映卡死计数和冷却状态。
7. 实车验证突破脉冲方向：若突破使小球更卡，应反转突破目标方向逻辑。
8. 根据实车效果微调 `Q3_STUCK_POS_THRESHOLD_PX`、`Q3_STUCK_VEL_THRESHOLD_PX`、`Q3_STUCK_CONFIRM_COUNT`、`Q3_BREAKTHROUGH_PULSE`。
