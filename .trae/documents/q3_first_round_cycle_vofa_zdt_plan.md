# 第三题第一轮控制周期与调试输出优化计划

## Summary

本计划用于第一轮优化第三题视觉闭环摆杆控制，目标是：降低第三题 ZDT 控制任务周期、确保比赛运行时不产生 VOFA 阻塞输出、减少无意义 ZDT 串口命令，并修复第三题退出后再次进入时的视觉零位状态残留。

第一轮改动只限定在 `empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)/user/rtos_tasks.c`，不修改底盘控制、题目菜单、UART1 视觉 ISR、UART2 发送函数、ZDT 协议层、灰度循迹和位置环逻辑，以降低对题目 2/4/5/6 的影响风险。

## Current State Analysis

### 入口与任务边界

- 题目菜单在 `user/rtos_tasks.c` 的 `Key_Scan_Task()` 中运行。
- 选择题目 3 后，菜单只设置 `g_question_ui_state = 1`，不启动底盘电机动作。
- 第三题实际由 `zdt_motor_test_task()` 后台检测：
  - `g_question_ui_state == 1U`
  - `g_selected_question == 3U`
- 第三题任务只控制 ZDT 摆杆，不写 `target_speed`、`target_turn`，当前与题目 2/4/5/6 的底盘运动链路边界清晰。

### 当前第三题控制周期

- `zdt_motor_test_task()` 末尾使用 `vTaskDelay(pdMS_TO_TICKS(50))`。
- 当前第三题闭环约 20Hz。
- 视觉位置滤波、速度估计、视觉丢失计数、ZDT 目标发送都跟随这个 50ms 周期运行。

### 当前 VOFA/printf 状态

- `USE_VOFA_DEBUG` 定义在 `user/rtos_tasks.h`，当前值为 `0`。
- 因此 `zdt_motor_test_task()` 内的 `snprintf()` 和 `VOFA_SendString()` CSV 输出当前不会编译进程序。
- `VOFA_SendString()` 位于 `user/uart.c`，通过 UART2 逐字节阻塞发送。
- `fputc()` 也重定向到 UART2 阻塞发送。
- 第一轮不应打开 `USE_VOFA_DEBUG`，并应在第三题代码里增加更明确的本地调试输出开关，防止后续把全局 VOFA 宏打开后第三题比赛控制周期被串口输出拖慢。

### 当前 ZDT 发送路径

第三题输出路径为：

```text
zdt_motor_test_task()
  -> RodActuator_SetTargetPulse()
  -> RodActuator_LimitPulse()
  -> RodActuator_LimitStep()
  -> RodActuator_SendAbsolute()
  -> ZDT_MoveAbsolute()
  -> ZDT_UART_Send()
```

`ZDT_UART_Send()` 在 `user/ZDT_X42S.c` 中轮询 TX FIFO 并等待 UART busy 结束，不是 DMA 或中断异步发送。正常 13 字节位置命令不长，但频繁发送相同目标会占用 UART0 和任务时间。

当前 `RodActuator_SetTargetPulse()` 即使限幅、限步后的目标等于 `g_rod_target_pulse`，仍会调用 `RodActuator_SendAbsolute()`。此外，第三题未运行且 `contest_started == false` 时，`zdt_motor_test_task()` 每 50ms 调用一次 `RodActuator_SetTargetPulse(0)`，会持续发送回中命令。

### 当前状态残留风险

`zdt_motor_test_task()` 中 `vision_zero_ready` 是局部状态。退出第三题时，代码清理了：

- `vision_zero_offset_px`
- `ball_pos_px`
- `last_ball_pos_px`
- `ball_vel_px`
- `last_target_pulse`
- `vision_lost_count`
- `g_vision_ready_flag`

但没有清 `vision_zero_ready`。因此再次进入题目 3 时，可能不会重新用首帧建立视觉零点，存在零位错误风险。

## Proposed Changes

### 1. 在 `user/rtos_tasks.c` 新增第三题控制周期宏

位置：第三题参数区附近，即 `Q3_*` 宏定义附近。

计划新增：

```c
#define Q3_CONTROL_PERIOD_MS           20U
#define Q3_DEBUG_OUTPUT_ENABLE         0U
#define Q3_DEBUG_OUTPUT_PERIOD_MS      100U
#define Q3_DEBUG_DIVIDER               (Q3_DEBUG_OUTPUT_PERIOD_MS / Q3_CONTROL_PERIOD_MS)
```

要求：

- `Q3_CONTROL_PERIOD_MS` 第一轮设为 `20U`，把第三题闭环从 50ms 降到 20ms。
- `Q3_DEBUG_OUTPUT_ENABLE` 第一轮固定为 `0U`，即使后续 `USE_VOFA_DEBUG` 被打开，第三题默认也不输出 CSV。
- `Q3_DEBUG_DIVIDER` 用于将来调参时低频输出，不直接按硬编码 `2U` 分频。

原因：把第三题周期配置集中化，避免任务循环、注释和调试输出分频各自硬编码。

### 2. 调整与控制周期强相关的第三题注释和计数

位置：`user/rtos_tasks.c` 的 `Q3_TARGET_STEP_PX`、`Q3_REACH_CONFIRM_COUNT`、`Q3_HOLD_POSITIVE_CYCLES`、`Q3_VISION_LOST_CYCLES` 附近。

计划：

- 更新注释中的“50ms 周期”为“Q3_CONTROL_PERIOD_MS 周期”。
- 第一轮不接入 O→+5cm→-5cm 状态机，所以 `Q3_TARGET_STEP_PX`、`Q3_REACH_CONFIRM_COUNT`、`Q3_HOLD_POSITIVE_CYCLES` 先只修正注释，不改变数值。
- `Q3_VISION_LOST_CYCLES` 从 `10U` 改为 `25U`，保持视觉丢失判定约 0.5s：
  - 原来 10 × 50ms = 500ms
  - 新周期 25 × 20ms = 500ms

原因：第一轮降周期后，最直接会改变实际时间含义的是视觉丢失回中。如果不调整，10 × 20ms 只剩约 200ms，可能造成正常视觉帧抖动时误判丢失。

### 3. 让摆杆每周期最大变化量随周期缩短而保守调整

位置：`user/rtos_tasks.c` 的摆杆默认参数区。

计划：

- 将 `ROD_DEFAULT_MAX_STEP` 从 `45` 改为 `25`。
- 保持 `ROD_DEFAULT_RPM = 220` 和 `ROD_DEFAULT_ACC = 50` 不变。

原因：控制周期从 50ms 降到 20ms 后，若每周期仍允许变化 45 脉冲，单位时间最大变化量会从约 900 pulse/s 增加到约 2250 pulse/s，动作会明显变硬。第一轮先把每周期步进降到 25，单位时间最大变化量约 1250 pulse/s，比原来略快但不激进。

### 4. 在 `RodActuator_SetTargetPulse()` 中跳过重复目标发送

位置：`user/rtos_tasks.c` 的 `RodActuator_SetTargetPulse()`。

计划逻辑：

1. 保留原有 `g_rod_ready` 检查。
2. 保留 `RodActuator_LimitPulse()`。
3. 保留 `RodActuator_LimitStep()`。
4. 在限幅和限步完成后增加判断：

```c
if (target_pulse == g_rod_target_pulse) {
    return true;
}
```

5. 只有目标实际发生变化时才调用 `RodActuator_SendAbsolute()`。

原因：避免零位附近、视觉丢失回中完成后、非运行态空闲时重复发送相同绝对位置命令，降低 UART0 占用，不改变最终目标位置语义。

影响控制边界：该函数只属于摆杆执行器封装，不参与底盘直流电机 PWM、循迹或位置环。

### 5. 修改第三题非运行态行为，避免空闲时持续回中发送

位置：`user/rtos_tasks.c` 的 `zdt_motor_test_task()` 中：

```c
if (!question3_running) {
    if (contest_started) {
        ...
    } else {
        RodActuator_SetTargetPulse(0);
    }
}
```

计划：

- 删除或不执行 `contest_started == false` 时的每周期 `RodActuator_SetTargetPulse(0)`。
- 保留初始化后的 `RodActuator_SetTargetPulse(0)`。
- 保留从第三题退出时的 `RodActuator_ReturnCenter()`。
- 退出回中后依赖第 4 步“重复目标跳过”避免回中完成后继续重复发送。

原因：第三题没有运行时不需要每 20ms 重复发 0 位置命令。这样能避免题目 2/4/5/6 运行期间 ZDT_Test 后台任务持续占用 UART0 和 CPU 时间。

### 6. 修复第三题退出后视觉零位状态残留

位置：`zdt_motor_test_task()` 的 `contest_started` 退出分支。

计划：

- 在退出第三题清理状态时补充：

```c
vision_zero_ready = false;
```

- 在新进入第三题的 `!contest_started` 分支也明确设置：

```c
vision_zero_ready = false;
```

原因：确保每次进入题目 3 都使用新收到的首帧视觉数据建立零点，避免上一次运行留下的 `vision_zero_ready` 导致 `vision_zero_offset_px` 错误。

### 7. 保护第三题 VOFA 调试输出默认关闭

位置：`zdt_motor_test_task()` 内 `#if USE_VOFA_DEBUG` 代码块。

计划：

- 将条件编译改为：

```c
#if USE_VOFA_DEBUG && Q3_DEBUG_OUTPUT_ENABLE
```

- `vofa_debug_div >= 2U` 改为使用 `Q3_DEBUG_DIVIDER`。
- 第一轮 `Q3_DEBUG_OUTPUT_ENABLE` 为 `0U`，所以第三题比赛运行时不会编译阻塞输出。

原因：源码当前 `USE_VOFA_DEBUG` 已是 0，但增加第三题本地开关可以防止后续为了其他模块打开 VOFA 时，第三题控制任务意外恢复阻塞 CSV 输出。

### 8. 将第三题任务延时改为新周期宏

位置：`zdt_motor_test_task()` 末尾。

计划：

```c
vTaskDelay(pdMS_TO_TICKS(Q3_CONTROL_PERIOD_MS));
```

替换当前：

```c
vTaskDelay(pdMS_TO_TICKS(50));
```

原因：真正降低第三题闭环周期，并让后续调周期只改一个宏。

### 9. 更新 `RTOS_Tasks_Init()` 中 ZDT_Test 栈注释

位置：`user/rtos_tasks.c` 中创建 `ZDT_Test` 任务处。

计划：

- 当前注释说任务包含 VOFA 缓冲、`snprintf` 和多组浮点局部变量。
- 因第一轮默认不编译第三题 VOFA 输出，可把注释改为强调“第三题视觉滤波、ZDT 帧缓存和浮点局部变量”。
- 不改变栈大小 `512`。

原因：只修正注释，不调整任务资源，避免引入栈风险。

### 10. 写修改日志

位置：`user/rtos_tasks.c` 文件末尾或现有修改日志区域对应位置。

计划按工程要求新增一条修改日志，格式包含：

- 修改背景：第三题摆杆闭环响应慢，需降低主控周期并避免阻塞调试输出和重复 ZDT 命令影响控制节拍。
- 修改内容：列出周期宏、视觉丢失计数、`ROD_DEFAULT_MAX_STEP`、重复目标跳过、非运行态不持续发 0、视觉零位复位、VOFA 本地开关。
- 仍需实车验证：
  - 题目 3 是否重新进入后零点正确。
  - 视觉正常时是否不会误判丢失。
  - 摆杆 20ms 周期下是否响应更快且不过冲。
  - 题目 2/4/5/6 运行时底盘循迹和位置环是否不受 ZDT_Test 空闲态影响。

## Assumptions & Decisions

1. 第一轮只修改 `user/rtos_tasks.c`。
2. 不修改 `user/rtos_tasks.h` 中的全局 `USE_VOFA_DEBUG`，因为当前源码已经是 `0`。
3. 不修改 `user/uart.c` 的 `VOFA_SendString()` 和 `fputc()`，避免影响 UART2 指令和 printf 行为。
4. 不修改 `user/ZDT_X42S.c`，避免改变 ZDT 协议层和其他 ZDT API 行为。
5. 不修改 `Q3_KP_PULSE_PER_PX`、`Q3_KD_PULSE_PER_PX`、`Q3_POS_FILTER_ALPHA`、`Q3_VEL_FILTER_ALPHA`，避免把调参变化和周期/调度优化混在一起。
6. 不接入 O→+5cm→-5cm 状态机；本轮只优化中心保持的执行节拍和状态边界。
7. `Q3_CONTROL_PERIOD_MS` 第一轮选择 20ms，而不是 10ms。原因是 20ms 已将闭环从 20Hz 提升到 50Hz，同时 UART0 和 M0+ 软件浮点压力可控。
8. `ROD_DEFAULT_RPM` 和 `ROD_DEFAULT_ACC` 第一轮保持不变。原因是周期变快本身已提高响应，先避免同时增加电机速度和加速度造成过冲。

## Verification Steps

### 静态检查

1. 在 CCS Theia 中编译工程，确认无新增 warning/error。
2. 检查 `USE_VOFA_DEBUG && Q3_DEBUG_OUTPUT_ENABLE` 条件下，默认 `Q3_DEBUG_OUTPUT_ENABLE=0U` 时不会编译第三题 CSV 输出变量和 `snprintf()`。
3. 检查 `RodActuator_SetTargetPulse()` 在目标未变化时直接返回 `true`，且目标变化时仍正常调用 `RodActuator_SendAbsolute()` 并更新 `g_rod_target_pulse`。
4. 检查 `zdt_motor_test_task()` 只有初始化、进入/退出第三题、视觉丢失回中、目标变化时才发送 ZDT 位置命令，不在非运行态持续发送 0。

### 上板验证

1. 开机停在菜单，不进入题目 3，观察 ZDT 不应持续收到 0 位置命令导致异常动作。
2. 进入题目 3，确认秒表启动、ZDT 使能、摆杆进入中心保持。
3. 退出题目 3 再重新进入，确认视觉零点重新建立，球在中心附近时 `ball_pos_px` 不出现明显历史偏置。
4. 遮挡或断开视觉输入约 0.5s，确认摆杆回中；短时间视觉帧抖动不应立即误判丢失。
5. 运行题目 2、题目 4、题目 5/6，确认底盘循迹、位置环和 OLED 菜单行为不因第三题改动改变。

## Rollback Plan

如实车出现摆杆换向过猛或振荡：

1. 先把 `Q3_CONTROL_PERIOD_MS` 从 `20U` 回调到 `50U`。
2. 把 `Q3_VISION_LOST_CYCLES` 从 `25U` 回调到 `10U`。
3. 把 `ROD_DEFAULT_MAX_STEP` 从 `25` 回调到 `45`。
4. 保留 `vision_zero_ready = false` 修复和重复目标跳过逻辑，因为这两项属于状态/通信边界修复，通常不应导致控制变差。
