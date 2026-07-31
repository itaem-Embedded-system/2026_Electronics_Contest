# 题目2进入后 OLED 崩溃/回菜单诊断计划

## Summary

用户现象：操作按键进入题目2后，OLED 像是崩溃并回到题目选择界面；选择1能正常计时。用户怀疑是否因为 VOFA 9 通道发送数据过多导致系统崩溃重启。

只读排查结论：

1. **当前代码中，题目2运行时不会发送 VOFA 9 通道数据。**
   题目3 VOFA 输出被 `question3_running` 限制，而 `question3_running = (g_question_ui_state == 1U && g_selected_question == 3U)`。因此题目2下不会执行 `VOFA_SendString()`。

2. **VOFA 发送过多导致题目2崩溃的可能性较低。**
   VOFA 9 通道对题目3有阻塞和栈压力风险，但不是题目2一进入就回菜单的首要嫌疑。

3. **更危险/更像的根因是系统复位。**
   `empty.c` 中 FreeRTOS malloc failed hook 和 stack overflow hook 都会直接 `NVIC_SystemReset()`，复位后全局状态回到初始菜单态，看起来就像 OLED 回到题目选择界面。

4. **题目2启动电机动作是高优先级嫌疑。**
   题目2入口使用普通 `LineTrace_Start()`，目标速度可能从 0 直接跳到 16；选择1能正常计时说明“只显示/计时”没问题，进入题目2才启动循迹电机，电机启动冲击、供电跌落或控制任务异常都可能触发复位。

本计划目标：用最小改动定位是“真复位 / 栈溢出 / malloc失败 / 电机供电冲击 / ZDT任务干扰 / OLED刷新负载 / VOFA发送”中的哪一类，并优先降低题目2启动风险。

---

## Current State Analysis

### 1. 功能宏当前状态

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.h`

当前关键宏：

```c
#define USE_IMU_SENSOR       0
#define USE_ULTRASONIC       0
#define USE_GRAY_SENSOR      1
#define USE_OLED_DISPLAY     1
#define USE_ZDT_STEPPER      1
#define USE_BLUETOOTH        0
#define USE_VOFA_DEBUG       1
```

含义：

- 题目2依赖灰度、OLED、底盘控制。
- ZDT 步进任务仍会创建，即使题目2不需要摆杆。
- VOFA 调试开启，但当前只在题目3运行时发送。

### 2. 题目2启动路径

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.c`

菜单状态相关变量：

```c
static volatile uint8_t g_question_ui_state = 0;
static volatile uint8_t g_selected_question = 2;
```

因此复位后天然会回到题目选择界面，并默认选中题目2。

按键进入题目2路径：

```text
Key_Scan_Task()
  -> S4 确认
  -> g_question_ui_state = 1
  -> if (g_selected_question == 2)
       LineTrace_Start(&g_line_trace_q2_config)
```

`LineTrace_Start()` 会启动循迹控制、秒表和蜂鸣提示，并让 `Ctrl_Task` 每 50ms 开始输出电机 PWM。

### 3. 题目2与题目1/只计时的差异

题目2会启动：

- 循迹状态机
- 灰度输入闭环
- 底盘 PID
- 电机 PWM 输出
- 停止线判断

而“选择1能正常计时”说明 OLED 显示和秒表本身大概率不是唯一问题。题目2新增的最大风险是：**电机启动和控制闭环开始运行**。

### 4. VOFA 9 通道发送风险评估

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.c`

当前 VOFA 9 通道输出逻辑在 `zdt_motor_test_task()` 内，并且被限制为题目3运行时：

```c
if (question3_running) {
    vofa_debug_div++;
    if (vofa_debug_div >= 2U) {
        ...
        VOFA_SendString(vofa_buf);
    }
}
```

题目2时：

```c
question3_running == false
```

所以不会发送。

风险判断：

- 对题目2：低风险，不是首要嫌疑。
- 对题目3：中等风险，因为 `VOFA_SendString()` 是阻塞逐字节发送，且 `snprintf()` 有栈和耗时开销。但当前 10Hz、9字段文本量不算大。

### 5. 系统复位钩子

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\empty.c`

当前：

```c
void vApplicationMallocFailedHook(void)
{
    __disable_irq();
    Motor_SetPWM(1, 0);
    Motor_SetPWM(2, 0);
    ...
    NVIC_SystemReset();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    __disable_irq();
    Motor_SetPWM(1, 0);
    Motor_SetPWM(2, 0);
    ...
    NVIC_SystemReset();
}
```

这会掩盖根因：只要发生 malloc 失败或栈溢出，系统直接复位，用户看到的就是“回到题目选择界面”。

### 6. OLED 任务状态

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.c`

OLED 任务每 20ms 全屏刷新，且使用 `OLED_Printf()`。

任务栈目前为 768 words，理论上已比较保守，但 20ms 全屏刷新仍有一定 CPU/I2C 负载。它可能加剧问题，但如果选择1能正常计时，OLED 本身不是最高优先级嫌疑。

### 7. ZDT 任务始终创建

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.c`

当前 `USE_ZDT_STEPPER=1`，所以 ZDT 任务会创建。即使非题目3，它也会初始化步进电机并周期回中。

这不是题目2的必需模块。若存在串口阻塞、供电干扰或任务栈问题，可能间接影响题目2。建议作为 A/B 验证项。

---

## Proposed Changes

### Change 1：先增加故障可观测性，避免 hook 直接无记录复位

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\empty.c`

目的：确认是否 FreeRTOS 栈溢出或 malloc failed 导致系统复位。

建议修改：

1. 在 `vApplicationMallocFailedHook()` 中设置故障码，例如：

```c
g_fault_code = 1;
g_fault_task_name = "MallocFail";
```

2. 在 `vApplicationStackOverflowHook()` 中设置故障码和任务名：

```c
g_fault_code = 2;
g_fault_task_name = pcTaskName;
```

3. 调试阶段不要立刻 `NVIC_SystemReset()`，改为停机保持：

```c
while (1) {
    DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
    delay_cycles(3200000);
}
```

或复用已有 `Fault_RecordAndHalt()`。

原因：

- 当前回菜单很像复位。
- 若是栈溢出，必须知道是哪一个任务。
- 这是定位“危险问题”的第一优先级。

影响：

- 调试阶段系统不再自动复位，而是停在故障态。
- 定位完成后可恢复为复位或保留故障提示机制。

### Change 2：题目2改为平滑起步，降低电机启动冲击

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.c`

位置：`Key_Scan_Task()` 中题目2入口。

当前：

```c
if (g_selected_question == 2) {
    LineTrace_Start(&g_line_trace_q2_config);
}
```

建议改为：

```c
if (g_selected_question == 2) {
    LineTrace_StartSmoothWithStep(&g_line_trace_q2_config, 0.5f);
}
```

原因：

- 题目2当前普通启动会直接给速度目标。
- 电机启动瞬间电流/干扰可能导致 MCU/OLED 复位。
- 题目5/6 已有平滑起步机制，复用即可，不新增控制结构。

影响：

- 题目2起步更柔和。
- 不改变题目2循迹参数本身。
- 若问题明显改善，说明启动冲击是高概率根因。

### Change 3：为题目2提供临时“只计时不跑车”验证开关

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.c`

目的：区分 OLED/菜单问题和电机/控制问题。

建议只在调试阶段临时做 A/B：

- 进入题目2后只执行 `Stopwatch_Start()`，不调用 `LineTrace_Start...()`。
- 或将电机 PWM 输出临时屏蔽。

若只计时稳定，而一跑车就复位，则优先查电机供电、平滑起步、控制任务和 PWM。

是否作为正式修改：否。该项是临时验证手段，不建议长期保留。

### Change 4：临时关闭 VOFA_DEBUG 做 A/B，但不作为首要修复

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.h`

临时改：

```c
#define USE_VOFA_DEBUG 0
```

目的：验证用户怀疑的 VOFA 发送风险。

预期：

- 如果题目2仍回菜单，则基本排除 VOFA 是题目2根因。
- 如果题目3稳定性改善，则说明 VOFA 对题目3负载有影响。

注意：这不是最可能根因，因为题目2根本不发 VOFA。

### Change 5：临时关闭 ZDT_STEPPER 做 A/B

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.h`

临时改：

```c
#define USE_ZDT_STEPPER 0
```

目的：确认 ZDT 任务/步进硬件是否参与题目2复位。

预期：

- 如果关闭 ZDT 后题目2稳定，进一步检查 ZDT 任务栈、UART0 阻塞、驱动板供电干扰。
- 如果无变化，ZDT 不是主要根因。

是否作为正式修改：先作为验证项，除非确定题目2不需要 ZDT 时才考虑默认关闭。

### Change 6：降低 OLED 刷新频率作为辅助验证

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.c`

当前 OLED 任务约 20ms 刷新：

```c
vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
```

调试建议改为 50ms 或 100ms。

目的：降低 OLED 全屏刷新负载，排除显示刷新造成调度/栈压力。

优先级：低于 hook 诊断和平滑起步。

### Change 7：检查任务创建返回值和栈水位

文件：`d:\26diansaidisanti\empty_LP_MSPM0G3507_nortos_ticlang(6)(1) (2)\user\rtos_tasks.c`

建议：

- 对 `OLED_Task`、`GrayTask`、`ZDT_Test` 等任务创建结果做检查。
- 使用已有 `cpu_monitor_task()` 的 `task_usage_table` 观察：
  - `OLED_Task`
  - `ZDT_Test`
  - `Ctrl_Task`
  - `GrayTask`
  - `KeyScan`

原因：

- 当前 FreeRTOS 配置支持 `uxTaskGetStackHighWaterMark()`。
- 若某任务栈水位接近 0，可直接定位栈不足。

---

## Recommended Execution Order

### 第一轮：确认是否复位/栈溢出

1. 修改 `empty.c` 的 malloc failed / stack overflow hook，记录故障码和任务名，调试阶段不要直接复位。
2. 复现进入题目2。
3. 如果停在故障态，读取：
   - `g_fault_code`
   - `g_fault_task_name`

判断：

- `g_fault_code = 1`：malloc failed。
- `g_fault_code = 2`：某任务栈溢出，任务名由 `g_fault_task_name` 给出。

### 第二轮：验证电机启动冲击

1. 将题目2入口改为 `LineTrace_StartSmoothWithStep(&g_line_trace_q2_config, 0.5f)`。
2. 复测题目2。
3. 如果稳定，说明普通起步速度突变/电机启动冲击是主要问题。

### 第三轮：A/B 排除模块影响

按顺序临时验证：

1. `USE_VOFA_DEBUG = 0`
2. `USE_ZDT_STEPPER = 0`
3. OLED 刷新周期从 20ms 降到 50/100ms

每次只改一个变量，记录题目2是否还会回菜单。

### 第四轮：根据结果定正式修复

- 若是电机冲击：保留题目2平滑起步，并检查供电。
- 若是任务栈溢出：增加对应任务栈，或减少其局部大缓冲/打印。
- 若是 malloc failed：减少任务栈、关闭不必要任务或增大 FreeRTOS heap。
- 若是 ZDT 干扰：题目2运行时停止 ZDT 周期回中，或按题目3需要再启用。
- 若是 OLED 负载：降低刷新频率。

---

## Assumptions & Decisions

1. 用户看到“回到题目选择界面”大概率是系统复位，而不是普通软件回菜单。
2. 题目2不会触发 VOFA 9 通道发送，所以 VOFA 不是题目2首要嫌疑。
3. 选择1能正常计时，说明 OLED/秒表单独工作大概率正常。
4. 题目2相比选择1多出的核心风险是底盘电机启动和循迹控制。
5. 优先做可观测性，再做修复；否则会被 `NVIC_SystemReset()` 掩盖根因。
6. 所有 A/B 验证都应一次只改一个变量。

---

## Verification Steps

### 1. 判断是否复位

进入题目2前后观察：

- `g_question_ui_state`
- `g_selected_question`
- `g_stopwatch_elapsed_cs`
- `debug_free_heap`
- `task_usage_table`

如果进入题目2后这些变量恢复初始值，基本就是系统复位。

### 2. 判断是否栈溢出/malloc failed

调试 hook 修改后观察：

- `g_fault_code`
- `g_fault_task_name`

如果故障码有效，直接按任务名修。

### 3. 判断是否电机启动冲击

题目2改平滑起步后测试：

- 若不再回菜单：保留平滑起步，并检查电源/电机驱动抗干扰。
- 若仍回菜单：继续查栈、ZDT、OLED、heap。

### 4. 判断 VOFA 是否相关

临时 `USE_VOFA_DEBUG = 0`：

- 题目2仍复位：排除 VOFA 是题目2根因。
- 题目3改善：VOFA 对题目3有负载影响，后续可进一步降频或减少字段。

### 5. 判断 ZDT 是否相关

临时 `USE_ZDT_STEPPER = 0`：

- 题目2稳定：ZDT 任务/硬件干扰参与问题。
- 题目2仍复位：ZDT 不是主要根因。

### 6. 判断 OLED 负载是否相关

降低 OLED 刷新周期到 50/100ms：

- 若稳定：OLED 刷新负载或 I2C 时序压力参与问题。
- 若无变化：OLED 只是显示了复位后的菜单，不是根因。

---

## Final Recommendation

建议优先实施两项：

1. **修改故障 hook，停止直接无记录复位。**这是确认根因的关键。
2. **题目2改为平滑起步。**这是最小侵入且最可能缓解电机启动导致复位的改动。

VOFA 9 通道发送需要保留关注，但按当前条件判断，它不会在题目2运行时发送，因此不是这次题目2回菜单的首要怀疑对象。