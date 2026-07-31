#include "alldata.h"

// 定义 50ms 周期内的满速理论最大脉冲数
#define MAX_PULSE_50MS  185

#if USE_IMU_SENSOR
static TaskHandle_t IMU_Task_Handle = NULL;
#endif
#if USE_ULTRASONIC
TaskHandle_t Sonar_Task_Handle = NULL;
extern float global_distance_cm;
#endif

volatile float roll = 0.0f;
volatile float pitch = 0.0f;
volatile float yaw = 0.0f;


volatile int16_t speed_L = 0;
volatile int16_t speed_R = 0;
/* =========================================================================
 * 底盘控制参数与状态
 * ========================================================================= */
volatile int32_t target_pwm_L = 0; 
volatile int32_t target_pwm_R = 0; 

// 策略层输入接口：整体前进速度 和 旋转差速
volatile float target_speed = 0.0f;
volatile float target_turn = 0.0f;  // 0为直线，正数右转，负数左转

// 相对 yaw 角度转向状态
volatile uint8_t g_yaw_turn_active = 0;
volatile float g_yaw_target = 0.0f;

// 相对位置运动状态
volatile uint8_t g_pos_move_active = 0;
volatile float g_pos_target_pulse = 0.0f;
volatile float g_pos_current_pulse = 0.0f;
static uint8_t g_pos_smooth_start = 0;
static float g_pos_speed_filter = 0.0f;
static float g_pos_speed_max = 15.0f;
static uint8_t g_pos_line_trace_assist = 0;
static uint8_t g_pos_stopwatch_active = 0;

// 题目菜单状态: 0=菜单, 1=题目运行/计时
static volatile uint8_t g_question_ui_state = 0;
static volatile uint8_t g_selected_question = 2;

// 控制模式: 0=蓝牙遥控, 1=循迹模式 (菜单确定后切换)
volatile uint8_t g_ctrl_mode = 0;

// OLED 秒表状态
static volatile uint8_t g_stopwatch_running = 0;
static volatile uint32_t g_stopwatch_elapsed_cs = 0;
static volatile TickType_t g_stopwatch_start_tick = 0;
static volatile uint8_t g_stop_line_ignore_cycles = 0;

// 默认循迹参数 (直线 + 弧线 + 停车线)
#define LINE_DEFAULT_STRAIGHT_SPEED  14.0f
#define LINE_DEFAULT_TURN_SPEED      10.0f
#define LINE_DEFAULT_TURN_IN_ERROR   50
#define LINE_DEFAULT_TURN_OUT_ERROR  25
#define LINE_DEFAULT_TURN_DEADBAND   0.0f
#define LINE_DEFAULT_KP              0.12f
#define LINE_DEFAULT_TURN_KP_SCALE   0.40f
#define LINE_TURN_IN_CONFIRM        2
#define LINE_TURN_OUT_CONFIRM       4
#define LINE_SPEED_STEP              1.0f
#define LINE_CURVE_ERROR_HOLD        120
#define LINE_DEFAULT_STOP_BLACK_MIN  4
#define LINE_DEFAULT_STOP_CONFIRM    1
#define LINE_DEFAULT_IGNORE_CYCLES   20

// S4 按键参数: 当前默认循迹参数
// straight_speed=16.0, turn_speed=12.0, turn_in_error=50, turn_out_error=25,
// turn_deadband=0.0, kp=0.12, stop_black_min=4, stop_confirm=1, ignore_cycles=20
static const LineTrace_Config_t g_line_trace_s4_config = {
    16.0f,
    12.0f,
    LINE_DEFAULT_TURN_IN_ERROR,
    LINE_DEFAULT_TURN_OUT_ERROR,
    LINE_DEFAULT_TURN_DEADBAND,
    LINE_DEFAULT_KP,
    LINE_DEFAULT_STOP_BLACK_MIN,
    LINE_DEFAULT_STOP_CONFIRM,
    LINE_DEFAULT_IGNORE_CYCLES,
};

// 题目2参数: 独立于题目5/6, 初始复制原S4参数
static const LineTrace_Config_t g_line_trace_q2_config = {
    16.0f,
    12.0f,
    LINE_DEFAULT_TURN_IN_ERROR,
    LINE_DEFAULT_TURN_OUT_ERROR,
    LINE_DEFAULT_TURN_DEADBAND,
    LINE_DEFAULT_KP,
    LINE_DEFAULT_STOP_BLACK_MIN,
    LINE_DEFAULT_STOP_CONFIRM,
    LINE_DEFAULT_IGNORE_CYCLES,
};

// 题目5/6参数: 两道题共用, 但独立于题目2和原S4配置
static const LineTrace_Config_t g_line_trace_q56_config = {
    10.0f,
    10.0f,
    LINE_DEFAULT_TURN_IN_ERROR,
    LINE_DEFAULT_TURN_OUT_ERROR,
    LINE_DEFAULT_TURN_DEADBAND,
    LINE_DEFAULT_KP,
    LINE_DEFAULT_STOP_BLACK_MIN,
    LINE_DEFAULT_STOP_CONFIRM,
    LINE_DEFAULT_IGNORE_CYCLES,
};

// S2 按键参数: 位置环直行时用于循迹纠偏，不决定终点距离
// straight_speed=14.0, turn_speed=10.0, turn_in_error=50, turn_out_error=25,
// turn_deadband=0.0, kp=0.12, stop_black_min=4, stop_confirm=1, ignore_cycles=20
static const LineTrace_Config_t g_line_trace_s2_config = {
    LINE_DEFAULT_STRAIGHT_SPEED,
    LINE_DEFAULT_TURN_SPEED,
    LINE_DEFAULT_TURN_IN_ERROR,
    LINE_DEFAULT_TURN_OUT_ERROR,
    LINE_DEFAULT_TURN_DEADBAND,
    LINE_DEFAULT_KP,
    LINE_DEFAULT_STOP_BLACK_MIN,
    LINE_DEFAULT_STOP_CONFIRM,
    LINE_DEFAULT_IGNORE_CYCLES,
};

static LineTrace_Config_t g_line_trace_config = {
    LINE_DEFAULT_STRAIGHT_SPEED,
    LINE_DEFAULT_TURN_SPEED,
    LINE_DEFAULT_TURN_IN_ERROR,
    LINE_DEFAULT_TURN_OUT_ERROR,
    LINE_DEFAULT_TURN_DEADBAND,
    LINE_DEFAULT_KP,
    LINE_DEFAULT_STOP_BLACK_MIN,
    LINE_DEFAULT_STOP_CONFIRM,
    LINE_DEFAULT_IGNORE_CYCLES,
};
static volatile uint8_t g_line_trace_running = 0;
static volatile uint8_t g_line_trace_stop_line_count = 0;
static float g_line_trace_error_filter = 0.0f;
static float g_line_trace_speed_filter = 0.0f;
static float g_line_trace_smooth_step = LINE_SPEED_STEP;
static uint8_t g_line_trace_smooth_start = 0;
static int32_t g_line_trace_curve_error = 0;
static uint8_t g_line_trace_turn_mode = 0;
static uint8_t g_line_trace_turn_in_count = 0;
static uint8_t g_line_trace_turn_out_count = 0;

// 摆杆执行器默认参数，先保守限制行程，后续按机构标定调整
// MIN/MAX_PULSE: 摆杆允许的最小/最大绝对目标位置，单位是电机脉冲；数值越大，摆杆最大倾角越大。
// MAX_STEP: 每次调用 RodActuator_SetTargetPulse() 时最多变化的脉冲数；越小越柔和，越大响应越快。
// CMD_TO_PULSE: RodActuator_SetTargetCmd(cmd) 的换算比例；cmd=1.0f 时对应多少脉冲。
// RPM: ZDT 位置模式运动速度；太大容易冲，太小跟随慢。
// ACC: ZDT 加速度档位，范围 0~255；太大换向会猛，太小启动和制动会慢。
#define ROD_DEFAULT_MIN_PULSE          (-180)   // 提高摆幅上限，让 out 不再过早顶到 ±120；若再次发散再降回 ±120
#define ROD_DEFAULT_MAX_PULSE          (180)    // 提高摆幅上限，让 out 不再过早顶到 ±120；若再次发散再降回 ±120
#define ROD_DEFAULT_MAX_STEP           45       // 提高每 50ms 最大变化量，加快响应；若输出边沿太硬再降到 30
#define ROD_DEFAULT_CMD_TO_PULSE       300.0f   // 后续钢珠控制输出 -1.0~+1.0 时，对应 -500~+500 脉冲
#define ROD_DEFAULT_RPM                220      // 提高位置模式速度；若换向太猛或过冲明显再降到 180
#define ROD_DEFAULT_ACC                50       // 提高加速度档位；若换向太猛或过冲明显再降到 40

// ================= Question 3 钢球视觉闭环参数 =================
// 当前视觉模块输出的是 x_offset，单位是像素；进入控制前会先用 Q3_VISION_POS_SIGN 统一成物理坐标。
// 调试顺序建议：先确认 Q3_VISION_POS_SIGN，再标定 Q3_TARGET_5CM_PX，再确认 Q3_KP_PULSE_PER_PX 正负号，最后调 Kp/Kd。

// 视觉坐标到物理坐标的方向系数。
// 如果钢球在物理 -5cm 时 g_vision_x_offset 为正，说明视觉方向与物理方向相反，应设为 -1.0f。
// 如果钢球在物理 +5cm 时 g_vision_x_offset 为正，应设为 +1.0f。
#define Q3_VISION_POS_SIGN             (1.0f)

// 软件零位补偿，单位像素。
// 当前现象：补偿 -313px 后，小球真实在零位时 ball_pos_px 显示为 -330。
// 因此往正方向补偿 330px：-313 - (-330) = +17。
#define Q3_ZERO_BIAS_PX                17.0f

// 实际 +5cm 对应的视觉像素偏差。必须上板标定：把球放在 +5cm 或 -5cm 标记处，读取 g_vision_x_offset 的绝对值。
// 数值偏小：目标实际距离不到 5cm；数值偏大：目标会超过 5cm。
#define Q3_TARGET_5CM_PX               500.0f

// 目标点每 50ms 移动多少像素。调大：目标移动更快，球更容易冲；调小：更稳但完成更慢。
// 例：1.0f 表示约 20px/s。如果球一启动就冲，先降到 0.5f。
#define Q3_TARGET_STEP_PX              3.0f

// 位置比例项：rod_cmd = Kp * (target_pos - ball_pos)。绝对值越大，摆杆拉球越用力。
// 正负号决定摆杆方向；如果球越控越远，先把这个值从 -2.0f 改成 +2.0f。
// 绝对值太小：球回不来；绝对值太大：球快速冲过目标、来回振荡。
#define Q3_KP_PULSE_PER_PX             (-0.8f)

// 速度阻尼项：rod_cmd 里会减去 Kd * ball_vel，用来提前刹车。
// 调大：过冲减少，但响应变钝；调小：动作更快，但容易冲过头。
// 如果球能到目标但总是越过目标，优先加大这个值。
#define Q3_KD_PULSE_PER_PX             (3.0f)

// 视觉位置低通滤波系数，范围 0~1。调大：跟随更快但更抖；调小：更稳但延迟更大。
// 你图里零位附近有跳变，先用 0.25f；如果仍抖，降到 0.15f。
#define Q3_POS_FILTER_ALPHA            0.35f

// 零位死区，单位像素。处理后的球位置绝对值小于该值时，直接认为在零位。
// 你图里零位附近大约在 45~50 之间跳，做零位偏置校准后，小抖动可先用 8px 死区吃掉。
#define Q3_ZERO_DEADBAND_PX            4.0f

// 速度滤波系数，范围 0~1。调大：速度估计更灵敏但更抖；调小：更平滑但延迟更大。
// 视觉数据抖动明显时减小，例如 0.2f；反应慢时增大，例如 0.5f。
#define Q3_VEL_FILTER_ALPHA            0.25f

// 到达目标的位置容差，单位像素。调大：更容易判定到达；调小：要求更准但可能一直判不到。
#define Q3_REACH_POS_TOL_PX            4.0f

// 到达目标的速度容差，单位为每控制周期的像素变化量。调大：更容易判定稳定；调小：必须更慢才算稳定。
#define Q3_REACH_VEL_TOL_PX            1.0f

// 连续多少个 50ms 周期都满足位置和速度容差，才认为到达。10U 约等于 0.5s。
#define Q3_REACH_CONFIRM_COUNT         10U

// 在 +5cm 到达后保持多少个 50ms 周期，再折返去 -5cm。20U 约等于 1s。
#define Q3_HOLD_POSITIVE_CYCLES        20U

// 连续多少个 50ms 周期没有收到视觉新数据，就认为视觉丢失并回中。10U 约等于 0.5s。
#define Q3_VISION_LOST_CYCLES          10U


static const RodActuator_Config_t g_rod_default_config = {
    ROD_DEFAULT_MIN_PULSE,
    ROD_DEFAULT_MAX_PULSE,
    ROD_DEFAULT_MAX_STEP,
    ROD_DEFAULT_CMD_TO_PULSE,
    ROD_DEFAULT_RPM,
    ROD_DEFAULT_ACC,
};

static RodActuator_Config_t g_rod_config = {
    ROD_DEFAULT_MIN_PULSE,
    ROD_DEFAULT_MAX_PULSE,
    ROD_DEFAULT_MAX_STEP,
    ROD_DEFAULT_CMD_TO_PULSE,
    ROD_DEFAULT_RPM,
    ROD_DEFAULT_ACC,
};
static int32_t g_rod_target_pulse = 0;
static uint8_t g_rod_ready = 0;

// yaw 相对转向参数
#define YAW_TURN_KP          0.8f
#define YAW_TURN_MAX         24.0f
#define YAW_TURN_MIN         8.0f
#define YAW_TURN_TOLERANCE   1.0f

// 位置环参数：PULSE_PER_CM 需要按实车标定
#define POSITION_PULSE_PER_CM      15.4f
#define POSITION_KP                0.12f
#define POSITION_SPEED_MAX         15.0f
#define POSITION_SPEED_MIN         5.0f
#define POSITION_SMOOTH_STEP       0.20f
#define POSITION_S2_DISTANCE_CM    165.0f
#define POSITION_S2_SPEED_MAX      11.0f
#define POSITION_TOLERANCE_PULSE   8.0f

// 控制目标互斥锁 (保护 target_speed / target_turn 的并发访问)
// 使用静态分配避免堆压力 — 25KB 堆已接近极限
static SemaphoreHandle_t target_ctrl_Mutex = NULL;


// 底盘执行层：速度环 和 转向环
// 注意: 这两个变量仅 control_test_task (pri 2) 独占访问,
// 无 ISR 或其他任务并发, 不需要 volatile.
PID_t pid_speed;
PID_t pid_turn;

/* =========================================================================
 * 跨任务状态与调试数据
 * ========================================================================= */
volatile uint8_t g_bt_beep_cmd = 0; // 0:静音, 1:短鸣(普通指令), 2:长鸣(急停)

// CPU 监控相关变量

typedef struct {
    char name[16];
    float cpu_usage;
    uint32_t min_stack_free; 
} TaskUsage_t;

// 定义一个数组，存放所有任务的占用率
#define MAX_TRACKED_TASKS 24
volatile TaskUsage_t task_usage_table[MAX_TRACKED_TASKS];

/* =========================================================================
 * VOFA JustFloat 调试全局变量
 *
 * 注意: VOFA_HandleTypeDef 包含 rx_ring_buf[512] + frame_buf[256] ≈ 800B,
 *       必须分配在全局区 (BSS), 绝不能放在任务栈上!
 *       vofa_types 同样放全局区, 减轻任务栈压力.
 * ========================================================================= */
static VOFA_HandleTypeDef hvofa;
static VOFA_Data_TypeDef_Enum vofa_types[9];

/* =========================================================================
 * 本文件内部辅助函数
 * ========================================================================= */
void LineTrace_Stop(void);

static void Chassis_ClearOuterLoops(void)
{
    g_yaw_turn_active = 0;
    g_pos_move_active = 0;
}

static void Chassis_ClearTarget(void)
{
    target_speed = 0.0f;
    target_turn = 0.0f;
}

static void Chassis_ClearPidIntegral(void)
{
    pid_speed.ErrorInt = 0.0f;
    pid_turn.ErrorInt = 0.0f;
}

static void Stopwatch_Start(void)
{
    g_stopwatch_elapsed_cs = 0;
    g_stopwatch_start_tick = xTaskGetTickCount();
    g_stopwatch_running = 1;
}

static void Stopwatch_Stop(void)
{
    TickType_t now = xTaskGetTickCount();

    if (g_stopwatch_running) {
        g_stopwatch_elapsed_cs += (uint32_t)(((now - g_stopwatch_start_tick) * 100U) / configTICK_RATE_HZ);
        g_stopwatch_running = 0;
    }
}

static uint8_t CountBits8(uint8_t value)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (value & (1U << i)) count++;
    }
    return count;
}

static void LineTrace_ResetState(void)
{
    g_line_trace_stop_line_count = 0;
    g_gray_stop_line_latched = 0;
    g_line_trace_error_filter = 0.0f;
    g_line_trace_speed_filter = 0.0f;
    g_line_trace_smooth_step = LINE_SPEED_STEP;
    g_line_trace_smooth_start = 0;
    g_line_trace_curve_error = 0;
    g_line_trace_turn_mode = 0;
    g_line_trace_turn_in_count = 0;
    g_line_trace_turn_out_count = 0;
}

static void LineTrace_StartInternal(const LineTrace_Config_t *config, uint8_t smooth_start)
{
    if (target_ctrl_Mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(target_ctrl_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_line_trace_config = (config != NULL) ? *config : g_line_trace_s4_config;
        g_ctrl_mode = 1;
        g_line_trace_running = 1;
        g_stop_line_ignore_cycles = g_line_trace_config.stop_line_ignore_cycles;
        LineTrace_ResetState();
        g_line_trace_smooth_start = smooth_start;
        Chassis_ClearTarget();
        Chassis_ClearOuterLoops();
        Chassis_ClearPidIntegral();
        xSemaphoreGive(target_ctrl_Mutex);
    }

    Stopwatch_Start();
    g_bt_beep_cmd = 1;
}

void LineTrace_Start(const LineTrace_Config_t *config)
{
    LineTrace_StartInternal(config, 0);
}

static void LineTrace_StartSmooth(const LineTrace_Config_t *config)
{
    LineTrace_StartInternal(config, 1);
}

static void LineTrace_StartSmoothWithStep(const LineTrace_Config_t *config, float smooth_step)
{
    LineTrace_StartInternal(config, 1);
    g_line_trace_smooth_step = smooth_step;
}

void LineTrace_Stop(void)
{
    if (target_ctrl_Mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(target_ctrl_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_ctrl_mode = 0;
        g_line_trace_running = 0;
        LineTrace_ResetState();
        Chassis_ClearTarget();
        Chassis_ClearOuterLoops();
        Chassis_ClearPidIntegral();
        xSemaphoreGive(target_ctrl_Mutex);
    }

    Stopwatch_Stop();
}

static void Chassis_StopAll(void)
{
    LineTrace_Stop();

    if (target_ctrl_Mutex != NULL && xSemaphoreTake(target_ctrl_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_pos_move_active = 0;
        g_pos_line_trace_assist = 0;
        g_pos_stopwatch_active = 0;
        g_ctrl_mode = 0;
        Chassis_ClearTarget();
        Chassis_ClearOuterLoops();
        Chassis_ClearPidIntegral();
        xSemaphoreGive(target_ctrl_Mutex);
    }

    Stopwatch_Stop();
}

uint8_t LineTrace_IsRunning(void)
{
    return g_line_trace_running;
}

static int32_t RodActuator_LimitPulse(int32_t target_pulse)
{
    if (target_pulse > g_rod_config.max_pulse) {
        target_pulse = g_rod_config.max_pulse;
    }
    if (target_pulse < g_rod_config.min_pulse) {
        target_pulse = g_rod_config.min_pulse;
    }

    return target_pulse;
}

static int32_t RodActuator_LimitStep(int32_t target_pulse)
{
    int32_t delta = target_pulse - g_rod_target_pulse;

    if (delta > g_rod_config.max_step_per_update) {
        target_pulse = g_rod_target_pulse + g_rod_config.max_step_per_update;
    } else if (delta < -g_rod_config.max_step_per_update) {
        target_pulse = g_rod_target_pulse - g_rod_config.max_step_per_update;
    }

    return target_pulse;
}

static bool RodActuator_SendAbsolute(int32_t target_pulse)
{
    ZDT_Dir_t dir = ZDT_DIR_CW;
    uint32_t pulses = 0;

    if (target_pulse < 0) {
        dir = ZDT_DIR_CCW;
        pulses = (uint32_t)(-target_pulse);
    } else {
        dir = ZDT_DIR_CW;
        pulses = (uint32_t)target_pulse;
    }

    return ZDT_MoveAbsolute(ZDT_DEFAULT_ADDR, dir, g_rod_config.rpm, g_rod_config.acc, pulses);
}

void RodActuator_Init(const RodActuator_Config_t *config)
{
    g_rod_config = (config != NULL) ? *config : g_rod_default_config;

    if (g_rod_config.min_pulse > g_rod_config.max_pulse) {
        int32_t temp = g_rod_config.min_pulse;
        g_rod_config.min_pulse = g_rod_config.max_pulse;
        g_rod_config.max_pulse = temp;
    }
    if (g_rod_config.max_step_per_update <= 0) {
        g_rod_config.max_step_per_update = ROD_DEFAULT_MAX_STEP;
    }
    if (g_rod_config.rpm == 0U) {
        g_rod_config.rpm = ROD_DEFAULT_RPM;
    }

    g_rod_target_pulse = 0;
    g_rod_ready = 1;
}

void RodActuator_SetCenter(void)
{
    ZDT_SetCurrentPositionAsZero(ZDT_DEFAULT_ADDR);
    g_rod_target_pulse = 0;
    g_rod_ready = 1;
}

bool RodActuator_SetTargetPulse(int32_t target_pulse)
{
    if (!g_rod_ready) {
        return false;
    }

    target_pulse = RodActuator_LimitPulse(target_pulse);
    target_pulse = RodActuator_LimitStep(target_pulse);

    if (RodActuator_SendAbsolute(target_pulse)) {
        g_rod_target_pulse = target_pulse;
        return true;
    }

    return false;
}

bool RodActuator_SetTargetCmd(float cmd)
{
    int32_t target_pulse = (int32_t)(cmd * g_rod_config.cmd_to_pulse);
    return RodActuator_SetTargetPulse(target_pulse);
}

bool RodActuator_ReturnCenter(void)
{
    return RodActuator_SetTargetPulse(0);
}

int32_t RodActuator_GetTargetPulse(void)
{
    return g_rod_target_pulse;
}

uint8_t RodActuator_IsReady(void)
{
    return g_rod_ready;
}

static float LineTrace_CalcTurn(void)
{
    int32_t line_error = g_gray_error;
    g_line_trace_error_filter = g_line_trace_error_filter * 0.2f + (float)line_error * 0.8f;
    int32_t abs_line_error = (line_error >= 0) ? line_error : -line_error;

    if (abs_line_error > g_line_trace_config.turn_in_error) {
        if (g_line_trace_turn_in_count < LINE_TURN_IN_CONFIRM) {
            g_line_trace_turn_in_count++;
        }
        g_line_trace_turn_out_count = 0;
    } else if (abs_line_error < g_line_trace_config.turn_out_error) {
        if (g_line_trace_turn_out_count < LINE_TURN_OUT_CONFIRM) {
            g_line_trace_turn_out_count++;
        }
        g_line_trace_turn_in_count = 0;
    } else {
        g_line_trace_turn_in_count = 0;
        g_line_trace_turn_out_count = 0;
    }

    if (g_line_trace_turn_in_count >= LINE_TURN_IN_CONFIRM) {
        g_line_trace_turn_mode = 1;
    } else if (g_line_trace_turn_out_count >= LINE_TURN_OUT_CONFIRM) {
        g_line_trace_turn_mode = 0;
    }

    float control_error = g_line_trace_error_filter;
    if (g_line_trace_turn_mode) {
        int32_t curve_error = (int32_t)g_line_trace_error_filter;
        int32_t curve_delta = curve_error - g_line_trace_curve_error;
        if (g_line_trace_curve_error != 0 &&
            ((curve_error > 0 && g_line_trace_curve_error > 0) || (curve_error < 0 && g_line_trace_curve_error < 0)) &&
            curve_delta < LINE_CURVE_ERROR_HOLD && curve_delta > -LINE_CURVE_ERROR_HOLD) {
            control_error = (float)g_line_trace_curve_error;
        } else {
            g_line_trace_curve_error = curve_error;
        }
    } else {
        g_line_trace_curve_error = 0;
    }

    float turn_error = 0.0f;
    if (control_error > g_line_trace_config.turn_deadband) {
        turn_error = control_error - g_line_trace_config.turn_deadband;
    } else if (control_error < -g_line_trace_config.turn_deadband) {
        turn_error = control_error + g_line_trace_config.turn_deadband;
    }

    float line_kp = g_line_trace_turn_mode ? (g_line_trace_config.kp * LINE_DEFAULT_TURN_KP_SCALE) : g_line_trace_config.kp;
    float line_turn = -turn_error * line_kp;
    if (line_turn > 100.0f) line_turn = 100.0f;
    if (line_turn < -100.0f) line_turn = -100.0f;
    return line_turn;
}

static void LineTrace_Update(void)
{
    uint8_t line_raw = g_gray_raw_data;
    uint8_t black_count = CountBits8(line_raw);
    uint8_t side_stop_line = ((line_raw & 0x07U) == 0x07U) || ((line_raw & 0xE0U) == 0xE0U);
    uint8_t stop_line_detected = (black_count >= g_line_trace_config.stop_line_black_min) ||
                                 side_stop_line ||
                                 g_gray_stop_line_latched;

    if (g_stop_line_ignore_cycles > 0) {
        g_stop_line_ignore_cycles--;
        g_line_trace_stop_line_count = 0;
        g_gray_stop_line_latched = 0;
    } else if (stop_line_detected) {
        g_gray_stop_line_latched = 0;
        if (g_line_trace_stop_line_count < g_line_trace_config.stop_line_confirm) {
            g_line_trace_stop_line_count++;
        }
    } else {
        g_line_trace_stop_line_count = 0;
    }

    if (g_line_trace_stop_line_count >= g_line_trace_config.stop_line_confirm) {
        LineTrace_Stop();
        g_gray_stop_line_latched = 0;
        g_bt_beep_cmd = 2;
        return;
    }

    float target_line_speed = g_line_trace_turn_mode ? g_line_trace_config.turn_speed : g_line_trace_config.straight_speed;
    if (g_line_trace_speed_filter == 0.0f && !g_line_trace_smooth_start) {
        g_line_trace_speed_filter = target_line_speed;
    } else if (g_line_trace_speed_filter < target_line_speed) {
        g_line_trace_speed_filter += g_line_trace_smooth_step;
        if (g_line_trace_speed_filter >= target_line_speed) {
            g_line_trace_speed_filter = target_line_speed;
            g_line_trace_smooth_start = 0;
        }
    } else if (g_line_trace_speed_filter > target_line_speed) {
        g_line_trace_speed_filter -= g_line_trace_smooth_step;
        if (g_line_trace_speed_filter < target_line_speed) g_line_trace_speed_filter = target_line_speed;
    } else {
        g_line_trace_smooth_start = 0;
    }

    float line_speed = g_line_trace_speed_filter;
    float line_turn = LineTrace_CalcTurn();
    if (xSemaphoreTake(target_ctrl_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        target_speed = line_speed;
        target_turn = line_turn;
        xSemaphoreGive(target_ctrl_Mutex);
    }
}

static void Chassis_LimitTarget(void)
{
    if (target_speed > 30.0f) target_speed = 30.0f;
    if (target_speed < -30.0f) target_speed = -30.0f;
    if (target_turn > 24.0f) target_turn = 24.0f;
    if (target_turn < -24.0f) target_turn = -24.0f;

    if (target_speed < 5.0f && target_speed > -5.0f) {
        target_speed = 0.0f;
    }
    if (target_turn < 9.9f && target_turn > -9.9f) {
        target_turn = 0.0f;
    }
}

/* =========================================================================
 * 任务函数实现区
 * ========================================================================= */

/* =========================================================================
 * IMU 姿态解算任务 (Test 工程移植版)
 *
 * 功能: LSM6DSR 驱动 + MEMS 热稳定 + P-P 校验零偏校准 + 静止锁定 +
 *       动态 dt Mahony 融合 → roll/pitch/yaw
 *
 * 采样触发: INT2 (PB0) 中断 → ISR 通知 → 本任务唤醒
 * 任务优先级: 3 (最高, 与 BT_Task 同级)
 * 任务栈: 768 words (3072 bytes)
 * ========================================================================= */

/* ---------- dt 测量定时器宏 (用户按 SysConfig 配置修改) ---------- */
/*
 * 用户需在 SysConfig 中配置一个空闲 GPTimer (推荐 TIMG7):
 *   名称: IMU_dt
 *   模式: PERIODIC_UP (自由运行计数器, 不产生中断)
 *   时钟源: BUSCLK
 *   分频: /8, 预分频: 99  →  计数频率 = BUSCLK/(8*100) = 50 kHz (80MHz PLL)
 *   周期: 49999 (1 秒回绕)
 */
// IMU_dt_INST 已由 SysConfig 在 ti_msp_dl_config.h 中定义, 此处不再重复
#define IMU_DT_FREQ_HZ     50000.0f    /* 计数器频率 (Hz)                   */
#define IMU_DT_WRAP_VALUE  50000       /* 计数器回绕值 (period + 1)         */

/* ---------- 校准参数 ---------- */
#define CALIB_SOAK_SECONDS       3
#define CALIB_WARMUP             50      /* 预热丢弃帧数 (~0.5s)            */
#define CALIB_SAMPLES            300     /* 采集帧数 @ 104Hz ≈ 2.88s       */
#define CALIB_PP_THRESHOLD_DPS   1.0f    /* P-P 静止阈值 (dps)             */
#define CALIB_MAX_RETRIES        5       /* 最大重试次数                   */

/* ---------- 静止锁定参数 ---------- */
#define STATIC_GYRO_THRESHOLD_DPS    0.30f   /* 陀螺静止阈值 (dps)         */
#define STATIC_ACCEL_THRESHOLD_G     0.02f   /* 加速度偏离阈值 (g)          */
#define STATIC_CONFIRM_FRAMES        10      /* 连续静止帧 → 触发锁定       */
#define STATIC_RELEASE_FRAMES        5       /* 连续运动帧 → 解除锁定       */

/* ---------- Debug 观测变量 (CCS Expressions 窗口) ---------- */
volatile float   debug_gyro_dps[3];
volatile float   debug_accel_g[3];
volatile float   debug_euler_deg[3];
volatile int32_t debug_loop_count;
volatile int     debug_static_state = 0;   /* 0=unlocked, 1=locked          */
volatile float   debug_dt_ms        = 0.0f;/* 实际采样间隔 (ms)             */
volatile int     lsm6dsr_fault      = 0;   /* 0=healthy, 1=WHO_AM_I mismatch */

/* ---- 堆内存监控 (CCS Expressions 可见) ---- */
volatile uint32_t debug_free_heap    = 0;   /* xPortGetFreeHeapSize() 每秒更新 */


/* =========================================================================
 * 陀螺零偏校准 (P-P 校验)
 *
 * 流程:
 *   1. 预热: 丢弃 CALIB_WARMUP 帧, 等待传感器机械稳定
 *   2. 采集: CALIB_SAMPLES 帧累加求和, 同时追踪每轴 min/max
 *   3. 校验: 若任一轴 P-P 超过阈值 → 板子被触碰, 丢弃本轮, 重试
 *   4. 重试: 最多 CALIB_MAX_RETRIES 次, 全部失败则降级使用最后一次结果
 *
 * 每帧用 INT1 通知驱动 (阻塞在 ulTaskNotifyTake, 不占 CPU)
 * 超时保护: 20ms 内无 INT1 则跳过本帧, 防止垃圾数据污染累加器
 * ========================================================================= */
#if USE_IMU_SENSOR
static int GyroBias_Calibrate(float *bx, float *by, float *bz) {
    int16_t accel[3], gyro[3];

    for (int attempt = 1; attempt <= CALIB_MAX_RETRIES; attempt++) {
        /* ---- 预热阶段: 丢弃数据, 等待传感器稳定 ---- */
        for (int i = 0; i < CALIB_WARMUP; i++) {
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)) == 0) {
                i--;  /* 超时: INT1 断流, 跳过本帧重试 */
                continue;
            }
            LSM6DSR_Read_RawData(accel, gyro);
        }

        /* ---- 采集阶段: 累加 + 追踪 P-P ---- */
        int32_t sum_x = 0, sum_y = 0, sum_z = 0;
        int16_t min_x =  32767, max_x = -32768;
        int16_t min_y =  32767, max_y = -32768;
        int16_t min_z =  32767, max_z = -32768;

        for (int i = 0; i < CALIB_SAMPLES; i++) {
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)) == 0) {
                i--;  /* 超时保护 */
                continue;
            }
            LSM6DSR_Read_RawData(accel, gyro);

            int16_t gx = gyro[0], gy = gyro[1], gz = gyro[2];
            sum_x += gx;  sum_y += gy;  sum_z += gz;

            if (gx < min_x) min_x = gx;  if (gx > max_x) max_x = gx;
            if (gy < min_y) min_y = gy;  if (gy > max_y) max_y = gy;
            if (gz < min_z) min_z = gz;  if (gz > max_z) max_z = gz;
        }

        /* ---- 计算 P-P (dps) ---- */
        float pp_x = (float)(max_x - min_x) * (LSM6DSR_GYRO_SENSITIVITY_MDPS / 1000.0f);
        float pp_y = (float)(max_y - min_y) * (LSM6DSR_GYRO_SENSITIVITY_MDPS / 1000.0f);
        float pp_z = (float)(max_z - min_z) * (LSM6DSR_GYRO_SENSITIVITY_MDPS / 1000.0f);
        float pp_max = pp_x;
        if (pp_y > pp_max) pp_max = pp_y;
        if (pp_z > pp_max) pp_max = pp_z;

        /* ---- 计算零偏 (dps) ---- */
        *bx = (float)sum_x / (float)CALIB_SAMPLES * (LSM6DSR_GYRO_SENSITIVITY_MDPS / 1000.0f);
        *by = (float)sum_y / (float)CALIB_SAMPLES * (LSM6DSR_GYRO_SENSITIVITY_MDPS / 1000.0f);
        *bz = (float)sum_z / (float)CALIB_SAMPLES * (LSM6DSR_GYRO_SENSITIVITY_MDPS / 1000.0f);

        /* ---- 静止校验 ---- */
        if (pp_max <= CALIB_PP_THRESHOLD_DPS) {
            return 0;   /* 静止校准成功 */
        }

        /* 触碰/抖动 — 短暂等待后重试 */
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    return 1;   /* 所有重试均失败, 降级使用最后结果 */
}


/* =========================================================================
 * IMU 任务主函数
 * ========================================================================= */
static void imu_task(void *pvParameters) {
    float gx_bias_dps = 0.0f, gy_bias_dps = 0.0f, gz_bias_dps = 0.0f;

    /* ================================================================
     *  Phase 0: 硬件自检
     * ================================================================ */
    LSM6DSR_Init();
    vTaskDelay(pdMS_TO_TICKS(100));   /* 等待传感器稳定 */

    if (LSM6DSR_Check_WhoAmI() != 0x6B) {
        lsm6dsr_fault = 1;
        /* LED 快闪报错, 然后任务自挂起 */
        while (1) {
            DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_Freertos_LED_PIN);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    /* ================================================================
     *  Phase 1: MEMS 热稳定 Soak (3 秒)
     * ================================================================
     *  MEMS 陀螺上电后前几秒存在显著的零偏漂移 (温漂) — 硅微机械结构
     *  在电流加热下逐渐趋向热平衡。在校准前让传感器持续运行 3 秒,
     *  给足够时间达到热稳态。
     * ================================================================ */
    {
        int soak_frames = (int)(LSM6DSR_ODR_HZ * CALIB_SOAK_SECONDS);  /* ~312 帧 */
        int16_t dummy_accel[3], dummy_gyro[3];
        for (int i = 0; i < soak_frames; i++) {
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)) == 0) {
                i--;  /* 超时保护: INT1 断流则重试 */
                continue;
            }
            LSM6DSR_Read_RawData(dummy_accel, dummy_gyro);
        }
    }

    /* ================================================================
     *  Phase 2: 陀螺零偏校准 (P-P 校验)
     * ================================================================ */
    {
        int calib_result = GyroBias_Calibrate(&gx_bias_dps, &gy_bias_dps, &gz_bias_dps);
        (void)calib_result;  /* 0=成功, 1=降级 — 调试时可在 Expressions 观察 */
    }

    /* ================================================================
     *  Phase 3: 启动完成信号
     * ================================================================ */
    DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_0_PIN);  /* 蜂鸣器响 */
    vTaskDelay(pdMS_TO_TICKS(50));
    DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_0_PIN);    /* 停 */
    vTaskDelay(pdMS_TO_TICKS(100));
    DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_0_PIN);  /* 响 */
    vTaskDelay(pdMS_TO_TICKS(50));
    DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_0_PIN);    /* 停 */

    /* ================================================================
     *  Phase 4: 正常运行循环
     * ================================================================ */
    Mahony_Init(LSM6DSR_ODR_HZ);
    Mahony_SetKp(0.5f);
    Mahony_SetKi(0.005f);

    /* --- 静态锁定状态变量 (static, 放 .bss) --- */
    static bool  static_locked     = false;
    static int   static_counter    = 0;
    static int   moving_counter    = 0;
    static float accel_baseline[3] = {0.0f, 0.0f, 0.0f};
    static bool  baseline_ready    = false;
    static float saved_ki          = 0.005f;

    /* --- dt 测量状态 --- */
    DL_TimerG_startCounter(IMU_dt_INST);
    uint32_t last_timer = DL_TimerG_getTimerCount(IMU_dt_INST);
    bool     dt_first   = true;

    /* --- 局部变量 --- */
    float accel_g[3], gyro_dps[3];
    float dt;

    while (1) {
        /* 死等 INT1 中断通知, 没收到信号就不往下跑 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* ---- 传感器读取 + 物理单位换算 ---- */
        LSM6DSR_Read_Scaled(accel_g, gyro_dps);

        /* ---- 陀螺零偏补偿 ---- */
        gyro_dps[0] -= gx_bias_dps;
        gyro_dps[1] -= gy_bias_dps;
        gyro_dps[2] -= gz_bias_dps;

        /* ================================================================
         *  安装方向坐标变换: 传感器坐标系 → 小车坐标系
         *
         *  传感器物理安装: 芯片面朝下, 绕 X 轴翻转 180°.
         *  变换矩阵 R_x(180°):
         *    X_car =  X_sensor       (前进方向, 不变)
         *    Y_car = -Y_sensor       (左右翻转)
         *    Z_car = -Z_sensor       (上下翻转, 重力从 -1g → +1g)
         *
         *  数学性质: det(R) = +1, 是合法旋转矩阵 (非镜像), 不破坏
         *  Mahony 滤波器的正交性假设. 旋转变换与 bias 补偿可交换:
         *    -(gyro_sensor - bias) = (-gyro_sensor) - (-bias) = car_gyro - car_bias
         *  所以先减 bias 再变换 ≡ 先变换再减 car_bias, 无精度损失.
         * ================================================================ */
        accel_g[1] = -accel_g[1];   /* Ay: 传感器 → 小车 */
        accel_g[2] = -accel_g[2];   /* Az: 传感器 → 小车 (重力归正) */
        gyro_dps[1] = -gyro_dps[1]; /* Gy: 传感器 → 小车 */
        gyro_dps[2] = -gyro_dps[2]; /* Gz: 传感器 → 小车 */

        /* ================================================================
         *  静止锁定 (Static Lock): 静止时强制陀螺清零, 阻断噪声积分
         *
         *  检测策略 (双条件 AND):
         *    - 陀螺仪: 三轴角速度绝对值均 < 0.30 dps (静止底噪)
         *    - 加速度计: 三轴偏离低通基线均 < 0.02 g (排除慢速平移)
         *    - 消抖: 连续 10 帧确认静止才锁定, 连续 5 帧检测运动才解锁
         *
         *  基线维护:
         *    - 锁定时以 0.5% 权重持续更新加速度基线 (LPF τ≈2s)
         *    - 解锁时基线冻结, 下次锁定后重新初始化
         * ================================================================ */

        /* 条件 1: 陀螺仪三轴均低于静止阈值 */
        bool gyro_quiet = (fabsf(gyro_dps[0]) < STATIC_GYRO_THRESHOLD_DPS)
                       && (fabsf(gyro_dps[1]) < STATIC_GYRO_THRESHOLD_DPS)
                       && (fabsf(gyro_dps[2]) < STATIC_GYRO_THRESHOLD_DPS);

        /* 条件 2: 加速度计偏离基线 (基线未就绪时跳过加速度检查) */
        bool accel_quiet = true;
        if (baseline_ready) {
            accel_quiet = (fabsf(accel_g[0] - accel_baseline[0]) < STATIC_ACCEL_THRESHOLD_G)
                       && (fabsf(accel_g[1] - accel_baseline[1]) < STATIC_ACCEL_THRESHOLD_G)
                       && (fabsf(accel_g[2] - accel_baseline[2]) < STATIC_ACCEL_THRESHOLD_G);
        }

        if (gyro_quiet && accel_quiet) {
            /* 本帧疑似静止 */
            static_counter++;
            moving_counter = 0;

            if (!static_locked && static_counter >= STATIC_CONFIRM_FRAMES) {
                /* 连续 N 帧静止 → 锁定! */
                static_locked = true;
                /* 初始化加速度基线: 直接用当前值 */
                accel_baseline[0] = accel_g[0];
                accel_baseline[1] = accel_g[1];
                accel_baseline[2] = accel_g[2];
                baseline_ready = true;
                /* 复位 Mahony 积分项: 清除锁前累积的偏置, 否则会继续注入漂移 */
                Mahony_ResetIntegral();
                Mahony_SetKi(0.0f);  /* 锁定期间禁用积分, 仅靠 P 项维持姿态 */
            }
        } else {
            /* 本帧在运动 */
            moving_counter++;
            static_counter = 0;

            if (static_locked && moving_counter >= STATIC_RELEASE_FRAMES) {
                /* 连续 M 帧运动 → 解锁 */
                static_locked = false;
                baseline_ready = false;
                Mahony_SetKi(saved_ki);  /* 恢复积分 */
            }
        }

        /* 锁定状态: 加速度基线持续低通更新, 追踪缓慢的姿态变化 */
        if (static_locked && baseline_ready) {
            const float alpha = 0.005f;  /* LPF 系数, τ ≈ 200 帧 ≈ 2 秒 */
            accel_baseline[0] = (1.0f - alpha) * accel_baseline[0] + alpha * accel_g[0];
            accel_baseline[1] = (1.0f - alpha) * accel_baseline[1] + alpha * accel_g[1];
            accel_baseline[2] = (1.0f - alpha) * accel_baseline[2] + alpha * accel_g[2];
        }

        debug_static_state = static_locked ? 1 : 0;

        /* ---- 动态 dt 测量 ---- */
        {
            uint32_t now = DL_TimerG_getTimerCount(IMU_dt_INST);
            if (dt_first) {
                dt = 1.0f / LSM6DSR_ODR_HZ;   /* 首帧用标称 dt (9.6ms) */
                dt_first = false;
            } else {
                int32_t delta = (int32_t)(now - last_timer);
                if (delta < 0) delta += IMU_DT_WRAP_VALUE;  /* 回绕补偿 */
                dt = (float)delta / IMU_DT_FREQ_HZ;
            }
            last_timer = now;
            debug_dt_ms = dt * 1000.0f;        /* CCS 可观测实际 dt (ms) */
        }

        /* ---- Mahony 融合更新 (dps→rad/s, 锁定时陀螺清零) ---- */
        {
            float gx_rad = (static_locked ? 0.0f : gyro_dps[0]) * 0.0174533f;  /* * π/180 */
            float gy_rad = (static_locked ? 0.0f : gyro_dps[1]) * 0.0174533f;
            float gz_rad = (static_locked ? 0.0f : gyro_dps[2]) * 0.0174533f;

            Mahony_UpdateIMU(gx_rad, gy_rad, gz_rad,
                             accel_g[0], accel_g[1], accel_g[2], dt);
        }

        /* ---- 获取欧拉角 ---- */
        Mahony_GetEulerAngles((float*)&roll, (float*)&pitch, (float*)&yaw);

        /* ---- 更新 Debug 观测变量 (CCS Expressions 可见) ---- */
        debug_gyro_dps[0]  = gyro_dps[0];
        debug_gyro_dps[1]  = gyro_dps[1];
        debug_gyro_dps[2]  = gyro_dps[2];
        debug_accel_g[0]   = accel_g[0];
        debug_accel_g[1]   = accel_g[1];
        debug_accel_g[2]   = accel_g[2];
        debug_euler_deg[0] = roll;
        debug_euler_deg[1] = pitch;
        debug_euler_deg[2] = yaw;
        debug_loop_count++;
    }
}
#endif // USE_IMU_SENSOR



/* =========================================================================
 * 底盘相对位置运动接口
 * ========================================================================= */
static void Chassis_MoveRelativeCmInternal(float distance_cm, uint8_t smooth_start, float speed_max)
{
    if (target_ctrl_Mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(target_ctrl_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_pos_current_pulse = 0.0f;
        g_pos_target_pulse = distance_cm * POSITION_PULSE_PER_CM;
        g_pos_move_active = (fabsf(g_pos_target_pulse) > POSITION_TOLERANCE_PULSE) ? 1 : 0;
        g_pos_smooth_start = smooth_start;
        g_pos_speed_filter = 0.0f;
        g_pos_speed_max = speed_max;
        g_pos_line_trace_assist = 0;

        g_ctrl_mode = 0;
        g_line_trace_running = 0;
        LineTrace_ResetState();
        g_yaw_turn_active = 0;
        target_speed = 0.0f;
        target_turn = 0.0f;
        pid_speed.ErrorInt = 0.0f;
        pid_turn.ErrorInt = 0.0f;

        xSemaphoreGive(target_ctrl_Mutex);
    }
}

void Chassis_MoveRelativeCm(float distance_cm)
{
    Chassis_MoveRelativeCmInternal(distance_cm, 0, POSITION_SPEED_MAX);
}

static void Chassis_MoveRelativeCmLineTrace(float distance_cm, float speed_max, const LineTrace_Config_t *line_config)
{
    Chassis_MoveRelativeCmInternal(distance_cm, 1, speed_max);
    if (target_ctrl_Mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(target_ctrl_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_line_trace_config = (line_config != NULL) ? *line_config : g_line_trace_s2_config;
        LineTrace_ResetState();
        g_pos_line_trace_assist = 1;
        g_pos_stopwatch_active = g_pos_move_active;
        xSemaphoreGive(target_ctrl_Mutex);
    }

    if (g_pos_stopwatch_active) {
        Stopwatch_Start();
    }
}

/* =========================================================================
 * 电机底盘核心控制任务 (执行层 / 内环)
 * ========================================================================= */
static void control_test_task(void *pvParameters) 
{
    // 1. 初始化 PID 结构体内存
    PID_Init((PID_t *)&pid_speed);
    PID_Init((PID_t *)&pid_turn);

    // 2. 配置【速度环】参数
    pid_speed.Kp = 3.0f;      
    pid_speed.Ki = 0.5f;       
    pid_speed.Kd = 0.0f;       
    pid_speed.OutMax = 70.0f;     
    pid_speed.OutMin = -70.0f;    
    pid_speed.ErrorIntMax = 130.0f;
    pid_speed.ErrorIntMin = -130.0f;
    pid_speed.OutOffset = 0.0f;     

    // 3. 配置【转向环】参数
    pid_turn.Kp = 1.2f;
    pid_turn.Ki = 0.0f;
    pid_turn.Kd = 0.0f;
    pid_turn.OutMax = 45.0f;
    pid_turn.OutMin = -45.0f;
    pid_turn.ErrorIntMax = 60.0f;
    pid_turn.ErrorIntMin = -60.0f;
    pid_turn.OutOffset = 0.0f;

    vTaskDelay(pdMS_TO_TICKS(1500)); // 等待系统和外设完全稳定


    while(1) {
        vTaskDelay(pdMS_TO_TICKS(50)); // 严格的 50ms 控制周期
        
        // ================= 蓝牙指令接入层 =================
        // 原子快照: 临界区保护, 防止 BT_Task (pri 3) 在读取中途抢占
        // 导致 cmd_type 与 D1/D2 来自不同帧 (struct tearing)
        BT_Ctrl_Cmd_t cmd;
        {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            cmd = g_bt_cmd;
            g_bt_cmd.cmd_type = 0x00;   /* 立即清零, 防止重复消费 */
            if (!primask) __enable_irq();
        }

        // 获取控制目标互斥锁 (10ms 超时, 防止死锁)
        if (xSemaphoreTake(target_ctrl_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // 0x01 指令代表：增量控制模式 (按一次加一次)
            if (cmd.cmd_type == CMD_TYPE_JOYSTICK) {
                g_ctrl_mode = 0;
                g_line_trace_running = 0;
                LineTrace_ResetState();
                target_speed += (float)cmd.D1;
                target_turn  += (float)cmd.D2;
                Chassis_ClearOuterLoops();
                g_bt_beep_cmd = 1; //告诉蜂鸣器任务，短响一声！
            }

            // 0x02 指令代表：紧急停止模式 (覆盖并归零)
            else if (cmd.cmd_type == CMD_TYPE_ACTION) {
                g_ctrl_mode = 0;
                g_line_trace_running = 0;
                LineTrace_ResetState();
                Chassis_ClearTarget();
                Chassis_ClearOuterLoops();
                g_bt_beep_cmd = 2; //告诉蜂鸣器任务，长鸣报警！
            }

            // 0x03 指令代表：基于当前 yaw 的相对角度转向，D1>0 左转，D1<0 右转
            else if (cmd.cmd_type == CMD_TYPE_YAW_TURN) {
                g_yaw_target = yaw - (float)cmd.D1;
                g_yaw_turn_active = 1;
                g_pos_move_active = 0;
                Chassis_ClearTarget();
                Chassis_ClearPidIntegral();
                g_bt_beep_cmd = 1;
            }

            Chassis_LimitTarget();

            xSemaphoreGive(target_ctrl_Mutex);
        }
        // ========================================================

        // === 循迹模式: 由 API 统一更新速度、转向和停车线处理 ===
        if (LineTrace_IsRunning() && !g_yaw_turn_active && !g_pos_move_active) {
            LineTrace_Update();
        }
        // ========================================================

        // === yaw 相对角度转向: 覆盖目标速度/转向，复用底盘内环 ===
        if (g_yaw_turn_active) {
            float yaw_error = g_yaw_target - yaw;

            if (xSemaphoreTake(target_ctrl_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (fabsf(yaw_error) <= YAW_TURN_TOLERANCE) {
                    Chassis_ClearTarget();
                    g_yaw_turn_active = 0;
                    pid_turn.ErrorInt = 0.0f;
                } else {
                    float yaw_turn = yaw_error * YAW_TURN_KP;

                    if (yaw_turn > YAW_TURN_MAX) yaw_turn = YAW_TURN_MAX;
                    if (yaw_turn < -YAW_TURN_MAX) yaw_turn = -YAW_TURN_MAX;

                    if (yaw_turn > 0.0f && yaw_turn < YAW_TURN_MIN) yaw_turn = YAW_TURN_MIN;
                    if (yaw_turn < 0.0f && yaw_turn > -YAW_TURN_MIN) yaw_turn = -YAW_TURN_MIN;

                    target_speed = 0.0f;
                    target_turn  = yaw_turn;
                }
                xSemaphoreGive(target_ctrl_Mutex);
            }
        }
        // ========================================================

        // 1. 获取当前轮速 (-100 ~ +100)
        int16_t raw_speed_L = Encoder_Get(1);
        int16_t raw_speed_R = Encoder_Get(2); 
        
        speed_L = (int16_t)(((int32_t)raw_speed_L * 100) / MAX_PULSE_50MS);
        speed_R = (int16_t)(((int32_t)raw_speed_R * 100) / MAX_PULSE_50MS);

        if (g_pos_move_active) {
            float delta_pulse = ((float)raw_speed_L + (float)raw_speed_R) * 0.5f;
            g_pos_current_pulse += delta_pulse;
        }

        // 2. 运动学正解：提取特征
        float ave_speed = (speed_L + speed_R) / 2.0f; // 整体前移速度
        float dif_speed = (float)(speed_L - speed_R); // 左右速度差 (转动特征)
        static float dif_speed_filter = 0.0f;
        float dif_speed_delta = dif_speed - dif_speed_filter;
        if (dif_speed_delta > 20.0f) dif_speed = dif_speed_filter + 20.0f;
        if (dif_speed_delta < -20.0f) dif_speed = dif_speed_filter - 20.0f;
        dif_speed_filter = dif_speed_filter * 0.8f + dif_speed * 0.2f;

        if (g_pos_move_active) {
            float pos_error = g_pos_target_pulse - g_pos_current_pulse;

            if (xSemaphoreTake(target_ctrl_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (fabsf(pos_error) <= POSITION_TOLERANCE_PULSE) {
                    Chassis_ClearTarget();
                    g_pos_move_active = 0;
                    g_pos_smooth_start = 0;
                    g_pos_speed_filter = 0.0f;
                    g_pos_line_trace_assist = 0;
                    if (g_pos_stopwatch_active) {
                        Stopwatch_Stop();
                        g_pos_stopwatch_active = 0;
                    }
                    pid_speed.ErrorInt = 0.0f;
                } else {
                    float pos_speed = pos_error * POSITION_KP;
                    float pos_speed_max = (g_pos_speed_max > 0.0f) ? g_pos_speed_max : POSITION_SPEED_MAX;

                    if (pos_speed > pos_speed_max) pos_speed = pos_speed_max;
                    if (pos_speed < -pos_speed_max) pos_speed = -pos_speed_max;

                    if (pos_speed > 0.0f && pos_speed < POSITION_SPEED_MIN) pos_speed = POSITION_SPEED_MIN;
                    if (pos_speed < 0.0f && pos_speed > -POSITION_SPEED_MIN) pos_speed = -POSITION_SPEED_MIN;

                    if (g_pos_smooth_start) {
                        if (pos_speed > 0.0f) {
                            g_pos_speed_filter += POSITION_SMOOTH_STEP;
                            if (g_pos_speed_filter >= pos_speed) {
                                g_pos_speed_filter = pos_speed;
                                g_pos_smooth_start = 0;
                            }
                        } else {
                            g_pos_speed_filter -= POSITION_SMOOTH_STEP;
                            if (g_pos_speed_filter <= pos_speed) {
                                g_pos_speed_filter = pos_speed;
                                g_pos_smooth_start = 0;
                            }
                        }
                        pos_speed = g_pos_speed_filter;
                    } else {
                        g_pos_speed_filter = pos_speed;
                    }

                    target_speed = pos_speed;
                    target_turn = g_pos_line_trace_assist ? LineTrace_CalcTurn() : 0.0f;
                }
                xSemaphoreGive(target_ctrl_Mutex);
            }
        }

        float local_target_speed = pid_speed.Target;
        float local_target_turn  = pid_turn.Target;
        if (xSemaphoreTake(target_ctrl_Mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            local_target_speed = target_speed;
            local_target_turn  = target_turn;
            xSemaphoreGive(target_ctrl_Mutex);
        }

        // 3. 并联 PID 运算
        // --- 速度环 ---
        pid_speed.Target = local_target_speed;
        pid_speed.Actual = ave_speed;
        PID_Update((PID_t *)&pid_speed);

        // --- 转向环 ---
        pid_turn.Target = local_target_turn;
        pid_turn.Actual = dif_speed_filter;
        PID_Update((PID_t *)&pid_turn);

        // 4.电子手刹防抖
        // 目标为0且小车几乎停稳时，清空积分旧账，防止因为死区摩擦力导致的抽搐
        if (local_target_speed == 0.0f && ave_speed > -3.0f && ave_speed < 3.0f) {
            pid_speed.ErrorInt = 0.0f; 
        }
        if (local_target_turn == 0.0f && dif_speed_filter > -3.0f && dif_speed_filter < 3.0f) {
            pid_turn.ErrorInt = 0.0f;
            pid_turn.Out = 0.0f;
        }
        if (local_target_speed == 0.0f && local_target_turn == 0.0f) {
            pid_speed.ErrorInt = 0.0f;
            pid_speed.Out = 0.0f;
            pid_turn.ErrorInt = 0.0f;
            pid_turn.Out = 0.0f;
        }

        // 5. 运动学逆解算：合并指令下发给轮子
        int32_t ave_pwm = (int32_t)pid_speed.Out;
        int32_t dif_pwm = (int32_t)pid_turn.Out;
        
        target_pwm_L = ave_pwm + (dif_pwm / 2);
        target_pwm_R = ave_pwm - (dif_pwm / 2);

        // 6. 最终物理限幅 (必须做，防止相加后溢出 100)
        if (target_pwm_L > 100) target_pwm_L = 100;
        else if (target_pwm_L < -100) target_pwm_L = -100;
        
        if (target_pwm_R > 100) target_pwm_R = 100;
        else if (target_pwm_R < -100) target_pwm_R = -100;

        // 7. 执行最终电机指令
        Motor_SetPWM(1, target_pwm_L); 
        Motor_SetPWM(2, target_pwm_R); 
        
        

    }
}

#if USE_ZDT_STEPPER
/* =========================================================================
 * 张大头闭环步进电机独立测试任务
 * ========================================================================= */

static void zdt_motor_test_task(void *pvParameters)
{
    // 1. 等待电机驱动板上电初始化彻底完成
    vTaskDelay(pdMS_TO_TICKS(1500));

    ZDT_Init();
    ZDT_Enable(ZDT_DEFAULT_ADDR, true, false);
    vTaskDelay(pdMS_TO_TICKS(1000));

    RodActuator_Init(NULL);
    RodActuator_SetCenter();
    vTaskDelay(pdMS_TO_TICKS(500));

    bool contest_started = false;
    const float target_pos_px = 0.0f;
    bool vision_zero_ready = false;
    float vision_zero_offset_px = 0.0f;
    float ball_pos_px = 0.0f;
    float last_ball_pos_px = 0.0f;
    float ball_vel_px = 0.0f;
    int32_t last_target_pulse = 0;
    uint16_t vision_lost_count = 0;
#if USE_VOFA_DEBUG
    char vofa_buf[64];
#endif

    RodActuator_SetTargetPulse(0);

    while (1) {
        bool question3_running = (g_question_ui_state == 1U && g_selected_question == 3U);

        // 实时读取串口反馈，避免接收队列堆积
        if (Motor1.rxReady == true) {
            uint8_t frame[ZDT_MAX_FRAME_LEN];
            uint8_t frame_len = 0;

            while (ZDT_ReadFrame(frame, &frame_len)) {
                (void)frame_len;
            }
        }

        if (g_vision_ready_flag != 0U) {
            float raw_ball_pos_px;
            float new_ball_pos_px;
            g_vision_ready_flag = 0U;

            if (!vision_zero_ready) {
                vision_zero_offset_px = (float)g_vision_x_offset;
                vision_zero_ready = true;
                ball_pos_px = 0.0f;
                last_ball_pos_px = 0.0f;
                ball_vel_px = 0.0f;
            }

            raw_ball_pos_px = ((float)g_vision_x_offset - vision_zero_offset_px) * Q3_VISION_POS_SIGN + Q3_ZERO_BIAS_PX;
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

        if (!question3_running) {
            if (contest_started) {
                RodActuator_ReturnCenter();
                Stopwatch_Stop();
                contest_started = false;
                vision_zero_offset_px = 0.0f;
                ball_pos_px = 0.0f;
                last_ball_pos_px = 0.0f;
                ball_vel_px = 0.0f;
                last_target_pulse = 0;
                vision_lost_count = 0;
                g_vision_ready_flag = 0U;
            } else {
                RodActuator_SetTargetPulse(0);
            }
        } else {
            if (!contest_started) {
                contest_started = true;
                ball_pos_px = 0.0f;
                last_ball_pos_px = ball_pos_px;
                ball_vel_px = 0.0f;
                last_target_pulse = 0;
                vision_lost_count = 0;
                Stopwatch_Start();
            }

            if (g_vision_ready_flag != 0U) {
                float raw_ball_pos_px = ((float)g_vision_x_offset - vision_zero_offset_px) * Q3_VISION_POS_SIGN;
                float new_ball_pos_px;
                g_vision_ready_flag = 0U;

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

            if (vision_lost_count >= Q3_VISION_LOST_CYCLES) {
                ball_vel_px = 0.0f;
                last_target_pulse = 0;
                RodActuator_ReturnCenter();
            } else {
                float pos_error_px = target_pos_px - ball_pos_px;
                float rod_cmd = Q3_KP_PULSE_PER_PX * pos_error_px - Q3_KD_PULSE_PER_PX * ball_vel_px;
                if (rod_cmd > (float)ROD_DEFAULT_MAX_PULSE) {
                    rod_cmd = (float)ROD_DEFAULT_MAX_PULSE;
                } else if (rod_cmd < (float)ROD_DEFAULT_MIN_PULSE) {
                    rod_cmd = (float)ROD_DEFAULT_MIN_PULSE;
                }

                last_target_pulse = (int32_t)rod_cmd;
                RodActuator_SetTargetPulse(last_target_pulse);
            }
        }

#if USE_VOFA_DEBUG
        snprintf(vofa_buf, sizeof(vofa_buf), "%ld,%ld,%ld\n",
                 (long)ball_pos_px,
                 (long)target_pos_px,
                 (long)last_target_pulse);
        VOFA_SendString(vofa_buf);
#endif

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
#endif


/* =========================================================================
 * 定义按键扫描任务
 * ========================================================================= */

static void Key_Scan_Task(void *pvParameters)
{
    while(1) {
        Key_Tick();
        uint8_t key_val = Key_GetNum();

        if (g_question_ui_state == 0) {
            if (key_val == 2) {
                g_selected_question--;
                if (g_selected_question < 2) {
                    g_selected_question = 6;
                }
            } else if (key_val == 3) {
                g_selected_question++;
                if (g_selected_question > 6) {
                    g_selected_question = 2;
                }
            } else if (key_val == 4) {
                g_question_ui_state = 1;

                if (g_selected_question == 2) {
                    LineTrace_Start(&g_line_trace_q2_config);
                } else if (g_selected_question == 3) {
                    // Question 3 由 ZDT_Test 任务接管步进电机和摆杆脚本
                } else if (g_selected_question == 4) {
                    Chassis_MoveRelativeCmLineTrace(POSITION_S2_DISTANCE_CM, POSITION_S2_SPEED_MAX, &g_line_trace_s2_config);
                } else if (g_selected_question == 5 || g_selected_question == 6) {
                    LineTrace_StartSmoothWithStep(&g_line_trace_q56_config, 0.5f);
                }
            }
        } else if (key_val == 2) {
            Chassis_StopAll();
            g_question_ui_state = 0;
            g_selected_question = 2;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


/* =========================================================================
 * 定义OLED任务
 * ========================================================================= */
/* 在文件顶部定义全局变量，供两个任务共享 */
TaskStatus_t pxTaskStatusArray[MAX_TRACKED_TASKS];
uint32_t ulTotalRunTime = 0; 
uint32_t g_task_count = 0; // 记录当前任务总数


static void Task_OLED_Display(void *pvParameters)
{
    OLED_Init();
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        OLED_Clear();

        if (g_question_ui_state == 0) {
            OLED_Printf(0, 0, OLED_8X16, "Select Question");
            for (uint8_t question = 2; question <= 6; question++) {
                uint8_t row = (uint8_t)(question - 2);
                OLED_Printf(0, 16 + row * 8, OLED_6X8, "%cQuestion %u",
                            (question == g_selected_question) ? '>' : ' ', question);
            }
        } else {
            uint32_t elapsed_cs = g_stopwatch_elapsed_cs;
            if (g_stopwatch_running) {
                TickType_t now = xTaskGetTickCount();
                elapsed_cs += (uint32_t)(((now - g_stopwatch_start_tick) * 100U) / configTICK_RATE_HZ);
            }

            OLED_Printf(0, 0, OLED_8X16, "Question %u", g_selected_question);
            if (g_selected_question == 3) {
                OLED_Printf(0, 16, OLED_6X8, "Vision UART1");
                OLED_Printf(0, 26, OLED_6X8, "RX:%lu", (unsigned long)g_rx_pulse);
                OLED_Printf(0, 36, OLED_6X8, "X:%d", (int)g_vision_x_offset);
                OLED_Printf(0, 48, OLED_6X8, "Zero Hold");
            } else {
                OLED_Printf(0, 24, OLED_6X8, "Running");
                OLED_Printf(0, 40, OLED_8X16, "%lu.%02lu s",
                            (unsigned long)(elapsed_cs / 100U),
                            (unsigned long)(elapsed_cs % 100U));
            }
        }

        OLED_Update();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

// --- cpu监控任务 ---
static void cpu_monitor_task(void *pvParameters) {
    static uint32_t last_task_runtime[MAX_TRACKED_TASKS] = {0};
    static uint32_t last_total_runtime = 0;
    
    vTaskDelay(pdMS_TO_TICKS(2000)); 

    while(1) {
        // 使用全局变量 pxTaskStatusArray 更新数据
        g_task_count = uxTaskGetSystemState(pxTaskStatusArray, MAX_TRACKED_TASKS, &ulTotalRunTime);
        uint32_t delta_total = ulTotalRunTime - last_total_runtime;

        if(g_task_count > 0 && delta_total > 0) {
            for(int i = 0; i < g_task_count; i++) {
                if(pxTaskStatusArray[i].xHandle != NULL) {
                    memcpy((void *)task_usage_table[i].name, (void *)pxTaskStatusArray[i].pcTaskName, 15);
                    uint32_t delta_task = pxTaskStatusArray[i].ulRunTimeCounter - last_task_runtime[i];
                    task_usage_table[i].cpu_usage = ((float)delta_task / (float)delta_total) * 100.0f;
                    last_task_runtime[i] = pxTaskStatusArray[i].ulRunTimeCounter;
                    task_usage_table[i].min_stack_free = uxTaskGetStackHighWaterMark(pxTaskStatusArray[i].xHandle);
                }
            }
            last_total_runtime = ulTotalRunTime;
        }

        /* 堆剩余量监控: CCS Expressions 可观察, 检测内存泄漏和碎片化 */
        debug_free_heap = xPortGetFreeHeapSize();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* =========================================================================
 * 蜂鸣器声音调度任务
 * ========================================================================= */
static void buzzer_task(void *pvParameters)
{
    // 确保开机时默认静音
    DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_0_PIN); 

    while(1) {
        // 1. 处理蓝牙按键提示音 (短鸣)
        if (g_bt_beep_cmd == 1) {
            DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_0_PIN);
            vTaskDelay(pdMS_TO_TICKS(50)); // 短促清脆
            DL_GPIO_setPins(GPIO_BEEP_PORT,GPIO_BEEP_PIN_0_PIN);
            g_bt_beep_cmd = 0; // 处理完毕，清除标志
        } 
        // 2. 处理蓝牙急停提示音 (长鸣)
        else if (g_bt_beep_cmd == 2) {
            DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_0_PIN);
            vTaskDelay(pdMS_TO_TICKS(400)); // 紧急长鸣
            DL_GPIO_setPins(GPIO_BEEP_PORT,GPIO_BEEP_PIN_0_PIN);
            g_bt_beep_cmd = 0;
        }
        // 3. 无发声任务时，保持静音并休眠
        else {
            DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_0_PIN);
            vTaskDelay(pdMS_TO_TICKS(50)); // 休眠 50ms 释放 CPU 资源
        }
    }
}

/* =========================================================================
 * LED 心跳指示任务
 * ========================================================================= */
static void led_heartbeat_task(void *pvParameters)
{
    while(1) {

        DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_Freertos_LED_PIN);
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* =========================================================================
 * VOFA JustFloat 调试任务
 *
 * 功能: 通过 UART_1 (UART1, PB4/PB5, 115200) 以 JustFloat 格式
 *       周期性地发送 IMU 9 通道数据到 VOFA+ 上位机.
 *
 * JustFloat 帧 (9 通道, 36 字节 + 4 字节帧尾 0x00 0x00 0x80 0x7F):
 *   Ch 0-2: GyroX/Y/Z       float   (dps)
 *   Ch 3-5: AccelX/Y/Z       float   (g)
 *   Ch 6-8: Roll/Pitch/Yaw   float   (deg)
 *
 * 发送周期: 10ms (100Hz)
 *
 * 优先级: 2 (低于 IMU_Task 的 3, 与 Ctrl_Task 同级)
 * 任务栈: 256 words (1024 bytes) — hvofa/vofa_types 均在全局区
 *
 * VOFA+ 配置: 数据协议选 JustFloat, 9 通道 (全 float)
 * ========================================================================= */

/* VOFA UART 发送回调 — 与 Test 工程完全一致 */
// static bool VOFA_UART_SendCallback(void *serial_handle,
//                                     const uint8_t *data, uint16_t len)
// {
//     (void)serial_handle;
//     for (uint16_t i = 0; i < len; i++) {
//         DL_UART_Main_transmitDataBlocking(UART_1_INST, data[i]);
//     }
//     return true;
// }

/* VOFA UART 发送回调 — 改为使用 UART_3_INST (蓝牙接口) */
static bool VOFA_UART_SendCallback(void *serial_handle,
                                    const uint8_t *data, uint16_t len)
{
    (void)serial_handle;
    for (uint16_t i = 0; i < len; i++) {
        // 将原代码的 UART_1_INST 替换为 UART_3_INST
        DL_UART_Main_transmitDataBlocking(UART_3_INST, data[i]);
    }
    return true;
}



#if USE_IMU_SENSOR
static void vofa_debug_task(void *pvParameters)
{
    /* 初始化 VOFA: JustFloat 3通道, 标明 115200 */
    VOFA_Init(&hvofa, NULL, VOFA_UART_SendCallback,
              VOFA_FMT_JUSTFLOAT, VOFA_BAUD_115200);

    for (int i = 0; i < 3; i++) vofa_types[i] = VOFA_DATA_FLOAT;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // 毫无压力！直接拉到 100Hz (10ms一帧)，波形绝对丝滑
    const TickType_t xPeriod = pdMS_TO_TICKS(10);  

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        float frame[3];

        taskENTER_CRITICAL();
        frame[0] = g_pos_target_pulse - g_pos_current_pulse;
        frame[1] = g_pos_current_pulse;
        frame[2] = target_speed;

        taskEXIT_CRITICAL();

        VOFA_SendData(&hvofa, frame, 3, vofa_types);
    }
}
#endif
/* =========================================================================
 * 3. 任务创建区 (对外暴露的唯一接口)
 * ========================================================================= */
void RTOS_Tasks_Init(void) {
    // 创建控制目标互斥锁 (必须在任务创建之前, configUSE_MUTEXES=1)
    target_ctrl_Mutex = xSemaphoreCreateMutex();
    configASSERT(target_ctrl_Mutex != NULL);  /* 堆耗尽时在调试器里立刻定位 */

    // IMU 采集任务 (关键: 创建失败则紧急停车+复位)
#if USE_IMU_SENSOR
    if (xTaskCreate((TaskFunction_t)imu_task,
                "IMU_Task",
                768,        // 3KB: 保证浮点数学库与姿态解算足够的栈空间
                NULL,
                4,
                &IMU_Task_Handle) != pdPASS) {
        /* 堆耗尽 → 关中断, 停电机, 报警, 复位 (与钩子函数相同的紧急流程) */
        __disable_irq();
        Motor_SetPWM(1, 0);  Motor_SetPWM(2, 0);
        DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_0_PIN);
        DL_GPIO_clearPins(GPIO_LED_PORT,  GPIO_LED_Freertos_LED_PIN);
        for (volatile uint32_t i = 0; i < 8000000; i++) { __NOP(); }
        NVIC_SystemReset();
        while(1) {}
    }
#endif

    // 控制测试任务 (关键: 创建失败则紧急停车+复位)
    if (xTaskCreate((TaskFunction_t)control_test_task,
                "Ctrl_Task",
                512,        // 2KB: 双环PID控制计算与蓝牙指令解析
                NULL,
                2,          // 优先级稍低于 IMU，确保姿态解算优先
                NULL) != pdPASS) {
        __disable_irq();
        Motor_SetPWM(1, 0);  Motor_SetPWM(2, 0);
        DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_0_PIN);
        DL_GPIO_clearPins(GPIO_LED_PORT,  GPIO_LED_Freertos_LED_PIN);
        for (volatile uint32_t i = 0; i < 8000000; i++) { __NOP(); }
        NVIC_SystemReset();
        while(1) {}
    }

    // CPU 监控任务 
    xTaskCreate((TaskFunction_t)cpu_monitor_task, 
                "CPU_Monitor",
                256,        // 1KB: 系统任务状态获取 API 开销较大
                NULL,
                1,
                NULL);

    // 按键扫描任务
    xTaskCreate((TaskFunction_t)Key_Scan_Task, 
                "KeyScan", 
                128, 
                NULL, 
                2, 
                NULL);

    // 超声波独立测距任务
#if USE_ULTRASONIC
    xTaskCreate((TaskFunction_t)ultrasonic_task, 
                "Sonar_Task", 
                256,        // 1KB: 超声波计算与距离状态更新
                NULL, 
                2, 
                &Sonar_Task_Handle);
#endif

    // 灰度传感器任务
#if USE_GRAY_SENSOR
    xTaskCreate(Gray_Task, 
                "GrayTask", 
                128,          
                NULL, 
                2, 
                NULL);
#endif

    // OLED 显示任务
#if USE_OLED_DISPLAY
    xTaskCreate(Task_OLED_Display,  
                "OLED_Task",         
                768,        // 3KB: OLED_Printf 底层 vsprintf 耗栈防御
                NULL,               
                1,                   
                NULL);               
#endif

    // 蓝牙通信接收任务
#if USE_BLUETOOTH
    xTaskCreate(Bluetooth_Task, 
                "BT_Task",
                128,       
                NULL, 
                3, 
                NULL);
#endif

    // LED 心跳指示任务 
    xTaskCreate((TaskFunction_t)led_heartbeat_task, 
                "Heartbeat", 
                128,        // 512B: FreeRTOS 任务栈安全底线
                NULL, 
                1,            
                NULL);

    // 蜂鸣器调度任务
    xTaskCreate((TaskFunction_t)buzzer_task, 
                "Buzzer", 
                128,        // 512B: 简单的引脚控制与延时
                NULL, 
                2,  
                NULL);
    // 创建步进电机测试任务
#if USE_ZDT_STEPPER
    xTaskCreate((TaskFunction_t)zdt_motor_test_task,
                "ZDT_Test",
                256,
                NULL,
                2,
                NULL);
#endif

    /* VOFA RawData 调试任务 — IMU 13ch + debug flags 100Hz */
#if USE_IMU_SENSOR && USE_VOFA_DEBUG
    xTaskCreate(vofa_debug_task,
                "VOFA_Debug",
                256,        /* 1024B: hvofa(~800B) 在全局区, frame 在栈上 ~64B */
                NULL,
                2,          /* 优先级 2, 低于 IMU_Task(3), 与 Ctrl_Task 同级 */
                NULL);
#endif
}

/* =========================================================================
 * 4. 硬件中断服务函数 (ISR)
 * ========================================================================= */

void GROUP1_IRQHandler(void) {
    // ================= 通道1：左轮 A 相 =================
    uint32_t gpioB_LA_status = DL_GPIO_getEnabledInterruptStatus(ENCODER_A_LA_PORT, ENCODER_A_LA_PIN);
    if ((gpioB_LA_status & ENCODER_A_LA_PIN) == ENCODER_A_LA_PIN) {
        DL_GPIO_clearInterruptStatus(ENCODER_A_LA_PORT, ENCODER_A_LA_PIN);
        bool encoder_L_B = DL_GPIO_readPins(ENCODER_B_LB_PORT, ENCODER_B_LB_PIN);
        if (encoder_L_B == 0) encoder_L++; else encoder_L--;
    }

    // ================= 通道2：右轮 A 相 =================
    uint32_t gpioA_RA_status = DL_GPIO_getEnabledInterruptStatus(ENCODER_A_RA_PORT, ENCODER_A_RA_PIN);
    if ((gpioA_RA_status & ENCODER_A_RA_PIN) == ENCODER_A_RA_PIN) {
        DL_GPIO_clearInterruptStatus(ENCODER_A_RA_PORT, ENCODER_A_RA_PIN);
        bool encoder_R_B = DL_GPIO_readPins(ENCODER_B_RB_PORT, ENCODER_B_RB_PIN);
        if (encoder_R_B == 0) encoder_R++; else encoder_R--;
    }

    // ================= 通道3：IMU 中断 =================
    if (DL_GPIO_getEnabledInterruptStatus(GPIO_IMU_PORT, GPIO_IMU_INT1_PIN)) {
        DL_GPIO_clearInterruptStatus(GPIO_IMU_PORT, GPIO_IMU_INT1_PIN);
#if USE_IMU_SENSOR
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (IMU_Task_Handle != NULL) {
            vTaskNotifyGiveFromISR(IMU_Task_Handle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
#endif
    }

    // ===== 超声波 ECHO =====
    uint32_t echo_status_A = DL_GPIO_getEnabledInterruptStatus(GPIO_Ultrasonic_PORT, GPIO_Ultrasonic_PIN_Echo_PIN);
    if ((echo_status_A & GPIO_Ultrasonic_PIN_Echo_PIN) == GPIO_Ultrasonic_PIN_Echo_PIN) {
        DL_GPIO_clearInterruptStatus(GPIO_Ultrasonic_PORT, GPIO_Ultrasonic_PIN_Echo_PIN);
#if USE_ULTRASONIC
        Ultrasonic_ISR_Handler();
#endif
    }
}


/* =========================================================================
 * FreeRTOS 运行时间统计 (Run Time Stats) 底层支持函数
 * ========================================================================= */

// 1. 初始化/启动秒表（由于 SysConfig 已经勾选了 Start Timer，这里可以为空）
void App_StartStatsTimer(void) 
{
    // 留空即可，底层的 SYSCFG_DL_init() 已经帮我们启动了 TIMER_STATS
}

// 2. 读取秒表的当前时间（单位：微秒）
uint32_t App_GetStatsTimerValue(void) 
{
    // 直接读取我们刚配置的 TIMER_STATS 的当前计数值
    return DL_Timer_getTimerCount(TIMER_STATS_INST);
}