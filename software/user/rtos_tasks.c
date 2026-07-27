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
// ==========================================
// 运动控制全局变量 (解耦架构)
// ==========================================
volatile int32_t target_pwm_L = 0; 
volatile int32_t target_pwm_R = 0; 

// 策略层输入接口：整体前进速度 和 旋转差速
volatile float target_speed = 0.0f;
volatile float target_turn = 0.0f;  // 0为直线，正数右转，负数左转

// 相对 yaw 角度转向状态
volatile uint8_t g_yaw_turn_active = 0;
volatile float g_yaw_target = 0.0f;

// 控制模式: 0=蓝牙遥控, 1=循迹模式 (S4 按键切换)
volatile uint8_t g_ctrl_mode = 0;

// 循迹参数 (可按需调整)
#define LINE_FOLLOW_SPEED  35.0f   // 循迹前进速度 (±100范围, 不宜太快)
#define LINE_KP            0.25f   // 循迹转向比例增益 (g_gray_error * Kp → target_turn)

// yaw 相对转向参数
#define YAW_TURN_KP          0.8f
#define YAW_TURN_MAX         24.0f
#define YAW_TURN_MIN         8.0f
#define YAW_TURN_TOLERANCE   1.0f

// 控制目标互斥锁 (保护 target_speed / target_turn 的并发访问)
// 使用静态分配避免堆压力 — 25KB 堆已接近极限
static SemaphoreHandle_t target_ctrl_Mutex = NULL;


// 底盘执行层：速度环 和 转向环
// 注意: 这两个变量仅 control_test_task (pri 2) 独占访问,
// 无 ISR 或其他任务并发, 不需要 volatile.
PID_t pid_speed;
PID_t pid_turn;

// ==========================================
// 蜂鸣器跨任务通信标志
// ==========================================
volatile uint8_t g_bt_beep_cmd = 0; // 0:静音, 1:短鸣(普通指令), 2:长鸣(急停)


//cpu相关的变量

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
    pid_turn.Kp = 2.0f;
    pid_turn.Ki = 0.02f;
    pid_turn.Kd = 0.0f;
    pid_turn.OutMax = 60.0f;
    pid_turn.OutMin = -60.0f;
    pid_turn.ErrorIntMax = 60.0f;
    pid_turn.ErrorIntMin = -60.0f;
    pid_turn.OutOffset = 10.0f;

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
                target_speed += (float)cmd.D1;
                target_turn  += (float)cmd.D2;
                g_yaw_turn_active = 0;
                g_bt_beep_cmd = 1; //告诉蜂鸣器任务，短响一声！
            }

            // 0x02 指令代表：紧急停止模式 (覆盖并归零)
            else if (cmd.cmd_type == CMD_TYPE_ACTION) {
                target_speed = 0.0f;
                target_turn  = 0.0f;
                g_yaw_turn_active = 0;
                g_bt_beep_cmd = 2; //告诉蜂鸣器任务，长鸣报警！
            }

            // 0x03 指令代表：基于当前 yaw 的相对角度转向，D1>0 左转，D1<0 右转
            else if (cmd.cmd_type == CMD_TYPE_YAW_TURN) {
                g_yaw_target = yaw - (float)cmd.D1;
                g_yaw_turn_active = 1;
                target_speed = 0.0f;
                target_turn  = 0.0f;
                pid_speed.ErrorInt = 0.0f;
                pid_turn.ErrorInt = 0.0f;
                g_bt_beep_cmd = 1;
            }

            // 【安全限幅】
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

            xSemaphoreGive(target_ctrl_Mutex);
        }
        // ========================================================

        // === 循迹模式: 用灰度传感器偏差覆盖 target_turn ===
        if (g_ctrl_mode == 1 && !g_yaw_turn_active) {
            // 读取 Gray_Task 算好的偏差 (±300 范围)
            int32_t line_error = g_gray_error;

            // 线性映射: error * Kp → target_turn, 并钳位在 ±100
            float line_turn = (float)line_error * LINE_KP;
            if (line_turn > 100.0f) line_turn = 100.0f;
            if (line_turn < -100.0f) line_turn = -100.0f;

            if (xSemaphoreTake(target_ctrl_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                target_speed = LINE_FOLLOW_SPEED;
                target_turn  = line_turn;
                xSemaphoreGive(target_ctrl_Mutex);
            }
        }
        // ========================================================

        // === yaw 相对角度转向: 覆盖目标速度/转向，复用底盘内环 ===
        if (g_yaw_turn_active) {
            float yaw_error = g_yaw_target - yaw;

            if (xSemaphoreTake(target_ctrl_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (fabsf(yaw_error) <= YAW_TURN_TOLERANCE) {
                    target_speed = 0.0f;
                    target_turn  = 0.0f;
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

        // 2. 运动学正解：提取特征
         float ave_speed = (speed_L + speed_R) / 2.0f; // 整体前移速度
         float dif_speed = (float)(speed_L - speed_R); // 左右速度差 (转动特征)
         static float dif_speed_filter = 0.0f;
         float dif_speed_delta = dif_speed - dif_speed_filter;
         if (dif_speed_delta > 20.0f) dif_speed = dif_speed_filter + 20.0f;
         if (dif_speed_delta < -20.0f) dif_speed = dif_speed_filter - 20.0f;
         dif_speed_filter = dif_speed_filter * 0.8f + dif_speed * 0.2f;

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

    // 初始化中断环境
    ZDT_Init();

    // 2. 发送使能指令 (锁轴)
    ZDT_Enable(0x01, true, false);
    
    // 给系统 1 秒钟的缓冲时间
    vTaskDelay(pdMS_TO_TICKS(1000)); 

    uint32_t timer_count = 0;
    uint8_t  test_step = 0; // 动作状态机步数

    while(1) {
        // ---------------------------------------------------
        // 动作 1：实时监控串口反馈并响铃
        // ---------------------------------------------------
        if (Motor1.rxReady == true) {
            Motor1.rxReady = false; 
            g_bt_beep_cmd = 1; // 听到滴声说明通信成功
        }

        // ---------------------------------------------------
        // 动作 2：基于 50ms 周期的动作状态机
        // ---------------------------------------------------
        timer_count++;
        if (timer_count >= 30) { 
            timer_count = 0; 
            test_step++;
            
            switch (test_step) {
                case 1:
                    // 【1：速度模式】
                    // 以 200 RPM 的速度，顺时针一直转 (不会自动停)
                    ZDT_RunVelocity(0x01, ZDT_DIR_CW, 200, 50, false);
                    break;
                    
                case 2:
                    // 【2：紧急停止】
                    // 强行打断上面的恒速旋转，瞬间刹车
                    ZDT_Stop(0x01, false);
                    break;
                    
                case 3:
                    // 【3：重置零点】
                    // 把刚刚急停停下的任意位置，强行认作绝对坐标的 "0"
                    ZDT_SetCurrentPositionAsZero(0x01);
                    break;
                    
                case 4:
                    // 【4：相对位置运动】
                    // 以刚才认定的新零点为起点，逆时针转一圈 (3200脉冲)
                    ZDT_MoveRelative(0x01, ZDT_DIR_CCW, 300, 50, 3200);
                    break;
                    
                case 5:
                    // 结束，回到起点，准备下一轮循环展示
                    test_step = 0; 
                    break;
            }
        }

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
        Key_Tick(); // 每 10ms 执行一次按键防抖扫描
        
        uint8_t key_val = Key_GetNum(); // 获取按键值
        if (key_val == 4) // 如果是 S4 被按下: 切换控制模式
        {
            g_ctrl_mode = !g_ctrl_mode;  // 0→1 或 1→0

            // 切换模式时清零控制目标, 防止模式间残留
            if (xSemaphoreTake(target_ctrl_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                target_speed = 0.0f;
                target_turn  = 0.0f;
                xSemaphoreGive(target_ctrl_Mutex);
            }

            // 短鸣提示模式切换
            g_bt_beep_cmd = 1;
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
        // 1. 获取最新距离 (单位：厘米)
#if USE_ULTRASONIC
        float current_dist = global_distance_cm;
#else
        float current_dist = -1.0f;
#endif
        
        // 2. 将浮点数厘米放大 10 倍，转换为整数毫米
        int32_t dist_mm = (int32_t)(current_dist * 10.0f);
        float display_target_speed = 0.0f;
        float display_target_turn  = 0.0f;
        if (xSemaphoreTake(target_ctrl_Mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            display_target_speed = target_speed;
            display_target_turn  = target_turn;
            xSemaphoreGive(target_ctrl_Mutex);
        }
       
        OLED_Printf(0, 0, OLED_6X8, "%s S:%d T:%d",
                    g_ctrl_mode ? "LINE" : "BT  ",
                    (int)display_target_speed, (int)display_target_turn);
        OLED_Printf(0, 8, OLED_6X8, "DIST_MM:%d", dist_mm);
        OLED_Printf(0, 16, OLED_6X8, "Gray:%d err:%ld",
                    (int)g_gray_raw_data, (long)g_gray_error);
        
        OLED_Update();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));
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
        frame[0] = g_yaw_target - yaw;
        frame[1] = yaw;
        frame[2] = target_turn;

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