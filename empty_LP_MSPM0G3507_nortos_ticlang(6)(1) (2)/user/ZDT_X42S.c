#include "alldata.h" 
#include "rtos_tasks.h"

#if USE_ZDT_STEPPER
// 全局电机对象
ZDT_Motor_t Motor1 = {
    .address      = ZDT_DEFAULT_ADDR,
    .isEnable     = false,
    .realPosition = 0,
    .realSpeed    = 0,
    .realCurrent  = 0,
    .isStall      = false,
    .errorCode    = 0,
    .rxReady      = false,
    .rxFrameCount = 0,
    .rxDropCount  = 0
};

/* ====================================================================
 * 底层通信层
 * ==================================================================== */

#define ZDT_TX_TIMEOUT     50000U
#define ZDT_RX_QUEUE_SIZE  4U

typedef struct {
    uint8_t data[ZDT_MAX_FRAME_LEN];
    uint8_t len;
} ZDT_RxFrame_t;

static volatile uint8_t zdt_rx_buf[ZDT_MAX_FRAME_LEN];
static volatile uint8_t zdt_rx_cnt = 0;
static volatile ZDT_RxFrame_t zdt_rx_queue[ZDT_RX_QUEUE_SIZE];
static volatile uint8_t zdt_rx_head = 0;
static volatile uint8_t zdt_rx_tail = 0;
static volatile uint8_t zdt_rx_num = 0;

static bool ZDT_IsValidFrame(const volatile uint8_t *frame, uint8_t len)
{
    if (len < 3U) {
        return false;
    }

    if (frame[0] != ZDT_DEFAULT_ADDR && frame[0] != 0x00U) {
        return false;
    }

    if (frame[len - 1U] != ZDT_DEFAULT_CHECK) {
        return false;
    }

    return true;
}

static void ZDT_PushRxFrameFromISR(const volatile uint8_t *frame, uint8_t len)
{
    if (!ZDT_IsValidFrame(frame, len)) {
        Motor1.rxDropCount++;
        return;
    }

    if (zdt_rx_num >= ZDT_RX_QUEUE_SIZE) {
        zdt_rx_tail = (uint8_t)((zdt_rx_tail + 1U) % ZDT_RX_QUEUE_SIZE);
        zdt_rx_num--;
        Motor1.rxDropCount++;
    }

    for (uint8_t i = 0; i < len; i++) {
        zdt_rx_queue[zdt_rx_head].data[i] = frame[i];
    }
    zdt_rx_queue[zdt_rx_head].len = len;
    zdt_rx_head = (uint8_t)((zdt_rx_head + 1U) % ZDT_RX_QUEUE_SIZE);
    zdt_rx_num++;

    Motor1.rxReady = true;
    Motor1.rxFrameCount++;
}

/**
 * @brief 串口发送底层接口
 * @param data 待发送的数据指针
 * @param len  数据长度
 * @return true: 发送成功, false: UART 超时或参数错误
 */
bool ZDT_UART_Send(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U) {
        return false;
    }

    for (uint16_t i = 0; i < len; i++) {
        uint32_t timeout = ZDT_TX_TIMEOUT;

        while (!DL_UART_Main_isTXFIFOEmpty(UART_ZDT_INST)) {
            if (--timeout == 0U) {
                return false;
            }
        }

        DL_UART_Main_transmitData(UART_ZDT_INST, data[i]);
    }

    uint32_t timeout = ZDT_TX_TIMEOUT;
    while (DL_UART_Main_isBusy(UART_ZDT_INST)) {
        if (--timeout == 0U) {
            return false;
        }
    }

    return true;
}

/**
 * @brief 初始化张大头电机通信
 * 开启 NVIC 中断，确保能收到闭环反馈数据
 */
void ZDT_Init(void)
{
    zdt_rx_cnt  = 0;
    zdt_rx_head = 0;
    zdt_rx_tail = 0;
    zdt_rx_num  = 0;
    Motor1.rxReady = false;

    NVIC_ClearPendingIRQ(UART_ZDT_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_ZDT_INST_INT_IRQN);
}

/* ====================================================================
 * 中断接收与解析逻辑
 * ==================================================================== */

/**
 * @brief 读取一帧完整反馈数据
 */
bool ZDT_ReadFrame(uint8_t *frame, uint8_t *len)
{
    if (frame == NULL || len == NULL) {
        return false;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (zdt_rx_num == 0U) {
        Motor1.rxReady = false;
        if (!primask) {
            __enable_irq();
        }
        return false;
    }

    uint8_t frame_len = zdt_rx_queue[zdt_rx_tail].len;
    for (uint8_t i = 0; i < frame_len; i++) {
        frame[i] = zdt_rx_queue[zdt_rx_tail].data[i];
    }
    *len = frame_len;

    zdt_rx_tail = (uint8_t)((zdt_rx_tail + 1U) % ZDT_RX_QUEUE_SIZE);
    zdt_rx_num--;
    Motor1.rxReady = (zdt_rx_num > 0U);

    if (!primask) {
        __enable_irq();
    }

    return true;
}

/**
 * @brief 串口接收中断服务函数 
 */
void UART_ZDT_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_ZDT_INST)) {
        case DL_UART_MAIN_IIDX_RX: {
            uint8_t rx_data = DL_UART_Main_receiveData(UART_ZDT_INST);

            if (zdt_rx_cnt == 0U && rx_data != ZDT_DEFAULT_ADDR && rx_data != 0x00U) {
                Motor1.rxDropCount++;
                break;
            }

            zdt_rx_buf[zdt_rx_cnt++] = rx_data;

            if (rx_data == ZDT_DEFAULT_CHECK) {
                ZDT_PushRxFrameFromISR(zdt_rx_buf, zdt_rx_cnt);
                zdt_rx_cnt = 0;
            } else if (zdt_rx_cnt >= ZDT_MAX_FRAME_LEN) {
                zdt_rx_cnt = 0;
                Motor1.rxDropCount++;
            }
            break;
        }

        default:
            break;
    }
}

/* ====================================================================
 * 基础控制层
 * ==================================================================== */

/**
 * @brief 电机使能控制
 * @param addr 电机地址 (默认0x01)
 * @param enable true:锁轴, false:松开
 * @param sync true:多机同步缓存, false:立即执行
 */
bool ZDT_Enable(uint8_t addr, bool enable, bool sync)
{
    uint8_t tx_buf[6];
    tx_buf[0] = addr;                
    tx_buf[1] = 0xF3;                 // 功能码
    tx_buf[2] = 0xAB;                 // 辅助码
    tx_buf[3] = enable ? 0x01 : 0x00; // 状态
    tx_buf[4] = sync ? 0x01 : 0x00;   // 多机同步标志
    tx_buf[5] = ZDT_DEFAULT_CHECK;    // 0x6B校验码
    
    return ZDT_UART_Send(tx_buf, 6);
}

/**
 * @brief 读取当前电机位置
 * @param addr 电机地址
 */
bool ZDT_ReadPosition(uint8_t addr)
{
    uint8_t tx_buf[3];
    tx_buf[0] = addr;                
    tx_buf[1] = 0x36;                 // 读取位置功能码
    tx_buf[2] = ZDT_DEFAULT_CHECK;   
    
    return ZDT_UART_Send(tx_buf, 3);
}

/**
 * @brief 绝对位置运动控制
 * @param addr 电机地址
 * @param dir 方向: ZDT_DIR_CW 或 ZDT_DIR_CCW
 * @param rpm 转速 (0~3000)
 * @param acc 加速度档位 (0~255)
 * @param pulses 目标位置对应的脉冲数
 */
bool ZDT_MoveAbsolute(uint8_t addr, ZDT_Dir_t dir, uint16_t rpm, uint8_t acc, uint32_t pulses)
{
    uint8_t tx_buf[13];
    tx_buf[0] = addr;
    tx_buf[1] = 0xFD;                 // 位置控制功能码
    tx_buf[2] = (uint8_t)dir;        
    
    // 转速 2Bytes (大端模式)
    tx_buf[3] = (uint8_t)(rpm >> 8); 
    tx_buf[4] = (uint8_t)(rpm & 0xFF);
    
    // 加速度 1Byte
    tx_buf[5] = acc;
    
    // 目标脉冲 4Bytes (大端模式)
    tx_buf[6] = (uint8_t)(pulses >> 24);
    tx_buf[7] = (uint8_t)(pulses >> 16);
    tx_buf[8] = (uint8_t)(pulses >> 8);
    tx_buf[9] = (uint8_t)(pulses & 0xFF);
    
    // 运动模式: 绝对位置
    tx_buf[10] = ZDT_MODE_ABS;     
    tx_buf[11] = 0x00;  // 立即执行标志
    tx_buf[12] = ZDT_DEFAULT_CHECK;  
    
    return ZDT_UART_Send(tx_buf, 13);
}

/**
 * @brief 紧急停止
 * @param addr 电机地址
 * @param sync 是否使用多机同步 (通常填 false 立即执行)
 */
bool ZDT_Stop(uint8_t addr, bool sync)
{
    uint8_t tx_buf[5];
    tx_buf[0] = addr;                 
    tx_buf[1] = 0xFE;                 // 急停功能码
    tx_buf[2] = 0x98;                 // 辅助码
    tx_buf[3] = sync ? 0x01 : 0x00;   // 同步标志
    tx_buf[4] = ZDT_DEFAULT_CHECK;    
    
    return ZDT_UART_Send(tx_buf, 5);
}

/**
 * @brief 恒速模式运行
 * @param addr 电机地址
 * @param dir 方向
 * @param rpm 转速 (0~3000)
 * @param acc 加速度档位 (0~255)
 * @param sync 是否多机同步
 */
bool ZDT_RunVelocity(uint8_t addr, ZDT_Dir_t dir, uint16_t rpm, uint8_t acc, bool sync)
{
    uint8_t tx_buf[8];
    tx_buf[0] = addr;                 
    tx_buf[1] = 0xF6;                 // 速度模式功能码
    tx_buf[2] = (uint8_t)dir;         
    
    // 转速 2Bytes (大端)
    tx_buf[3] = (uint8_t)(rpm >> 8);  
    tx_buf[4] = (uint8_t)(rpm & 0xFF);
    
    tx_buf[5] = acc;                  // 加速度
    tx_buf[6] = sync ? 0x01 : 0x00;   // 同步标志
    tx_buf[7] = ZDT_DEFAULT_CHECK;    
    
    return ZDT_UART_Send(tx_buf, 8);
}

/**
 * @brief 相对位置运动控制
 */
bool ZDT_MoveRelative(uint8_t addr, ZDT_Dir_t dir, uint16_t rpm, uint8_t acc, uint32_t pulses)
{
    uint8_t tx_buf[13];
    tx_buf[0] = addr;
    tx_buf[1] = 0xFD;                 // 位置模式功能码
    tx_buf[2] = (uint8_t)dir;         
    
    tx_buf[3] = (uint8_t)(rpm >> 8);  
    tx_buf[4] = (uint8_t)(rpm & 0xFF);
    tx_buf[5] = acc;
    
    // 脉冲数 4Bytes (大端)
    tx_buf[6] = (uint8_t)(pulses >> 24);
    tx_buf[7] = (uint8_t)(pulses >> 16);
    tx_buf[8] = (uint8_t)(pulses >> 8);
    tx_buf[9] = (uint8_t)(pulses & 0xFF);
    
    // 运动模式: 0x02 代表相对当前实际位置运动
    tx_buf[10] = ZDT_MODE_REL_CUR;      
    tx_buf[11] = 0x00;  // 立即执行
    tx_buf[12] = ZDT_DEFAULT_CHECK;   
    
    return ZDT_UART_Send(tx_buf, 13);
}

/**
 * @brief 清零当前位置 (设为机械原点)
 * @param addr 电机地址
 */
bool ZDT_SetCurrentPositionAsZero(uint8_t addr)
{
    uint8_t tx_buf[4];
    tx_buf[0] = addr;                 
    tx_buf[1] = 0x93;                 // 清零功能码
    tx_buf[2] = 0x88;                 // 辅助码
    tx_buf[3] = ZDT_DEFAULT_CHECK;    
    
    return ZDT_UART_Send(tx_buf, 4);
}

#else
/**
 * @brief 哑函数 ISR：防止孤儿中断触发 Default Handler 死机
 */
void UART_ZDT_INST_IRQHandler(void)
{
    DL_UART_Main_clearInterruptStatus(UART_ZDT_INST, DL_UART_Main_getPendingInterrupt(UART_ZDT_INST));
    DL_UART_Main_receiveData(UART_ZDT_INST); // 读空 RX 寄存器
}
#endif // USE_ZDT_STEPPER
