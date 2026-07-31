#include "mahony.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

// 算法常量 (根据电赛实际情况可微调 Kp 和 Ki)
#define twoKpDef    (2.0f * 0.5f)   // 比例增益，控制加速度计收敛速度
#define twoKiDef    (2.0f * 0.005f)   // 积分增益，控制陀螺仪零偏消除

static float twoKp = twoKpDef;
static float twoKi = twoKiDef;

void Mahony_SetKp(float kp) {
    twoKp = 2.0f * kp;
}

void Mahony_SetKi(float ki) {
    twoKi = 2.0f * ki;
}

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 四元数
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // 积分误差

void Mahony_ResetIntegral(void) {
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
}

// 快速计算平方根倒数 (经典的 Fast Inverse Square Root)
// 使用 memcpy 进行类型双关, 避免 strict-aliasing 未定义行为
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    int32_t i;
    memcpy(&i, &y, sizeof(i));
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void Mahony_Init(float sampleFrequency) {
    (void)sampleFrequency;  /* dt 现在每帧传入, 不再依赖固定频率 */
}

void Mahony_UpdateIMU(float gx, float gy, float gz,
                      float ax, float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 如果加速度计无有效数据则忽略
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // 加速度计数据归一化
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 根据当前四元数推算重力分量
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 叉乘得到误差
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 误差积分 (仅 X/Y 轴; Z 轴不积分, 因为加速度计无法观测 Yaw)
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            // integralFBz 不累积: 加速度计无法观测绕重力轴旋转,
            // halfez 中只有噪声, 积分会导致 Yaw 漂移
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // 比例反馈
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // 四元数积分步长 (使用真实 dt, 严格对齐物理时间)
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // 四元数归一化
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void Mahony_GetEulerAngles(float *roll, float *pitch, float *yaw) {
    // 将四元数转换为欧拉角 (raw yaw ∈ [-180°, +180°])
    *roll  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29578f;
    *pitch = asinf(-2.0f * (q1*q3 - q0*q2)) * 57.29578f;
    *yaw   = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * 57.29578f;

    // Yaw unwrap: 消除 ±180° 边界跳变, 使角度连续可微
    //   atan2f 输出在 [-180°,+180°], 跨越边界时会产生 ~358° 的虚假跳变
    //   通过检测帧间跳变方向, 累加 ±360° 补偿, 输出连续角度
    //   例如: +179° → -179° 会被解缠为 +179° → +181° (仅 2° 物理变化)
    static float prev_raw_yaw = 0.0f;
    static int   wrap_count   = 0;
    static bool  initialized  = false;

    if (!initialized) {
        prev_raw_yaw = *yaw;
        initialized  = true;
    } else {
        float delta = *yaw - prev_raw_yaw;
        if (delta > 180.0f) {
            wrap_count--;          // 正向跨越: -179 → +179, raw 跳了 +358°
        } else if (delta < -180.0f) {
            wrap_count++;          // 反向跨越: +179 → -179, raw 跳了 -358°
        }
        prev_raw_yaw = *yaw;
    }
    *yaw += (float)wrap_count * 360.0f;
}
