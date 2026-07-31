#ifndef MAHONY_H_
#define MAHONY_H_

#include <stdint.h>

// 算法初始化 (保留接口兼容性, dt 现在每帧传入)
void Mahony_Init(float sampleFrequency);

// 更新 IMU 数据 (参数：陀螺仪弧度/秒, 加速度计 g, 真实时间差 秒)
//   dt: 本次距上次调用的真实时间差 (秒), 由硬件定时器测量
//   静态 dt (固定频率) 在旋转时会导致积分失准, 动态 dt 严格对齐物理时间
void Mahony_UpdateIMU(float gx, float gy, float gz,
                      float ax, float ay, float az, float dt);

// 获取欧拉角 (单位：度)
void Mahony_GetEulerAngles(float *roll, float *pitch, float *yaw);

// 调整增益
void Mahony_SetKp(float kp);
void Mahony_SetKi(float ki);

// 复位积分项 (静止锁定时调用, 防止历史累积误差继续注入)
void Mahony_ResetIntegral(void);

#endif /* MAHONY_H_ */
