#include "quaternion.h"
#include "zf_device_imu963ra.h"
#include <math.h>

// 传感器单位转换参数（根据实际传感器量程调整）
// 加速度计：±8g，4096 LSB/g，转换为 m/s²
#define ACC_SCALE   (9.8f / 4096.0f)
// 陀螺仪：±2000°/s，16.4 LSB/°/s，转换为 rad/s
#define GYRO_SCALE  ((3.14159265358979f / 180.0f) / 16.4f)

// 快速平方根倒数（用于归一化）
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// 静态滤波器实例
static quaternion_filter_t filter = {
    .q0 = 1.0f, .q1 = 0.0f, .q2 = 0.0f, .q3 = 0.0f,
    .integralFBx = 0.0f, .integralFBy = 0.0f, .integralFBz = 0.0f,
    .twoKp = 2.0f, .twoKi = 0.0f    // 默认值，会在初始化时覆盖
};

// Mahony AHRS 更新（包含磁力计）
static void MahonyAHRSupdate(float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float mx, float my, float mz,
                             float dt) {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, hz, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float ex, ey, ez;
    float qa, qb, qc;

    // 归一化加速度计和磁力计
    recipNorm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    recipNorm = invSqrt(mx*mx + my*my + mz*mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // 四元数相关中间量
    q0q0 = filter.q0 * filter.q0;
    q0q1 = filter.q0 * filter.q1;
    q0q2 = filter.q0 * filter.q2;
    q0q3 = filter.q0 * filter.q3;
    q1q1 = filter.q1 * filter.q1;
    q1q2 = filter.q1 * filter.q2;
    q1q3 = filter.q1 * filter.q3;
    q2q2 = filter.q2 * filter.q2;
    q2q3 = filter.q2 * filter.q3;
    q3q3 = filter.q3 * filter.q3;

    // 重力向量在体坐标系中的表示（加速度计参考）
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;

    // 导航坐标系下的地磁场向量
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // 地磁参考方向（假设磁场在导航系中仅有X和Z分量）
    bx = sqrtf(hx*hx + hy*hy);
    bz = hz;

    // 预测的地磁场向量在体坐标系中的表示
    halfwx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
    halfwy = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
    halfwz = 2.0f * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));

    // 误差计算（叉积）
    ex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    ey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    ez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    // 积分误差
    filter.integralFBx += filter.twoKi * ex * dt;
    filter.integralFBy += filter.twoKi * ey * dt;
    filter.integralFBz += filter.twoKi * ez * dt;

    // 应用反馈修正陀螺仪
    gx += filter.twoKp * ex + filter.integralFBx;
    gy += filter.twoKp * ey + filter.integralFBy;
    gz += filter.twoKp * ez + filter.integralFBz;

    // 四元数微分方程
    filter.q0 += 0.5f * (-filter.q1 * gx - filter.q2 * gy - filter.q3 * gz) * dt;
    filter.q1 += 0.5f * ( filter.q0 * gx + filter.q2 * gz - filter.q3 * gy) * dt;
    filter.q2 += 0.5f * ( filter.q0 * gy - filter.q1 * gz + filter.q3 * gx) * dt;
    filter.q3 += 0.5f * ( filter.q0 * gz + filter.q1 * gy - filter.q2 * gx) * dt;

    // 四元数归一化
    recipNorm = invSqrt(filter.q0*filter.q0 + filter.q1*filter.q1 +
                        filter.q2*filter.q2 + filter.q3*filter.q3);
    filter.q0 *= recipNorm;
    filter.q1 *= recipNorm;
    filter.q2 *= recipNorm;
    filter.q3 *= recipNorm;
}

// 初始化滤波器
void quaternion_init(float kp, float ki) {
    filter.q0 = 1.0f;
    filter.q1 = 0.0f;
    filter.q2 = 0.0f;
    filter.q3 = 0.0f;
    filter.integralFBx = 0.0f;
    filter.integralFBy = 0.0f;
    filter.integralFBz = 0.0f;
    filter.twoKp = 2.0f * kp;
    filter.twoKi = 2.0f * ki;
}

// 更新四元数（在中断中调用，使用全局IMU数据）
void quaternion_update(void) {
    static uint32_t last_time = 0;
    float dt = 0.1f;   // 固定10ms周期

    // 获取原始数据并转换单位
    // 对陀螺仪进行粗糙滤波
    float gx = (float)(imu963ra_gyro_x / 100 * 100) * GYRO_SCALE;
    float gy = (float)(imu963ra_gyro_y / 100 * 100) * GYRO_SCALE;
    float gz = (float)(imu963ra_gyro_z / 100 * 100) * GYRO_SCALE;
    float ax = (float)(imu963ra_acc_x / 100 * 100) * ACC_SCALE;
    float ay = (float)(imu963ra_acc_y / 100 * 100) * ACC_SCALE;
    float az = (float)(imu963ra_acc_z / 100 * 100) * ACC_SCALE;
    float mx = (float)(imu963ra_mag_x / 100 * 100);   // 磁力计直接使用原始值，归一化后不影响方向
    float my = (float)(imu963ra_mag_y / 100 * 100);
    float mz = (float)(imu963ra_mag_z / 100 * 100);

    // 执行Mahony更新
    MahonyAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
}

// 从四元数计算欧拉角（弧度）
void quaternion_get_euler(float *roll, float *pitch, float *yaw) {
    // 公式：roll = atan2(2(q0q1+q2q3), 1-2(q1^2+q2^2))
    //       pitch = asin(2(q0q2 - q3q1))
    //       yaw   = atan2(2(q0q3+q1q2), 1-2(q2^2+q3^2))
    float q0 = filter.q0;
    float q1 = filter.q1;
    float q2 = filter.q2;
    float q3 = filter.q3;

    *roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    *pitch = asinf(2.0f*(q0*q2 - q3*q1));
    *yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
}
