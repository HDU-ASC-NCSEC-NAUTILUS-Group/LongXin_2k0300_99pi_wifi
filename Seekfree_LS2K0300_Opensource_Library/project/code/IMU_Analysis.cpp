/********************************************************************************************************************
* IMU姿态解算
********************************************************************************************************************/
#include "zf_device_imu963ra.h"
#include "zf_driver_file.h"

#include "IMU_Analysis.h"
#include "MahonyAHRS.h"
#include <math.h>
#include <chrono>


// 全局变量
volatile float Yaw_Result = 0.0f;    // 偏航角（Yaw）
volatile float Roll_Result = 0.0f;   // 横滚角（Roll）
volatile float Pitch_Result = 0.0f;  // 俯仰角（Pitch）
// IMU963RA 分析使能标志位
volatile uint8_t IMU963RA_analysis_enable = 0;

// 坐标系配置：根据您的IMU安装方向调整
// 1 = 正常方向，-1 = 反方向
// 加速度计和陀螺仪的坐标系
#define IMU_AXIS_X_SIGN  1
#define IMU_AXIS_Y_SIGN  1
#define IMU_AXIS_Z_SIGN  -1  // Z轴取反，因为重力方向是-Z

// 磁力计的坐标系（可能需要单独调整）
#define IMU_MAG_X_SIGN   1
#define IMU_MAG_Y_SIGN   1
#define IMU_MAG_Z_SIGN   1

// 磁力计轴旋转配置
// 0: 不旋转
// 1: 顺时针旋转 90 度
// 2: 顺时针旋转 180 度
// 3: 顺时针旋转 270 度
#define IMU_MAG_AXIS_ROTATE  1

// IIO子系统读取的scale值（如果没有scale文件，则使用硬编码的IMU963RA参数）
static float imu_acc_scale = 0.0f;      // 加速度计scale (m/s² per LSB)
static float imu_gyro_scale = 0.0f;     // 陀螺仪scale (rad/s per LSB)
static float imu_mag_scale = 0.0f;      // 磁力计scale (uT per LSB)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化IMU scale值（使用硬编码的IMU963RA参数）
// 使用示例     IMU963RA_Init_Scale();
//-------------------------------------------------------------------------------------------------------------------
void IMU963RA_Init_Scale(void)
{
    // IMU963RA 典型参数：
    // 加速度计：±8g, 16-bit
    // 陀螺仪：±2000°/s, 16-bit
    // 磁力计：±4912 μT, 16-bit
    
    // 加速度计 scale: 8g / 32768 * 9.81 m/s²/g
    imu_acc_scale = (8.0f / 32768.0f) * 9.81f;
    
    // 陀螺仪 scale: 2000°/s / 32768 * π/180 rad/°
    imu_gyro_scale = (2000.0f / 32768.0f) * (M_PI / 180.0f);
    
    // 磁力计 scale: 4912 μT / 32768
    #if (IMU_ANALYSIS_USE_MAG == 1)
    imu_mag_scale = 4912.0f / 32768.0f;
    #endif
}


// 零飘校准校准需要的样本数
#define CALIB_TARGET_SAMPLES    400  
// 枚举定义校准状态
typedef enum {
    CALIB_STATE_SPARE   = 0,          // 未开始
    CALIB_STATE_RUNNING = 1,          // 校准中
    CALIB_STATE_DONE    = 2           // 已校准
} CalibState_t;
static CalibState_t calib_state = CALIB_STATE_SPARE;
// 样本数量
static uint16_t calib_count = 0;






// 选择九轴数据的解算
#if (IMU_ANALYSIS_USE_MAG == 1)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 获取九轴原始数据
// 使用示例     imu963ra_get_data();                                              // 执行该函数后，直接查看对应的变量即可
// 备注信息     定时器定时中断定时调用该函数，获取 IMU963RA 原始数据
//-------------------------------------------------------------------------------------------------------------------
// IMU963RA 原始数据变量
// imu963ra_acc_x           imu963ra_acc_y          imu963ra_acc_z
// imu963ra_gyro_x          imu963ra_gyro_y         imu963ra_gyro_z
// imu963ra_mag_x           imu963ra_mag_y          imu963ra_mag_z  
void imu963ra_update_data(void)
{
    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963ra_get_mag();
}

// 条件编译选项
#define ENABLE_FULL_EULER    1   // 1: 计算全部欧拉角, 0: 只计算Yaw角


/*******************************************************************************************************************/
/*[S] 零飘校准 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

static int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
static float gyro_off_x = 0, gyro_off_y = 0, gyro_off_z = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 校准初始化
// 使用示例     IMU963RA_Calibration_Start();                                              // 开始校准
// 备注信息     校准状态机启用前的操作，用于初始化校准状态
//-------------------------------------------------------------------------------------------------------------------
void IMU963RA_Calibration_Start(void)
{
    calib_state = CALIB_STATE_RUNNING;
    calib_count = 0;
    sum_gx = 0;
    sum_gy = 0;
    sum_gz = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 校准状态机
// 使用示例     IMU963RA_Calibration_Check();                                              // 检查校准状态
// 返回校准状态（数字对应CalibState_t的枚举定义）
// 0 未校准
// 1 校准中
// 2 已校准
// 备注信息     调用方法特殊，可以参考调试文件的代码调用
//-------------------------------------------------------------------------------------------------------------------
int8_t IMU963RA_Calibration_Check(void)
{
    if(calib_state == CALIB_STATE_DONE)
    {
        return 2;
    }

    if(calib_state == CALIB_STATE_SPARE)
    {
        return 0;
    }
    
    // 检查是否允许收集数据
    if(IMU963RA_analysis_enable)
    {
        imu963ra_update_data();

        // 收集数据（仅陀螺仪）
        sum_gx += imu963ra_gyro_x;
        sum_gy += imu963ra_gyro_y;
        sum_gz += imu963ra_gyro_z;
        calib_count++;

        IMU963RA_analysis_enable = 0;
        
        if(calib_count >= CALIB_TARGET_SAMPLES)
        {
            gyro_off_x = (float)sum_gx / CALIB_TARGET_SAMPLES;
            gyro_off_y = (float)sum_gy / CALIB_TARGET_SAMPLES;
            gyro_off_z = (float)sum_gz / CALIB_TARGET_SAMPLES;
            
            calib_state = CALIB_STATE_DONE;
        }
    }
    
    return 1;
}

/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 零飘校准 [E]*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 姿态解算 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数转欧拉角
// 使用示例     QuaternionToEuler();                                              // 四元数转欧拉角
// 备注信息     将MahonyAHRS的四元数结果转换为欧拉角
//-------------------------------------------------------------------------------------------------------------------
void QuaternionToEuler(void)
{
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    
    #if ENABLE_FULL_EULER
    // 计算Roll（横滚角）
    Roll_Result = atan2f(2.0f * (q0q1 + q2q3), q0q0 - q1q1 - q2q2 + q3q3) * 180.0f / M_PI;
    
    // 计算Pitch（俯仰角）
    Pitch_Result = asinf(-2.0f * (q1q3 - q0q2)) * 180.0f / M_PI;
    #endif
    
    // 计算Yaw（偏航角）
    Yaw_Result = atan2f(2.0f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3) * 180.0f / M_PI;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 姿态解算
// 使用示例     IMU963RA_Analysis_Update();                                              // 执行姿态解算
// 备注信息     定时调用该函数，更新姿态解算结果
//-------------------------------------------------------------------------------------------------------------------
void IMU963RA_Analysis_Update(void)
{
    // 只有在校准完成后才执行姿态解算
    if (calib_state != CALIB_STATE_DONE)
    {
        return;  // 未校准完成，不执行解算
    }
    
    // 使用从IIO子系统读取的真实scale值（应用坐标系配置）
    float ax = imu963ra_acc_x * IMU_AXIS_X_SIGN * imu_acc_scale;
    float ay = imu963ra_acc_y * IMU_AXIS_Y_SIGN * imu_acc_scale;
    float az = imu963ra_acc_z * IMU_AXIS_Z_SIGN * imu_acc_scale;

    // 应用零飘校准，陀螺仪scale已经是rad/s（IIO子系统单位）（应用坐标系配置）
    float gx = (imu963ra_gyro_x - gyro_off_x) * IMU_AXIS_X_SIGN * imu_gyro_scale;
    float gy = (imu963ra_gyro_y - gyro_off_y) * IMU_AXIS_Y_SIGN * imu_gyro_scale;
    float gz = (imu963ra_gyro_z - gyro_off_z) * IMU_AXIS_Z_SIGN * imu_gyro_scale;
    
    // 磁力计scale已经是uT（IIO子系统单位）（应用磁力计特殊坐标系配置）
    // 注意：磁力计不做零飘校准！
    float mx0 = imu963ra_mag_x * IMU_MAG_X_SIGN * imu_mag_scale;
    float my0 = imu963ra_mag_y * IMU_MAG_Y_SIGN * imu_mag_scale;
    float mz0 = imu963ra_mag_z * IMU_MAG_Z_SIGN * imu_mag_scale;
    
    // 应用磁力计轴旋转
    float mx, my, mz;
    #if IMU_MAG_AXIS_ROTATE == 0
    mx = mx0;
    my = my0;
    mz = mz0;
    #elif IMU_MAG_AXIS_ROTATE == 1
    mx = -my0;  // 顺时针旋转 90 度
    my = mx0;
    mz = mz0;
    #elif IMU_MAG_AXIS_ROTATE == 2
    mx = -mx0;  // 顺时针旋转 180 度
    my = -my0;
    mz = mz0;
    #elif IMU_MAG_AXIS_ROTATE == 3
    mx = my0;   // 顺时针旋转 270 度
    my = -mx0;
    mz = mz0;
    #endif
    
    // 调用Mahony AHRS算法
    MahonyAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
    
    // 四元数转欧拉角
    QuaternionToEuler();
}

/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 姿态解算 [E]*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 工具函数 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/
void IMU963RA_Get_Calibrated_Data(float *acc_x, float *acc_y, float *acc_z, 
                                        float *gyro_x, float *gyro_y, float *gyro_z, 
                                        float *mag_x, float *mag_y, float *mag_z)
{
    if (calib_state != CALIB_STATE_DONE)
    {
        return;  // 未校准完成，不执行校准
    }

    // 加速度无零飘
    *acc_x = (float)imu963ra_acc_x;
    *acc_y = (float)imu963ra_acc_y;
    *acc_z = (float)imu963ra_acc_z;
    
    // 陀螺仪赋值
    *gyro_x = (float)imu963ra_gyro_x - gyro_off_x;
    *gyro_y = (float)imu963ra_gyro_y - gyro_off_y;
    *gyro_z = (float)imu963ra_gyro_z - gyro_off_z;
    
    // 磁力计赋值（不做零飘校准）
    *mag_x = (float)imu963ra_mag_x;
    *mag_y = (float)imu963ra_mag_y;
    *mag_z = (float)imu963ra_mag_z;
} 
// 简化版：直接重置四元数为初始状态
void IMU963RA_Reset_Yaw(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    
    // 同时重置欧拉角结果
    Yaw_Result = 0.0f;
    Roll_Result = 0.0f;
    Pitch_Result = 0.0f;
}  
/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 工具函数 [E]*/
/*******************************************************************************************************************/

#endif






// 六轴数据的解算
#if (IMU_ANALYSIS_USE_MAG == 0)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 获取六轴原始数据
// 使用示例     imu963ra_get_data();                                              // 执行该函数后，直接查看对应的变量即可
// 备注信息     定时器定时中断定时调用该函数，获取 IMU963RA 原始数据
//-------------------------------------------------------------------------------------------------------------------
// IMU963RA 原始数据变量
// imu963ra_acc_x           imu963ra_acc_y          imu963ra_acc_z
// imu963ra_gyro_x          imu963ra_gyro_y         imu963ra_gyro_z
void imu963ra_update_data(void)
{
    imu963ra_get_acc();
    imu963ra_get_gyro();
}

/*******************************************************************************************************************/
/*[S] 零飘校准 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/
// 上次时间戳
static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();

// 获取实际采样时间间隔
static float Get_Real_dt(void)
{
    auto current_time = std::chrono::steady_clock::now();
    auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();
    
    // 限制dt范围
    float dt = (float)time_diff / 1000.0f;
    if (dt > 0.05f) dt = 0.05f;
    if (dt < 0.001f) dt = 0.001f;
    
    last_time = current_time;
    return dt;
}

// 零飘校准相关
static int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
static float gyro_off_x = 0, gyro_off_y = 0, gyro_off_z = 0;

// 开始校准
void IMU963RA_Calibration_Start(void)
{
    calib_state = CALIB_STATE_RUNNING;
    calib_count = 0;
    sum_gx = 0;
    sum_gy = 0;
    sum_gz = 0;
}

// 校准状态检查
int8_t IMU963RA_Calibration_Check(void)
{
    if(calib_state == CALIB_STATE_DONE)
    {
        return 2;
    }

    if(calib_state == CALIB_STATE_SPARE)
    {
        return 0;
    }
    
    // 检查是否允许收集数据
    if(IMU963RA_analysis_enable)
    {
        imu963ra_update_data();
        // 收集数据
        sum_gx += imu963ra_gyro_x;
        sum_gy += imu963ra_gyro_y;
        sum_gz += imu963ra_gyro_z;
        calib_count++;

        IMU963RA_analysis_enable = 0;
        
        if(calib_count >= CALIB_TARGET_SAMPLES)
        {
            gyro_off_x = (float)sum_gx / CALIB_TARGET_SAMPLES;
            gyro_off_y = (float)sum_gy / CALIB_TARGET_SAMPLES;
            gyro_off_z = (float)sum_gz / CALIB_TARGET_SAMPLES;
            
            calib_state = CALIB_STATE_DONE;
        }
    }
    
    return 1;
}
/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 零飘校准 [E]*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 卡尔曼滤波 [S]------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 卡尔曼滤波开关：1=使用卡尔曼滤波，0=使用互补滤波
#define USE_KALMAN_FILTER 1

//Q_angle（角度过程噪声）
//数值越大，越信任陀螺仪（动态响应快，但噪声大）；
//数值越小，越信任加速度计（静态准，但动态延迟大）；
//平衡车推荐初始值：0.001f~0.005f

//Q_bias（陀螺仪零飘过程噪声）
//数值越大，对零飘的动态补偿越灵敏，但容易引入抖动；
//数值越小，零飘补偿越平滑，但响应慢；
//平衡车推荐初始值：0.003f~0.01f

//R_measure（加速度计测量噪声）
//数值越大，越不信任加速度计（抗振动干扰能力强，但静态误差大）；
//数值越小，越信任加速度计（静态准，但易受动态加速度干扰）；
//平衡车推荐初始值：0.03f~0.1f

// 卡尔曼滤波结构体定义
typedef struct {
    float Q_angle;        // 角度过程噪声协方差
    float Q_bias;         // 偏差过程噪声协方差
    float R_measure;      // 测量噪声协方差
    
    float angle;          // 滤波后角度
    float bias;           // 陀螺仪偏差
    float rate;           // 去偏后的角速度
    
    float P[2][2];        // 误差协方差矩阵
} KalmanFilter;

// 定义Roll/Pitch轴卡尔曼滤波器实例
static KalmanFilter kf_roll = {0};
static KalmanFilter kf_pitch = {0};

// 卡尔曼滤波初始化函数
static void Kalman_Init(KalmanFilter* kf, float Q_angle, float Q_bias, float R_measure)
{
    kf->Q_angle = Q_angle;
    kf->Q_bias = Q_bias;
    kf->R_measure = R_measure;

    kf->angle = 0.0f;
    kf->bias = 0.0f;
    
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

// 卡尔曼滤波计算核心函数
static float Kalman_Calculate(KalmanFilter* kf, float newAngle, float newRate, float dt)
{
    // 预测步骤
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;
    
    // 更新误差协方差矩阵
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;
    
    // 更新步骤
    float S = kf->P[0][0] + kf->R_measure;   // 创新协方差
    float K[2];                               // 卡尔曼增益
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;
    
    float y = newAngle - kf->angle;          // 角度残差
    kf->angle += K[0] * y;                   // 更新角度
    kf->bias += K[1] * y;                    // 更新偏差
    
    // 更新后验误差协方差
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;
    
    return kf->angle;
}

// 动态调整R_measure（根据加速度计模长判断运动状态）
static float Get_Dynamic_Rmeasure(float ax, float ay, float az)
{
    // 计算加速度模长（使用真实的scale值转换到m/s²）
    float acc_scaled = ax * imu_acc_scale;
    float ay_scaled = ay * imu_acc_scale;
    float az_scaled = az * imu_acc_scale;
    
    // 计算模长（理想值为1g ≈ 9.81 m/s²）
    float acc_mag = sqrtf(acc_scaled*acc_scaled + ay_scaled*ay_scaled + az_scaled*az_scaled);
    
    // 归一化到1g参考系（9.81 m/s²）
    acc_mag = acc_mag / 9.81f;
    
    // 静态（模长接近1g）：信任加速度计，R小
    if(fabs(acc_mag - 1.0f) < 0.1f)
    {
        return 0.03f;  // 静态R值
    }
    // 动态（模长偏离1g）：不信任加速度计，R大
    else
    {
        return 0.3f;   // 动态R值
    }
}
/*******************************************************************************************************************/
/*------------------------------------------------------------------------------------------------[E] 卡尔曼滤波 [E]*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 姿态解算+滤波 [S]----------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 中间变量
static float RollAcc = 0.0f;         // 加速度计计算的横滚角
static float Roll = 0.0f;            // 融合后的横滚角

static float Yaw = 0.0f;             // 偏航角

static float PitchAcc = 0.0f;        // 加速度计算的俯仰角
static float Pitch = 0.0f;            // 融合后的俯仰角

static float Roll_Temp = 0.0f;        // 横滚角 中间处理值
static float Yaw_Temp = 0.0f;         // 偏航角 中间处理值
static float Pitch_Temp = 0.0f;       // 俯仰角 中间处理值

// 低通滤波系数（0.2 = 强滤波，0.5 = 中等，0.8 = 弱滤波）
#define MPU6050_LOW_PASS_FILTER 0.3f
// 输出死区系数
#define MPU6050_OUTPUT_DEAD_ZONE 0.05f

// 固化解算系数
// 弧度转角度
const float mpu6050_const_data1 = (1.0f / M_PI) * 180.0f;
// rad/s 转 °/s 的转换系数
const float rad_to_deg_per_s = (180.0f / M_PI);

// IMU963RA 姿态解算
void IMU963RA_Analysis_Update(void)
{
    // 只有在校准完成后才执行姿态解算
    if (calib_state != CALIB_STATE_DONE)
    {
        return;  // 未校准完成，不执行解算
    }
    
    // 静态初始化卡尔曼滤波器（仅第一次执行）
    static uint8_t kalman_init_flag = 0;
    if(kalman_init_flag == 0 && USE_KALMAN_FILTER)
    {
        // 卡尔曼参数初始化（平衡车推荐值）
        Kalman_Init(&kf_roll, 0.003f, 0.003f, 0.03f);
        Kalman_Init(&kf_pitch, 0.003f, 0.003f, 0.03f);
        kalman_init_flag = 1;
    }
    
    // 应用校准与中间变量赋值（应用坐标系配置）
    float ax = imu963ra_acc_x * IMU_AXIS_X_SIGN;
    float ay = imu963ra_acc_y * IMU_AXIS_Y_SIGN;
    float az = imu963ra_acc_z * IMU_AXIS_Z_SIGN;

    float gx = (imu963ra_gyro_x - gyro_off_x) * IMU_AXIS_X_SIGN;
    float gy = (imu963ra_gyro_y - gyro_off_y) * IMU_AXIS_Y_SIGN;
    float gz = (imu963ra_gyro_z - gyro_off_z) * IMU_AXIS_Z_SIGN;

    // 输入死区
    if(-4 < gx && gx < 4) { gx = 0; }
    if(-4 < gy && gy < 4) { gy = 0; }
    if(-4 < gz && gz < 4) { gz = 0; }
    
    // 使用从IIO子系统读取的真实scale值，陀螺仪scale已经是rad/s
    float gyro_roll_rate  = (float)gx * imu_gyro_scale * rad_to_deg_per_s;
    float gyro_pitch_rate = (float)gy * imu_gyro_scale * rad_to_deg_per_s;
    
    // 横滚角加速度计计算
    RollAcc   = atan2f((float)ay, (float)az) * mpu6050_const_data1;
    // 俯仰角加速度计计算
    PitchAcc  = -atan2f((float)ax, (float)az) * mpu6050_const_data1;
    
    float dt = Get_Real_dt();
    
#if USE_KALMAN_FILTER
    // 卡尔曼滤波模式（Roll/Pitch轴）
    // 动态调整R_measure（根据加速度计状态）
    kf_roll.R_measure  = Get_Dynamic_Rmeasure(ax, ay, az);
    kf_pitch.R_measure = kf_roll.R_measure;
    
    // 卡尔曼滤波计算
    Roll  = Kalman_Calculate(&kf_roll, RollAcc, gyro_roll_rate, dt);
    Pitch = Kalman_Calculate(&kf_pitch, PitchAcc, gyro_pitch_rate, dt);
#else
    // 互补滤波模式（使用真实的scale值）
    float RollGyro  = Roll + (float)gx * imu_gyro_scale * rad_to_deg_per_s * dt;
    Roll      = 0.005 * RollAcc + (1 - 0.005) * RollGyro;
    
    float PitchGyro = Pitch + (float)gy * imu_gyro_scale * rad_to_deg_per_s * dt;
    Pitch     = 0.005 * PitchAcc + (1 - 0.005) * PitchGyro;
#endif
    
    // 偏航角计算：仅陀螺仪积分（无加速度计校准，会漂移）
    Yaw       += (float)gz * imu_gyro_scale * rad_to_deg_per_s * dt * 1.014f;
    
    // 一阶低通滤波
    Roll_Temp  = MPU6050_LOW_PASS_FILTER * Roll + (1 - MPU6050_LOW_PASS_FILTER) * Roll_Temp;
    Yaw_Temp   = MPU6050_LOW_PASS_FILTER * Yaw + (1 - MPU6050_LOW_PASS_FILTER) * Yaw_Temp;
    Pitch_Temp = MPU6050_LOW_PASS_FILTER * Pitch + (1 - MPU6050_LOW_PASS_FILTER) * Pitch_Temp;
    
    // 输出死区
    if (fabs(Roll_Result - Roll_Temp) > MPU6050_OUTPUT_DEAD_ZONE) { Roll_Result = Roll_Temp; }
    if (fabs(Yaw_Result - Yaw_Temp) > MPU6050_OUTPUT_DEAD_ZONE) { Yaw_Result = Yaw_Temp; }
    if (fabs(Pitch_Result - Pitch_Temp) > MPU6050_OUTPUT_DEAD_ZONE) { Pitch_Result = Pitch_Temp; }
}
/*******************************************************************************************************************/
/*----------------------------------------------------------------------------------------------[E] 姿态解算+滤波 [E]*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 工具函数 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

void IMU963RA_Get_Calibrated_Data(float *acc_x, float *acc_y, float *acc_z, 
                                        float *gyro_x, float *gyro_y, float *gyro_z)
{
    if (calib_state != CALIB_STATE_DONE)
    {
        return;  // 未校准完成，不执行校准
    }

    // 加速度无零飘
    *acc_x = (float)imu963ra_acc_x;
    *acc_y = (float)imu963ra_acc_y;
    *acc_z = (float)imu963ra_acc_z;
    
    // 陀螺仪赋值
    *gyro_x = (float)imu963ra_gyro_x - gyro_off_x;
    *gyro_y = (float)imu963ra_gyro_y - gyro_off_y;
    *gyro_z = (float)imu963ra_gyro_z - gyro_off_z;
} 

// 清零Yaw角
void IMU963RA_Reset_Yaw(void)
{
    Yaw = 0.0f;
    Yaw_Temp = 0.0f;
    Yaw_Result = 0.0f;
}
/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 工具函数 [E]*/
/*******************************************************************************************************************/


#endif
