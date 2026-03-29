/********************************************************************************************************************
* IMU姿态解算
********************************************************************************************************************/
#include "zf_device_imu963ra.h"
#include "MahonyAHRS.h"
#include <math.h>
#include <chrono>


// 全局变量
volatile float Yaw_Result = 0.0f;    // 偏航角（Yaw）
volatile float Roll_Result = 0.0f;   // 横滚角（Roll）
volatile float Pitch_Result = 0.0f;  // 俯仰角（Pitch）
// IMU963RA 分析使能标志位
volatile uint8_t IMU963RA_analysis_enable = 0;


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
#if (USE_MAG == 1)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 获取九轴原始数据
// 使用示例     imu963ra_get_data();                                              // 执行该函数后，直接查看对应的变量即可
// 备注信息     定时器定时中断定时调用该函数，获取 IMU963RA 原始数据
//-------------------------------------------------------------------------------------------------------------------
// IMU963RA 原始数据变量
// imu963ra_acc_x           imu963ra_acc_y          imu963ra_acc_z
// imu963ra_gyro_x          imu963ra_gyro_y         imu963ra_gyro_z
// imu963ra_mag_x           imu963ra_mag_y          imu963ra_mag_z  
void imu963ra_get_data(void)
{
    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963ra_get_mag();
}

// 条件编译选项
#define ENABLE_FULL_EULER    1   // 1: 计算全部欧拉角, 0: 只计算Yaw角

// 传感器数据缩放因子
#define GYRO_SCALE    (2000.0f / 32768.0f * M_PI / 180.0f)  // 陀螺仪刻度因子（转换为rad/s）
#define ACC_SCALE     (8.0f / 32768.0f * 9.81f)            // 加速度计刻度因子（转换为m/s²）
#define MAG_SCALE     (4912.0f / 32768.0f)                 // 磁力计刻度因子（转换为uT）


/*******************************************************************************************************************/
/*[S] 零飘校准 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

static int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
static int32_t sum_mx = 0, sum_my = 0, sum_mz = 0;
static float gyro_off_x = 0, gyro_off_y = 0, gyro_off_z = 0;
static float mag_off_x = 0, mag_off_y = 0, mag_off_z = 0;

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
    sum_mx = 0;
    sum_my = 0;
    sum_mz = 0;
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
        imu963ra_get_data();

        // 收集数据
        sum_gx += imu963ra_gyro_x;
        sum_gy += imu963ra_gyro_y;
        sum_gz += imu963ra_gyro_z;
        sum_mx += imu963ra_mag_x;
        sum_my += imu963ra_mag_y;
        sum_mz += imu963ra_mag_z;
        calib_count++;

        IMU963RA_analysis_enable = 0;
        
        if(calib_count >= CALIB_TARGET_SAMPLES)
        {
            gyro_off_x = (float)sum_gx / CALIB_TARGET_SAMPLES;
            gyro_off_y = (float)sum_gy / CALIB_TARGET_SAMPLES;
            gyro_off_z = (float)sum_gz / CALIB_TARGET_SAMPLES;
            mag_off_x  = (float)sum_mx / CALIB_TARGET_SAMPLES;
            mag_off_y  = (float)sum_my / CALIB_TARGET_SAMPLES;
            mag_off_z  = (float)sum_mz / CALIB_TARGET_SAMPLES;
            
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
    
    // 应用零飘校准
    float gx = (imu963ra_gyro_x - gyro_off_x) * GYRO_SCALE;
    float gy = (imu963ra_gyro_y - gyro_off_y) * GYRO_SCALE;
    float gz = (imu963ra_gyro_z - gyro_off_z) * GYRO_SCALE;
    
    float ax = imu963ra_acc_x * ACC_SCALE;
    float ay = imu963ra_acc_y * ACC_SCALE;
    float az = imu963ra_acc_z * ACC_SCALE;
    
    float mx = (imu963ra_mag_x - mag_off_x) * MAG_SCALE;
    float my = (imu963ra_mag_y - mag_off_y) * MAG_SCALE;
    float mz = (imu963ra_mag_z - mag_off_z) * MAG_SCALE;
    
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
    
    // 磁力计赋值
    *mag_x = (float)imu963ra_mag_x - mag_off_x;
    *mag_y = (float)imu963ra_mag_y - mag_off_y;
    *mag_z = (float)imu963ra_mag_z - mag_off_z;
}   
/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 工具函数 [E]*/
/*******************************************************************************************************************/

#endif






// 六轴数据的解算
#if (USE_MAG == 0)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 获取六轴原始数据
// 使用示例     imu963ra_get_data();                                              // 执行该函数后，直接查看对应的变量即可
// 备注信息     定时器定时中断定时调用该函数，获取 IMU963RA 原始数据
//-------------------------------------------------------------------------------------------------------------------
// IMU963RA 原始数据变量
// imu963ra_acc_x           imu963ra_acc_y          imu963ra_acc_z
// imu963ra_gyro_x          imu963ra_gyro_y         imu963ra_gyro_z
void imu963ra_get_data(void)
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
        imu963ra_get_data();
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
    // 计算加速度模长（理想值为1g，约16384 LSB/g）
    float acc_mag = sqrtf((float)ax*ax + (float)ay*ay + (float)az*az);
    // 归一化到1g参考系
    acc_mag = acc_mag / 16384.0f;
    
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
// 陀螺仪积分系数
const float mpu6050_const_data2 = (1.0f / 32768.0f) * 2000.0f;

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
    
    // 尝试使用校准的系数修正原始数据
    if(calib_state == CALIB_STATE_DONE)
    {
        imu963ra_gyro_x -= gyro_off_x;
        imu963ra_gyro_y -= gyro_off_y;
        imu963ra_gyro_z -= gyro_off_z;
    }
    else 
	// 使用固定值修正原始数据
	{
		mpu6050_gyro_x += 0.0f;
		mpu6050_gyro_y += 0.0f;
		mpu6050_gyro_z -= 0.0f;
	}
    
    // 输入死区
    if(-4 < imu963ra_gyro_x && imu963ra_gyro_x < 4) { imu963ra_gyro_x = 0; }
    if(-4 < imu963ra_gyro_y && imu963ra_gyro_y < 4) { imu963ra_gyro_y = 0; }
    if(-4 < imu963ra_gyro_z && imu963ra_gyro_z < 4) { imu963ra_gyro_z = 0; }
    
    // 计算陀螺仪角速度（转换为 °/s）
    float gyro_roll_rate  = (float)imu963ra_gyro_x * mpu6050_const_data2;
    float gyro_pitch_rate = (float)imu963ra_gyro_y * mpu6050_const_data2;
    
    // 横滚角加速度计计算
    RollAcc   = atan2f((float)imu963ra_acc_y, (float)imu963ra_acc_z) * mpu6050_const_data1;
    // 俯仰角加速度计计算
    PitchAcc  = -atan2f((float)imu963ra_acc_x, (float)imu963ra_acc_z) * mpu6050_const_data1;
    
    float dt = Get_Real_dt();
    
#if USE_KALMAN_FILTER
    // 卡尔曼滤波模式（Roll/Pitch轴）
    // 动态调整R_measure（根据加速度计状态）
    kf_roll.R_measure  = Get_Dynamic_Rmeasure(imu963ra_acc_x, imu963ra_acc_y, imu963ra_acc_z);
    kf_pitch.R_measure = kf_roll.R_measure;
    
    // 卡尔曼滤波计算
    Roll  = Kalman_Calculate(&kf_roll, RollAcc, gyro_roll_rate, dt);
    Pitch = Kalman_Calculate(&kf_pitch, PitchAcc, gyro_pitch_rate, dt);
#else
    // 互补滤波模式
    RollGyro  = Roll + (float)imu963ra_gyro_x * GYRO_SCALE_6AXIS * dt;
    Roll      = 0.005 * RollAcc + (1 - 0.005) * RollGyro;
    
    PitchGyro = Pitch + (float)imu963ra_gyro_y * GYRO_SCALE_6AXIS * dt;
    Pitch     = 0.005 * PitchAcc + (1 - 0.005) * PitchGyro;
#endif
    
    // 偏航角计算：仅陀螺仪积分（无加速度计校准，会漂移）
    Yaw       += (float)imu963ra_gyro_z * GYRO_SCALE_6AXIS * dt * 1.014f;
    
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
