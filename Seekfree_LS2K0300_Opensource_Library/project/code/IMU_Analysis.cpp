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
// IMU963RA 数据采集和分析使能标志位
volatile uint8_t IMU963RA_analysis_enable = 0;

// 坐标系配置：根据您的IMU安装方向调整
// 1 = 正常方向，-1 = 反方向
// 加速度计的坐标系
#define IMU_ACC_X_SIGN  1
#define IMU_ACC_Y_SIGN  1
#define IMU_ACC_Z_SIGN  -1  // Z轴取反，因为重力方向是-Z

// 陀螺仪的坐标系（可能需要单独调整）
#define IMU_GYRO_X_SIGN  1
#define IMU_GYRO_Y_SIGN  1
#define IMU_GYRO_Z_SIGN  -1

// 磁力计的坐标系（可能需要单独调整）
#define IMU_MAG_X_SIGN   1
#define IMU_MAG_Y_SIGN   1
#define IMU_MAG_Z_SIGN   1

// 磁力计轴旋转配置
// 0: 不旋转
// 1: 顺时针旋转 90 度
// 2: 顺时针旋转 180 度
// 3: 顺时针旋转 270 度
#define IMU_MAG_AXIS_ROTATE  3


// 硬编码的IMU963RA参数
// 陀螺仪：±2000°/s, 16-bit
static float imu_gyro_scale = (2000.0f / 32768.0f) * (M_PI / 180.0f);     // 陀螺仪scale (rad/s per LSB)


// 零飘校准校准需要的样本数
#define CALIB_TARGET_SAMPLES    800  
// 枚举定义校准状态
typedef enum {
    CALIB_STATE_SPARE   = 0,          // 未开始
    CALIB_STATE_RUNNING = 1,          // 校准中
    CALIB_STATE_DONE    = 2           // 已校准
} CalibState_t;
static CalibState_t calib_state = CALIB_STATE_SPARE; // 当前校准状态
static uint16_t calib_count = 0;// 已收集样本数量






// 选择九轴数据的解算
#if (IMU_ANALYSIS_USE_MAG == 1)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 获取九轴原始数据
// 使用示例     imu963ra_get_data();                                              // 执行该函数后，直接查看对应的变量即可
// 备注信息     定时器定时中断定时调用该函数，获取 IMU963RA 原始数据
//-------------------------------------------------------------------------------------------------------------------
// IMU963RA 原始数据变量
// 加速度计原始值
// imu963ra_acc_x           imu963ra_acc_y          imu963ra_acc_z
// 陀螺仪原始值
// imu963ra_gyro_x          imu963ra_gyro_y         imu963ra_gyro_z
// 磁力计原始值
// imu963ra_mag_x           imu963ra_mag_y          imu963ra_mag_z  
void imu963ra_update_data(void)
{
    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963ra_get_mag();
}

// Madgwick算法相关定义
#define DEG2RAD (M_PI / 180.0f) // 角度转弧度
#define RAD2DEG (180.0f / M_PI) // 弧度转角度

// 三轴数据结构体
typedef struct {
    float x; // X轴数据
    float y; // Y轴数据
    float z; // Z轴数据
} Axis3f;

// Madgwick算法四元数（表示姿态）
// q = q1 + q2*i + q3*j + q4*k
// 初始值：q1=1, q2=0, q3=0, q4=0 表示水平朝北姿态
static float madgwick_q1 = 1.0f; // 四元数实部
static float madgwick_q2 = 0.0f; // 四元数虚部
static float madgwick_q3 = 0.0f; // 四元数虚部
static float madgwick_q4 = 0.0f; // 四元数k分量

// Madgwick算法增益参数（控制响应速度和稳定性）
// 推荐值：0.08-0.12（四轮车定角度转向应用）
// 调整建议：
//   - 增大（如0.15-0.3）：响应更快，适合快速运动场景
//   - 减小（如0.05-0.08）：更稳定，适合平稳场景
static float madgwick_beta = 0.15f;

// 时间戳记录（用于计算dt）
static std::chrono::steady_clock::time_point last_time_madgwick = std::chrono::steady_clock::now();

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取时间间隔（dt）
// 返回参数     dt：时间间隔（秒），范围[0.001s, 0.05s]
// 备注信息     用于Madgwick算法的时间积分，限制范围避免异常值
//-------------------------------------------------------------------------------------------------------------------
static float Get_dt(void)
{
    auto current_time = std::chrono::steady_clock::now();
    auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time_madgwick).count();
    float dt = (float)time_diff / 1000.0f;
    if (dt > 0.05f) dt = 0.05f; // 限制最大dt
    if (dt < 0.001f) dt = 0.005f; // 限制最小dt
    last_time_madgwick = current_time;
    return dt;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     快速平方根倒数计算（牛顿迭代法）
// 输入参数     x：输入值
// 返回参数     1/√x：平方根倒数
// 备注信息     比标准sqrtf()函数更快，用于向量归一化
//-------------------------------------------------------------------------------------------------------------------
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    union {
        float f;
        int32_t i;
    } conv = {x};
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f *= 1.5f - halfx * conv.f * conv.f;
    return conv.f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Madgwick AHRS算法核心函数 
// 算法原理：
// 1. 使用梯度下降法融合加速度计和磁力计数据
// 2. 结合陀螺仪数据更新四元数
// 3. 通过四元数计算欧拉角（Roll、Pitch、Yaw）
//
// 输入参数：
//   acc  - 加速度计数据（m/s²，已归一化）
//   gyro - 陀螺仪数据（rad/s）
//   mag  - 磁力计数据（已归一化）
//   dt   - 时间间隔（秒）
//
// 输出结果：
//   更新全局四元数 madgwick_q1~madgwick_q4
//-------------------------------------------------------------------------------------------------------------------
static void MadgwickQuaternionUpdate(Axis3f acc, Axis3f gyro, Axis3f mag, float dt)
{
    float recipNorm;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q1mx, _2q1my, _2q1mz, _2q2mx, _2bx, _2bz, _4bx, _4bz, _2q1, _2q2, _2q3, _2q4, _2q1q3, _2q3q4, q1q1, q1q2, q1q3, q1q4, q2q2, q2q3, q2q4, q3q3, q3q4, q4q4;

    float ax = acc.x;
    float ay = acc.y;
    float az = acc.z;
    float gx = gyro.x;
    float gy = gyro.y;
    float gz = gyro.z;
    float mx = mag.x;
    float my = mag.y;
    float mz = mag.z;

    // 检查加速度计数据是否有效（避免除零）
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 检查磁力计数据是否有效
        if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            // 计算中间变量（优化计算效率）
            _2q1mx = 2.0f * madgwick_q1 * mx;
            _2q1my = 2.0f * madgwick_q1 * my;
            _2q1mz = 2.0f * madgwick_q1 * mz;
            _2q2mx = 2.0f * madgwick_q2 * mx;
            _2q1 = 2.0f * madgwick_q1;
            _2q2 = 2.0f * madgwick_q2;
            _2q3 = 2.0f * madgwick_q3;
            _2q4 = 2.0f * madgwick_q4;
            _2q1q3 = 2.0f * madgwick_q1 * madgwick_q3;
            _2q3q4 = 2.0f * madgwick_q3 * madgwick_q4;
            q1q1 = madgwick_q1 * madgwick_q1;
            q1q2 = madgwick_q1 * madgwick_q2;
            q1q3 = madgwick_q1 * madgwick_q3;
            q1q4 = madgwick_q1 * madgwick_q4;
            q2q2 = madgwick_q2 * madgwick_q2;
            q2q3 = madgwick_q2 * madgwick_q3;
            q2q4 = madgwick_q2 * madgwick_q4;
            q3q3 = madgwick_q3 * madgwick_q3;
            q3q4 = madgwick_q3 * madgwick_q4;
            q4q4 = madgwick_q4 * madgwick_q4;

            // 计算参考磁场方向（在机体坐标系中）
            hx = mx * q1q1 - _2q1my * madgwick_q4 + _2q1mz * madgwick_q3 + mx * q2q2 + _2q2 * my * madgwick_q3 + _2q2 * mz * madgwick_q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * madgwick_q4 + my * q1q1 - _2q1mz * madgwick_q2 + _2q2mx * madgwick_q3 - my * q2q2 + my * q3q3 + _2q3 * mz * madgwick_q4 - my * q4q4;
            _2bx = sqrtf(hx * hx + hy * hy);
            _2bz = -_2q1mx * madgwick_q3 + _2q1my * madgwick_q2 + mz * q1q1 + _2q2mx * madgwick_q4 - mz * q2q2 + _2q3 * my * madgwick_q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // 梯度下降法计算四元数导数（九轴模式）
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * madgwick_q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * madgwick_q4 + _2bz * madgwick_q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * madgwick_q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * madgwick_q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * madgwick_q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * madgwick_q3 + _2bz * madgwick_q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * madgwick_q4 - _4bz * madgwick_q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * madgwick_q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * madgwick_q3 - _2bz * madgwick_q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * madgwick_q2 + _2bz * madgwick_q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * madgwick_q1 - _4bz * madgwick_q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * madgwick_q4 + _2bz * madgwick_q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * madgwick_q1 + _2bz * madgwick_q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * madgwick_q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        } else {
            // 六轴模式（无磁力计数据）
            _2q1 = 2.0f * madgwick_q1;
            _2q2 = 2.0f * madgwick_q2;
            _2q3 = 2.0f * madgwick_q3;
            _2q4 = 2.0f * madgwick_q4;
            _2q1q3 = 2.0f * madgwick_q1 * madgwick_q3;
            _2q3q4 = 2.0f * madgwick_q3 * madgwick_q4;
            q1q1 = madgwick_q1 * madgwick_q1;
            q1q2 = madgwick_q1 * madgwick_q2;
            q1q3 = madgwick_q1 * madgwick_q3;
            q1q4 = madgwick_q1 * madgwick_q4;
            q2q2 = madgwick_q2 * madgwick_q2;
            q2q4 = madgwick_q2 * madgwick_q4;
            q3q3 = madgwick_q3 * madgwick_q3;
            q3q4 = madgwick_q3 * madgwick_q4;
            q4q4 = madgwick_q4 * madgwick_q4;

            // 梯度下降法计算四元数导数（六轴模式）
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * madgwick_q1 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * madgwick_q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * madgwick_q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay);
        }

        // 归一化梯度向量
        recipNorm = invSqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        s4 *= recipNorm;

        // 计算四元数导数（结合陀螺仪和梯度下降）
        qDot1 = 0.5f * (-madgwick_q2 * gx - madgwick_q3 * gy - madgwick_q4 * gz) - madgwick_beta * s1;
        qDot2 = 0.5f * (madgwick_q1 * gx + madgwick_q3 * gz - madgwick_q4 * gy) - madgwick_beta * s2;
        qDot3 = 0.5f * (madgwick_q1 * gy - madgwick_q2 * gz + madgwick_q4 * gx) - madgwick_beta * s3;
        qDot4 = 0.5f * (madgwick_q1 * gz + madgwick_q2 * gy - madgwick_q3 * gx) - madgwick_beta * s4;
    } else {
        // 加速度计数据无效，仅使用陀螺仪
        qDot1 = 0.5f * (-madgwick_q2 * gx - madgwick_q3 * gy - madgwick_q4 * gz);
        qDot2 = 0.5f * (madgwick_q1 * gx + madgwick_q3 * gz - madgwick_q4 * gy);
        qDot3 = 0.5f * (madgwick_q1 * gy - madgwick_q2 * gz + madgwick_q4 * gx);
        qDot4 = 0.5f * (madgwick_q1 * gz + madgwick_q2 * gy - madgwick_q3 * gx);
    }

    // 积分更新四元数
    madgwick_q1 += qDot1 * dt;
    madgwick_q2 += qDot2 * dt;
    madgwick_q3 += qDot3 * dt;
    madgwick_q4 += qDot4 * dt;

    // 归一化四元数（保持单位四元数）
    recipNorm = invSqrt(madgwick_q1 * madgwick_q1 + madgwick_q2 * madgwick_q2 + madgwick_q3 * madgwick_q3 + madgwick_q4 * madgwick_q4);
    madgwick_q1 *= recipNorm;
    madgwick_q2 *= recipNorm;
    madgwick_q3 *= recipNorm;
    madgwick_q4 *= recipNorm;
}

/*******************************************************************************************************************/
/*[S] 零飘校准 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 陀螺仪零飘校准相关变量
static int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;// 陀螺仪数据累加和
static float gyro_off_x = 0, gyro_off_y = 0, gyro_off_z = 0;// 陀螺仪零点偏移量

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 校准初始化
// 使用示例     IMU963RA_Calibration_Start();                                              // 开始校准
// 备注信息     校准状态机启用前的操作，用于初始化校准状态
//-------------------------------------------------------------------------------------------------------------------
void IMU963RA_Calibration_Start(void)
{
    calib_state = CALIB_STATE_RUNNING;  // 设置状态为校准中
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
        return 2; // 已校准完成
    }

    if(calib_state == CALIB_STATE_SPARE)
    {
        return 0; // 未开始校准
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
        
        // 检查是否收集够样本数
        if(calib_count >= CALIB_TARGET_SAMPLES)
        {
            gyro_off_x = (float)sum_gx / CALIB_TARGET_SAMPLES;
            gyro_off_y = (float)sum_gy / CALIB_TARGET_SAMPLES;
            gyro_off_z = (float)sum_gz / CALIB_TARGET_SAMPLES;
            
            calib_state = CALIB_STATE_DONE;
        }
    }
    
    return 1; // 校准中
}

/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 零飘校准 [E]*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 姿态解算 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

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
        return; // 未校准完成，不执行解算
    }
    
    float dt = Get_dt(); // 获取时间间隔

    // 准备传感器数据
    Axis3f acc, gyro, mag;

    // 加速度计数据（应用坐标系配置）
    acc.x = imu963ra_acc_x * IMU_ACC_X_SIGN;
    acc.y = imu963ra_acc_y * IMU_ACC_Y_SIGN;
    acc.z = imu963ra_acc_z * IMU_ACC_Z_SIGN;

    // 陀螺仪数据（应用零飘校准、坐标系配置、scale转换）
    gyro.x = (imu963ra_gyro_x - gyro_off_x) * IMU_GYRO_X_SIGN * imu_gyro_scale;
    gyro.y = (imu963ra_gyro_y - gyro_off_y) * IMU_GYRO_Y_SIGN * imu_gyro_scale;
    gyro.z = (imu963ra_gyro_z - gyro_off_z) * IMU_GYRO_Z_SIGN * imu_gyro_scale;

    // 磁力计数据（应用坐标系配置）
    float mx0 = imu963ra_mag_x * IMU_MAG_X_SIGN;
    float my0 = imu963ra_mag_y * IMU_MAG_Y_SIGN;
    float mz0 = imu963ra_mag_z * IMU_MAG_Z_SIGN;

    // 磁力计轴旋转（修正坐标系差异）
    #if IMU_MAG_AXIS_ROTATE == 0
    mag.x = mx0;
    mag.y = my0;
    mag.z = mz0;
    #elif IMU_MAG_AXIS_ROTATE == 1
    mag.x = my0;
    mag.y = -mx0;
    mag.z = mz0;
    #elif IMU_MAG_AXIS_ROTATE == 2
    mag.x = -mx0;
    mag.y = -my0;
    mag.z = mz0;
    #elif IMU_MAG_AXIS_ROTATE == 3
    mag.x = -my0;
    mag.y = mx0;
    mag.z = mz0;
    #endif

    // 调用Madgwick算法更新姿态
    MadgwickQuaternionUpdate(acc, gyro, mag, dt);

    // 四元数转欧拉角
    // Roll：绕X轴旋转，范围[-90°, 90°]
    Roll_Result = atan2f(2.0f * (madgwick_q1 * madgwick_q2 + madgwick_q3 * madgwick_q4), 1.0f - 2.0f * (madgwick_q2 * madgwick_q2 + madgwick_q3 * madgwick_q3)) * RAD2DEG;
    // Pitch：绕Y轴旋转，范围[-90°, 90°]
    Pitch_Result = asinf(2.0f * (madgwick_q1 * madgwick_q3 - madgwick_q4 * madgwick_q2)) * RAD2DEG;  
    // Yaw：绕Z轴旋转，范围[-180°, 180°]
    Yaw_Result = atan2f(2.0f * (madgwick_q1 * madgwick_q4 + madgwick_q2 * madgwick_q3), 1.0f - 2.0f * (madgwick_q3 * madgwick_q3 + madgwick_q4 * madgwick_q4)) * RAD2DEG;
}

/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 姿态解算 [E]*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 工具函数 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取校准后的IMU数据
// 输入参数     指向输出变量的指针（加速度计、陀螺仪、磁力计各三轴）
// 备注信息     用于调试或数据监控，返回经过校准的原始数据
//-------------------------------------------------------------------------------------------------------------------
void IMU963RA_Get_Calibrated_Data(float *acc_x, float *acc_y, float *acc_z, 
                                        float *gyro_x, float *gyro_y, float *gyro_z, 
                                        float *mag_x, float *mag_y, float *mag_z)
{
    if (calib_state != CALIB_STATE_DONE)
    {
        return;
    }

    *acc_x = (float)imu963ra_acc_x;
    *acc_y = (float)imu963ra_acc_y;
    *acc_z = (float)imu963ra_acc_z;
    
    *gyro_x = (float)imu963ra_gyro_x - gyro_off_x;
    *gyro_y = (float)imu963ra_gyro_y - gyro_off_y;
    *gyro_z = (float)imu963ra_gyro_z - gyro_off_z;
    
    *mag_x = (float)imu963ra_mag_x;
    *mag_y = (float)imu963ra_mag_y;
    *mag_z = (float)imu963ra_mag_z;
} 

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置Yaw角（归零）
// 备注信息     将当前方向设为0°，用于定角度转向的起点
//-------------------------------------------------------------------------------------------------------------------
void IMU963RA_Reset_Yaw(void)
{
     // 重置四元数为初始姿态（水平朝北）
    madgwick_q1 = 1.0f;
    madgwick_q2 = 0.0f;
    madgwick_q3 = 0.0f;
    madgwick_q4 = 0.0f;
    
    // 重置姿态角输出变量
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
// 加速度计原始值
// imu963ra_acc_x           imu963ra_acc_y          imu963ra_acc_z
// 陀螺仪原始值
// imu963ra_gyro_x          imu963ra_gyro_y         imu963ra_gyro_z
void imu963ra_update_data(void)
{
    imu963ra_get_acc();
    imu963ra_get_gyro();
}

/*******************************************************************************************************************/
/*[S] 零飘校准 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/
// 上次时间戳（用于计算dt）
static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取实际采样时间间隔
// 返回参数     dt：时间间隔（秒），范围[0.001s, 0.05s]
// 备注信息     用于姿态解算的时间积分，限制范围避免异常值
//-------------------------------------------------------------------------------------------------------------------
// 获取实际采样时间间隔
static float Get_Real_dt(void)
{
    auto current_time = std::chrono::steady_clock::now();
    auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();
    
    // 限制dt范围
    float dt = (float)time_diff / 1000.0f;
    if (dt > 0.05f) dt = 0.05f; // 限制最大dt
    if (dt < 0.001f) dt = 0.005f; // 限制最小dt
    
    last_time = current_time;
    return dt;
}

// 陀螺仪零飘校准相关变量
static int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0; // 陀螺仪数据累加和
static float gyro_off_x = 0, gyro_off_y = 0, gyro_off_z = 0; // 陀螺仪零点偏移量

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     开始校准
// 备注信息     初始化校准状态，准备收集陀螺仪零飘数据
//-------------------------------------------------------------------------------------------------------------------
void IMU963RA_Calibration_Start(void)
{
    calib_state = CALIB_STATE_RUNNING; // 设置状态为校准中
    calib_count = 0;
    sum_gx = 0;
    sum_gy = 0;
    sum_gz = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     校准状态检查
// 返回参数     校准状态：0-未校准，1-校准中，2-已校准
// 备注信息     需要配合IMU963RA_analysis_enable标志位使用
//-------------------------------------------------------------------------------------------------------------------
int8_t IMU963RA_Calibration_Check(void)
{
    if(calib_state == CALIB_STATE_DONE)
    {
        return 2; // 已校准完成
    }

    if(calib_state == CALIB_STATE_SPARE)
    {
        return 0; // 未开始校准
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
    
    return 1; // 校准中
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
// Yaw 轴陀螺仪增益系数（用于修正 Yaw 变化偏小的问题）
#define YAW_GYRO_GAIN  1.38f

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
    float ax = imu963ra_acc_x * IMU_ACC_X_SIGN;
    float ay = imu963ra_acc_y * IMU_ACC_Y_SIGN;
    float az = imu963ra_acc_z * IMU_ACC_Z_SIGN;

    float gx = (imu963ra_gyro_x - gyro_off_x) * IMU_GYRO_X_SIGN;
    float gy = (imu963ra_gyro_y - gyro_off_y) * IMU_GYRO_Y_SIGN;
    float gz = (imu963ra_gyro_z - gyro_off_z) * IMU_GYRO_Z_SIGN;

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
    // 使用固定的R_measure值
    kf_roll.R_measure  = 0.03f;
    kf_pitch.R_measure = 0.03f;
    
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
    Yaw       += (float)gz * imu_gyro_scale * rad_to_deg_per_s * dt * YAW_GYRO_GAIN;
    
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
