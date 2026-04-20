/********************************************************************************************************************
* 文件名称     IMU_Analysis.cpp
* 功能描述     IMU姿态解算和校准模块
* 使用说明     1. 支持三轴/六轴/九轴模式，通过 IMU_ANALYSIS_MODE 宏选择
*              2. 陀螺仪校准：采集静止时的数据，计算零偏
*              3. 磁力计校准：支持Min-Max法和椭球拟合法，旋转设备采集数据
*              4. 磁力计校准方法通过 MAG_CALIB_METHOD 宏选择（1:Min-Max, 2:椭球拟合）
********************************************************************************************************************/
#ifndef __IMU_ANALYSIS_H__
#define __IMU_ANALYSIS_H__


// YAW_ONLY_ANALYSIS_MODE和IMU_ANALYSIS_MODE是互斥的，不能同时开启
// 仅Yaw输出的解算方法设置优先级高，会覆盖IMU_ANALYSIS_MODE的设置

// 仅Yaw输出的解算方法设置
// 0 关闭
// 1 Mag_Get_Yaw（仅磁力计+倾斜补偿）
// 2 Mahony AHRS（九轴，仅输出Yaw）
// 3 Madgwick AHRS（九轴，仅输出Yaw）
// 4 TiltMagYaw（重力投影磁修正陀螺积分）
#define YAW_ONLY_ANALYSIS_MODE        0

// IMU_解算模式设置（全欧拉角解算）
// 0 关闭
// 3 三轴
// 6 六轴
// 9 九轴
#define IMU_ANALYSIS_MODE             6

// YAW_ONLY_ANALYSIS_MODE和IMU_ANALYSIS_MODE是互斥的，不能同时开启
// 仅Yaw输出的解算方法设置优先级高，会覆盖IMU_ANALYSIS_MODE的设置


// 采样周期设置
#define DELTA_T_3AXIS                   0.001f  // 三轴采样周期
#define DELTA_T_6AXIS                   0.001f  // 六轴采样周期
#define DELTA_T_9AXIS                   0.001f  // 九轴采样周期

// 陀螺仪死区阈值
#define GYRO_DEADZONE                   5.0f

// PI值
#define PI                              3.1415926535f


// 全局变量声明
extern volatile float Yaw_Result;    // 偏航角（Yaw）
extern volatile float Roll_Result;   // 横滚角（Roll）
extern volatile float Pitch_Result;  // 俯仰角（Pitch）
// IMU 通信+解析 使能标志位
extern volatile uint8_t IMU_D_and_A_Enable;

// IMU快速收敛相关
extern volatile uint8_t imu_quick_count;
extern volatile uint8_t imu_stable;


// IMU获取数据
void IMU_Update_Data(void);

// 应用加速度计数据
void    IMU_Acc_Apply       (float *ax, float *ay, float *az);

/*======================================================*/
/*[陀螺仪校准]********************************************/
/*======================================================*/

// 用于存储陀螺仪校准的状态、参数和中间数据
typedef struct {
    // 校准状态
    uint8_t calib_state;  // 校准状态（0: 未校准, 1: 校准中, 2: 已校准）
    // 校准计数
    uint32_t calib_count;
    // 零点偏移量
    float offset_x;  // X轴偏移
    float offset_y;  // Y轴偏移
    float offset_z;  // Z轴偏移  

    // 零飘数据采样累加
    float sum_x;  // X轴数据累加和
    float sum_y;  // Y轴数据累加和
    float sum_z;  // Z轴数据累加和
} Gyro_Calib_StructDef;

// 枚举定义校准状态
typedef enum {
    GYRO_CALIB_STATE_IDLE    = 0,          // 未校准  
    GYRO_CALIB_STATE_RUNNING = 1,          // 校准中
    GYRO_CALIB_STATE_DONE    = 2           // 校准完
} Gyro_CalibState_t;

// 零飘校准校准需要的样本数
#define GYRO_CALIB_TARGET_SAMPLES    800  

extern Gyro_Calib_StructDef gyro_cal;

void    IMU_Gyro_Calib_Start       (Gyro_Calib_StructDef *cal);
int     IMU_Gyro_Calib_Check       (Gyro_Calib_StructDef *cal);
void    IMU_Gyro_Apply       (Gyro_Calib_StructDef *cal, float *gx, float *gy, float *gz);
/*======================================================*/
/********************************************[陀螺仪校准]*/
/*======================================================*/


/*======================================================*/
/*[磁力计校准]********************************************/
/*======================================================*/
#define MAG_CALIB_METHOD    1
// 1: Min-Max法
// 2: 椭球拟合法

// 用于存储磁力计校准的状态、参数和中间数据
typedef struct {
    // 校准状态
    uint8_t calib_state;  // 校准状态（0: 未校准, 1: 校准中, 2: 已校准）
    // 校准计数
    uint32_t calib_count;  
    // 硬铁偏移（中心点）
    float offset_x;  // X轴偏移
    float offset_y;  // Y轴偏移
    float offset_z;  // Z轴偏移
    // 软铁缩放因子
    float scale_x;   // X轴缩放
    float scale_y;   // Y轴缩放
    float scale_z;   // Z轴缩放 
    // 采集的最大最小值
    int16_t max_x, min_x;
    int16_t max_y, min_y;
    int16_t max_z, min_z;
} Mag_Calib_StructDef;

// 枚举定义磁力计校准状态
typedef enum {
    MAG_CALIB_STATE_IDLE    = 0,          // 未校准  
    MAG_CALIB_STATE_RUNNING = 1,          // 校准中
    MAG_CALIB_STATE_DONE    = 2           // 校准完
} Mag_CalibState_t;

#define MAG_CALIB_MINMAX_TARGET_SAMPLES    2000  // 10ms * 2000 = 20秒

extern Mag_Calib_StructDef mag_cal;

void    IMU_Mag_Calib_Start       (Mag_Calib_StructDef *cal);
int     IMU_Mag_Calib_Check       (Mag_Calib_StructDef *cal);
void    IMU_Mag_Apply       (Mag_Calib_StructDef *cal, int16_t *mx, int16_t *my, int16_t *mz);
/*======================================================*/
/********************************************[磁力计校准]*/
/*======================================================*/


// // IMU姿态解算
// void    IMU_Update_Analysis         (void);

// 四元数结构体
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_StructDef;

// Mahony AHRS参数结构体
typedef struct {
    Quaternion_StructDef q;         // 四元数
    float Kp;                        // 比例增益
    float Ki;                        // 积分增益
    float quick_Kp;                  // 快速收敛比例增益
    float quick_Ki;                  // 快速收敛积分增益
    float exInt;                     // 误差积分项X
    float eyInt;                     // 误差积分项Y
    float ezInt;                     // 误差积分项Z
    uint8_t quick_mode;              // 快速收敛模式标志
} Mahony_AHRS_StructDef;

// 三轴姿态解算结构体
typedef struct {
    Quaternion_StructDef q;         // 四元数
} ThreeAxis_StructDef;

// 九轴姿态解算结构体
typedef struct {
    Mahony_AHRS_StructDef mahony;   // Mahony算法参数
    float mag_declination;           // 磁偏角
} NineAxis_StructDef;

// Mag_Get_Yaw算法结构体
typedef struct {
    float mag_declination;           // 磁偏角
    float yaw_filter_alpha;          // Yaw滤波系数
    float yaw_filtered;              // 滤波后的Yaw
} Mag_Get_Yaw_StructDef;

// Madgwick AHRS参数结构体
typedef struct {
    Quaternion_StructDef q;          // 四元数
    float beta;                       // 梯度下降步长
    float quick_beta;                 // 快速收敛梯度下降步长
    float invSampleFreq;              // 采样周期倒数
    float mag_declination;            // 磁偏角
} Madgwick_AHRS_StructDef;

// TiltMagYaw算法结构体
typedef struct {
    float yaw;                        // Yaw角
    float yaw_filtered;               // 滤波后的Yaw
    float yaw_error_int;              // Yaw误差积分
    float kp;                         // 比例增益
    float ki;                         // 积分增益
    float quick_kp;                   // 快速收敛比例增益
    float quick_ki;                   // 快速收敛积分增益
    float yaw_filter_alpha;           // Yaw滤波系数
    float mag_declination;            // 磁偏角
} TiltMagYaw_StructDef;

// Yaw-only算法联合体
typedef struct {
    Mag_Get_Yaw_StructDef mag_get_yaw;
    Mahony_AHRS_StructDef mahony;
    Madgwick_AHRS_StructDef madgwick;
    TiltMagYaw_StructDef tilt_mag_yaw;
} YawOnly_UnionDef;

// 全局变量声明
#if IMU_ANALYSIS_MODE == 3
extern ThreeAxis_StructDef three_axis;
#endif
#if IMU_ANALYSIS_MODE == 6
extern Mahony_AHRS_StructDef six_axis;
#endif
#if IMU_ANALYSIS_MODE == 9
extern NineAxis_StructDef nine_axis;
#endif
#if YAW_ONLY_ANALYSIS_MODE > 0
extern YawOnly_UnionDef yaw_only;
#endif

// IMU姿态解算
void    IMU_Update_Analysis         (void);

// 工具性函数

// 重置IMU姿态角数据
void    IMU_Reset_Data              (void);

#endif