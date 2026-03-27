#include "zf_device_imu963ra.h"


/*******************************************************************************************************************/
/*[S] 零飘校准 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 校准需要的样本数
#define CALIB_TARGET_SAMPLES    400  

// 枚举定义动态校准状态
typedef enum {
    CALIB_STATE_SPARE   = 0,     				// c
    CALIB_STATE_RUNNING = 1,   					// 校准中
    CALIB_STATE_DONE    = 2       				// 已校准
} CalibState_t;

static CalibState_t calib_state = CALIB_STATE_SPARE;
static uint16_t calib_count = 0;
static int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
static int32_t sum_mx = 0, sum_my = 0, sum_mz = 0;
static float gyro_off_x = 0, gyro_off_y = 0, gyro_off_z = 0;
static float mag_off_x = 0, mag_off_y = 0, mag_off_z = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU963RA 获取原始数据
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

    // 如果校准是完成状态，应用校准值获取校准后的数据
    if (calib_state == CALIB_STATE_DONE)
    {
        imu963ra_gyro_x = (float)imu963ra_gyro_x - gyro_off_x;
        imu963ra_gyro_y = (float)imu963ra_gyro_y - gyro_off_y;
        imu963ra_gyro_z = (float)imu963ra_gyro_z - gyro_off_z;
        imu963ra_mag_x = (float)imu963ra_mag_x - mag_off_x;
        imu963ra_mag_y = (float)imu963ra_mag_y - mag_off_y;
        imu963ra_mag_z = (float)imu963ra_mag_z - mag_off_z;
    }
}

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
    
    // 收集数据
    sum_gx += imu963ra_gyro_x;
    sum_gy += imu963ra_gyro_y;
    sum_gz += imu963ra_gyro_z;
    sum_mx += imu963ra_mag_x;
    sum_my += imu963ra_mag_y;
    sum_mz += imu963ra_mag_z;
    calib_count++;
    
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
    
    return 1;
}

/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 零飘校准 [E]*/
/*******************************************************************************************************************/
