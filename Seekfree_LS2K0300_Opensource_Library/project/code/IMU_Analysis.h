/********************************************************************************************************************
* IMU姿态解算
********************************************************************************************************************/
#ifndef __IMU_ANALYSIS_H__
#define __IMU_ANALYSIS_H__

// 是否启用磁力计
#define IMU_ANALYSIS_USE_MAG 0

// 全局变量声明
extern volatile float Yaw_Result;    // 偏航角（Yaw）
extern volatile float Roll_Result;   // 横滚角（Roll）
extern volatile float Pitch_Result;  // 俯仰角（Pitch）

// IMU963RA 通信+解析 使能标志位
extern volatile uint8_t IMU963RA_analysis_enable;

// 函数声明

void    imu963ra_update_data            (void);
void    IMU963RA_Calibration_Start      (void);
int8_t  IMU963RA_Calibration_Check      (void);
void    IMU963RA_Analysis_Update        (void);
# if (IMU_ANALYSIS_USE_MAG == 1)
    void    IMU963RA_Get_Calibrated_Data      (float *acc_x, float *acc_y, float *acc_z, 
                                            float *gyro_x, float *gyro_y, float *gyro_z, 
                                            float *mag_x, float *mag_y, float *mag_z);
# else
    void    IMU963RA_Get_Calibrated_Data      (float *acc_x, float *acc_y, float *acc_z, 
                                            float *gyro_x, float *gyro_y, float *gyro_z);    
#endif

void    IMU963RA_Reset_Yaw              (void);

#endif