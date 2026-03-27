/********************************************************************************************************************
* IMU姿态解算
********************************************************************************************************************/
#ifndef __IMU_ANALYSIS_H__
#define __IMU_ANALYSIS_H__

void    imu963ra_get_data               (void);
void    IMU963RA_Calibration_Start      (void);
uint8_t IMU963RA_Calibration_Check      (void);


#endif