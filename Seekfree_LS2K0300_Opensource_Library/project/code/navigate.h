#ifndef __navigate_h
#define __navigate_h

#include "zf_common_headfile.h"

// // 角度位置环PID结构体（串级PID的外环）
// typedef struct {
//     float target;           // 目标角度
//     float actual;           // 实际角度
//     float output;           // 输出（目标转向速度）
    
//     float Kp;               // 比例系数
//     float Ki;               // 积分系数
//     float Kd;               // 微分系数
    
//     float error;            // 当前误差
//     float last_error;       // 上次误差
//     float error_integral;   // 误差积分
//     float integral_max;     // 积分限幅
    
//     float OutMax;           // 最大输出（最大转向速度）
//     float OutMin;           // 最小输出（最小转向速度）
// } Angle_Position_PID;

// extern int16_t LeftPWM, RightPWM;         // 左右电机的PWM值
// extern uint8_t is_angle_turning;  // 是否正在执行转向动作的标志

//============================================================
// UWB跟随参数设置（参照商家 Followingcar1.2 原始值方案）
//   Motor_Set 输入范围: -10000 ~ 10000
//   商家公式: PWM_diff = |raw_azim| × Dis (Dis=50)
//============================================================
// uwb跟随函数参数定义
// UWB 跟随参数
#define UWB_FOLLOW_DIST_M       1.5f    // 跟随距离阈值（米），超过此距离停车
#define UWB_FOLLOW_BASE_SPEED   3500    // 基础速度（0~10000），值越大越快
#define UWB_FOLLOW_STEER_COEFF  600.0f  // 转向灵敏度系数，方位角(度) × 此值 = 差速量
#define UWB_FOLLOW_CALL_MS      50      // uwb_follow 调用间隔（毫秒），需与 main.cpp 中 number_task 计数一致
#define UWB_FOLLOW_TIMEOUT_MS   200     // UWB 数据超时（毫秒），超时停车

// 避障+跟随函数的参数定义
// #define FOLLOW_BASE_SPEED            3500    // 基础前进速度（Motor_Set PWM 值）
// #define FOLLOW_DIS_GAIN              500     // 差速系数（= 商家 MotorData.Dis），|raw_azi| × 此值 = PWM 差分量
// #define FOLLOW_MAX_DIST_M            5.0f    // 最大有效距离（m），超过此距离认为无目标
// #define FOLLOW_MIN_DIST_M            0.3f    // 最小有效距离（m），低于此距离认为过近
// #define FOLLOW_RAW_DEADZONE          5       // 原始 azimuth 死区（±5），在此范围内直行

// // 雷达避障参数
// #define RADAR_FORWARD_ANGLE          270.0f  // 雷达正前方对应的角度（度），因安装方向偏移
// #define FOLLOW_OBS_NEAR_MM           200     // 紧急避障距离（mm），原地转向
// #define FOLLOW_OBS_AVOID_MM          330     // 常规避障距离（mm），匹配 avoid() 的 330mm 阈值
// #define FOLLOW_OBS_ANGLE_RANGE       50.0f   // 前方避障角度范围（±度，220°~320°），以 RADAR_FORWARD_ANGLE 为中心

// // 超时保护
// #define FOLLOW_UWB_TIMEOUT_CNT       200     // UWB 超时计数值 (×10ms)，超时停机

// 雷达避障函数
void avoid(void);

// 转向控制函数
// void Start_Angle_Turn(float angle);
// void Stop_Angle_Turn(void);
// uint8_t Update_Angle_Turn(void);
// uint8_t Is_Angle_Turning(void);
// float Get_Angle_Turn_Error(void);

// uwb导航函数
void uwb_follow(void);

#endif