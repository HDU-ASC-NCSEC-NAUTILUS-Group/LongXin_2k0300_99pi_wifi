#ifndef __navigate_h
#define __navigate_h

#include "zf_common_headfile.h"

// 角度位置环PID结构体（串级PID的外环）
typedef struct {
    float target;           // 目标角度
    float actual;           // 实际角度
    float output;           // 输出（目标转向速度）
    
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float Kd;               // 微分系数
    
    float error;            // 当前误差
    float last_error;       // 上次误差
    float error_integral;   // 误差积分
    float integral_max;     // 积分限幅
    
    float OutMax;           // 最大输出（最大转向速度）
    float OutMin;           // 最小输出（最小转向速度）
} Angle_Position_PID;

extern int16_t LeftPWM, RightPWM;         // 左右电机的PWM值
extern uint8_t is_angle_turning;  // 是否正在执行转向动作的标志

//============================================================
// UWB跟随参数设置（参照商家 Followingcar1.2 原始值方案）
//   Motor_Set 输入范围: -10000 ~ 10000
//   商家公式: PWM_diff = |raw_azim| × Dis (Dis=50)
//============================================================
// uwb跟随函数参数定义
// UWB 跟随参数
#define UWB_FOLLOW_DIST_M       1.5f    // 跟随距离阈值（米），超过此距离停车
#define UWB_FOLLOW_BASE_SPEED   5000    // 基础速度（0~10000），值越大越快
#define UWB_FOLLOW_STEER_COEFF  600.0f  // 转向灵敏度系数，方位角(度) × 此值 = 差速量
#define UWB_FOLLOW_CALL_MS      50      // uwb_follow 调用间隔（毫秒），需与 main.cpp 中 number_task 计数一致
#define UWB_FOLLOW_TIMEOUT_MS   200     // UWB 数据超时（毫秒），超时停车

// navigate_process 综合导航参数
#define NAV_AVOID_DIST_MM       330     // 避障距离阈值（毫米）
#define NAV_FRONT_ANGLE         270.0f  // 雷达正前方角度
#define NAV_FOV_DEG             120.0f  // 避障检测视场角（度），以正前方为中心 ±60°
#define NAV_UWB_DIST_MIN_M      0.4f    // UWB 跟随最小距离（米），≤ 此值停车
#define NAV_UWB_DIST_MAX_M      6.0f    // UWB 跟随最大距离（米），≥ 此值停车
#define NAV_UWB_TIMEOUT_MS      200     // UWB 数据超时（毫秒）
#define NAV_CALL_MS             50      // navigate_process 调用间隔（毫秒）
// 超时调用次数 = NAV_UWB_TIMEOUT_MS / NAV_CALL_MS = 4
#define NAV_AVOID_SPEED         5000    // 避障转向速度

// 雷达避障函数
void avoid(void);

// 转向控制函数
void Start_Angle_Turn(float angle);
void Stop_Angle_Turn(void);
uint8_t Update_Angle_Turn(void);
uint8_t Is_Angle_Turning(void);
float Get_Angle_Turn_Error(void);

// uwb导航函数
void uwb_follow(void);
void navigate_process(void);

#endif