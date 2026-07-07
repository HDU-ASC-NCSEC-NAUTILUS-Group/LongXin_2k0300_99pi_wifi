#include "zf_common_headfile.h"
#include <math.h>

// -----------------------------------------------------------------------------------
// 变量申明
uint8_t text_cnt = 0;
float text_angle = 0;

//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
// 函数简介     数据处理函数，小车前进避障，只需要前方点的数据
// 使用示例     data_process();
//------------------------------------------------------------------------------------

void avoid(void)
{
    volatile uint8_t i = 0;
    volatile uint8_t calculation_angle_cnt = 0;
    float angle_sum = 0;
    uint8_t flag_avoid = 0;
    static uint16_t distance = 330;          // 避障阈值 mm

    const float FRONT_ANGLE = 270.0f;        // ★ 雷达 0° 对应的方向？这里按正前方 = 270° 设定

    for(i = 0; i < 50; i++)
    {
        uint16_t d = PointDataProcess[i].distance;
        if(d > 0 && d < distance)
        {
            float raw_angle = PointDataProcess[i].angle;

            // 1. 计算相对于正前方的偏离角（范围 -180° ~ 180°）
            float diff = raw_angle - FRONT_ANGLE;
            if(diff > 180.0f)       diff -= 360.0f;
            else if(diff < -180.0f)  diff += 360.0f;

            // 2. 累加偏离角（左侧为负，右侧为正）
            angle_sum += diff;
            calculation_angle_cnt++;
        }
    }

    if(calculation_angle_cnt == 0)          // 无障碍
    {
        Motor_Move_Straight(2500);
        flag_avoid = 0;
    }
    else
    {
        float avg_angle = angle_sum / calculation_angle_cnt;

        // avg_angle < 0 → 障碍物偏左 → 需向右转
        if(avg_angle < 0)
        {
            Motor_Spot_Right(3600);
            flag_avoid = 1;             // 右转
        }
        else                             // avg_angle > 0 → 偏右 → 左转
        {
            Motor_Spot_Left(3600);
            flag_avoid = 2;             // 左转
        }
    }

    text_cnt = calculation_angle_cnt;
    text_angle = angle_sum;
    printf("避障状态：%d\n", flag_avoid);
}

//====================================================================================

// -----------------------------------------------------------------------------------
// 函数简介     转向函数，执行转向动作
// 参数说明     
// 使用示例     
// -----------------------------------------------------------------------------------
int16_t turn_difpwm = 0;               // 转向差速PWM值
int16_t LeftPWM, RightPWM = 0;         // 左右电机的PWM值

uint8_t is_angle_turning = 0;          // 是否正在执行转向动作的标志
float target_angle_increment = 0.0f;   // 目标角度增量
float initial_yaw = 0.0f;              // 转向开始时的初始角度

// ------------------ 角度位置PID结构体（单级） ------------------
Angle_Position_PID angle_pos_pid = {
    .Kp = 3250.0f,       // 比例系数，需根据实际调试
    .Ki = 0.0f,       // 积分系数
    .Kd = 0.0f,       // 微分系数

    .OutMax = 16000.0f,    // 最大差速PWM输出
    .OutMin = -16000.0f,   // 最小差速PWM输出
};

// ------------------ 内部PID计算 ------------------
static void AnglePositionPID_Update(Angle_Position_PID *pid)
{
    // 计算角度误差，归一化到 [-180, 180]
    pid->error = pid->target - pid->actual;
    while (pid->error > 180.0f)  pid->error -= 360.0f;
    while (pid->error < -180.0f) pid->error += 360.0f;

    // 积分项（带限幅）
    if (pid->Ki != 0) {
        pid->error_integral += pid->error;
        if (pid->error_integral > pid->integral_max)
            pid->error_integral = pid->integral_max;
        if (pid->error_integral < -pid->integral_max)
            pid->error_integral = -pid->integral_max;
    }

    // 微分项（也需归一化，防止yaw环绕时error跳变导致D项尖峰）
    float error_diff = pid->error - pid->last_error;
    while (error_diff > 180.0f)  error_diff -= 360.0f;
    while (error_diff < -180.0f) error_diff += 360.0f;
    pid->last_error = pid->error;

    // PID输出
    pid->output = pid->Kp * pid->error
                + pid->Ki * pid->error_integral
                + pid->Kd * error_diff;

    // 输出限幅
    if (pid->output > pid->OutMax) pid->output = pid->OutMax;
    if (pid->output < pid->OutMin) pid->output = pid->OutMin;
}

// ------------------ 应用差速到电机 ------------------
static void Apply_Differential_Steer(float diff_pwm)
{
    // 直接设置差速，同时保留平均速度（直行分量由其他任务控制）
    turn_difpwm = (int16_t)diff_pwm;
    LeftPWM = -turn_difpwm / 2;
    RightPWM = turn_difpwm / 2;
}

// ------------------ 开始角度转向任务 ------------------
// >0 右转（顺时针） <0 左转（逆时针）
void Start_Angle_Turn(float angle)
{
    if (is_angle_turning) {
        Stop_Angle_Turn();
    }

    is_angle_turning = 1;
    target_angle_increment = angle;
    initial_yaw = Yaw_Result;

    // 初始化位置PID目标
    angle_pos_pid.target = initial_yaw + angle;
    while (angle_pos_pid.target > 180.0f)  angle_pos_pid.target -= 360.0f;
    while (angle_pos_pid.target < -180.0f) angle_pos_pid.target += 360.0f;
    angle_pos_pid.actual = initial_yaw;
    angle_pos_pid.error = 0;
    angle_pos_pid.last_error = 0;
    angle_pos_pid.error_integral = 0;
    angle_pos_pid.integral_max = 100.0f;   // 积分上限，可调

    // 初始化差速为0，避免突变
    Apply_Differential_Steer(0.0f);
}

// ------------------ 停止角度转向任务 ------------------
void Stop_Angle_Turn(void)
{
    is_angle_turning = 0;
    angle_pos_pid.error_integral = 0;       // 清空积分
    angle_pos_pid.output = 0.0f;           // 清空输出
    Apply_Differential_Steer(0.0f);         // 差速回零，停止转向
}

// ------------------ 转向任务更新（中断中周期调用） ------------------
uint8_t Update_Angle_Turn(void)
{
    if (!is_angle_turning) return 1;

    // 更新当前角度
    angle_pos_pid.actual = Yaw_Result;

    // 计算位置PID
    AnglePositionPID_Update(&angle_pos_pid);

    // 位置环输出直接作为差速PWM，负号用于匹配方向（可现场调整）
    float diff_output = -angle_pos_pid.output;   // 负号取决于你的电机接线
    Apply_Differential_Steer(diff_output);

    // 检查是否到达目标角度（带死区）
    float angle_error = fabsf(angle_pos_pid.target - Yaw_Result);
    if (angle_error < 1.0f) {   // 1度误差内认为完成
        Stop_Angle_Turn();
        return 1;               // 转向完成
    }

    return 0;                   // 转向中
}

// ------------------ 状态查询 ------------------
uint8_t Is_Angle_Turning(void)
{
    return is_angle_turning;
}

float Get_Angle_Turn_Error(void)
{
    return angle_pos_pid.error;
}

//====================================================================================

//------------------------------------------------------------------------------------
// 函数简介     UWB 跟随函数，基于方位角差速转向 + 距离门控
// 核心思路     对齐商家 STM32 代码 DriveProcess()：Azimuth<0 左侧减速左转，
//              Azimuth>=0 右侧减速右转，Distance>阈值停车，Distance≤阈值前进
// 使用示例     在 pit_callback_10ms() 中 uwb_usart_task() 之后调用 uwb_follow()
// 备注信息     电机 1/2 = 左侧（共用 PWM），电机 3/4 = 右侧（共用 PWM）
//              PWM 范围 -10000~10000，正值前进、负值后退
//------------------------------------------------------------------------------------
void uwb_follow(void)
{
    static uint32_t last_frame_count = 0;
    static uint32_t stale_calls = 0;   // 连续无新帧的调用次数（每次=50ms）

    // ---- 检测是否有新帧到达 ----
    if (g_uwb_frame_count != last_frame_count)
    {
        last_frame_count = g_uwb_frame_count;
        stale_calls = 0;
    }
    else
    {
        stale_calls++;
    }

    // ---- 从未收到过任何 UWB 帧 → 不动 ----
    if (g_uwb_frame_count == 0)
    {
        Motor_Reset_ALL();
        return;
    }

    // ---- 超时保护：4 次调用无新帧 = 200ms → 停车 ----
    if (stale_calls >= UWB_FOLLOW_TIMEOUT_MS / UWB_FOLLOW_CALL_MS)
    {
        Motor_Reset_ALL();
        printf("[FOLLOW] timeout, no new frame for %d calls\r\n", (int)stale_calls);
        return;
    }

    // ---- 距离门控 ----
    float dist = g_uwb_data.distance_m;
    if (dist > UWB_FOLLOW_DIST_M)
    {
        Motor_Reset_ALL();
        return;
    }

    // ---- 差速转向 ----
    float azim = g_uwb_data.azimuth_f;

    int pwmL = UWB_FOLLOW_BASE_SPEED;
    int pwmR = UWB_FOLLOW_BASE_SPEED;

    int steer = (int)(std::fabs(azim) * UWB_FOLLOW_STEER_COEFF);

    if (azim < 0.0f)
    {
        pwmR = UWB_FOLLOW_BASE_SPEED - steer;
    }
    else
    {
        pwmL = UWB_FOLLOW_BASE_SPEED - steer;
    }

    // 下限钳位（PWM 不为负）
    if (pwmL < 0) pwmL = 0;
    if (pwmR < 0) pwmR = 0;

    // 输出
    Motor_Set(1, pwmL);
    Motor_Set(2, pwmL);
    Motor_Set(3, pwmR);
    Motor_Set(4, pwmR);

    printf("[FOLLOW] dist=%.2fm azim=%.1f steer=%d L=%d R=%d\r\n",
           dist, azim, steer, pwmL, pwmR);
}

//------------------------------------------------------------------------------------
// 函数简介     navigate_process 综合导航函数，避障 + UWB 跟随一体化
//              避障优先级 > 跟随：跟随途中遇到障碍物先执行避障
//              雷达正前方 = 270°，检测视场角 = 120° (210° ~ 330°)
//              UWB 距离门控：≤0.4m 停车（太近），≥6m 停车（太远/丢失）
//              数据超时 200ms 自动停车
// 使用示例     在 pit_callback_xxms() 中 uwb_usart_task() 之后调用 navigate_process()
//------------------------------------------------------------------------------------
void navigate_process(void)
{
    // ============================================================
    // 第一优先级：雷达避障
    // ============================================================

    const float half_fov = NAV_FOV_DEG * 0.5f;  // ±60°
    uint8_t  obs_cnt = 0;
    float    angle_sum = 0;

    for (int i = 0; i < 50; i++)
    {
        uint16_t d = PointDataProcess[i].distance;
        if (d > 0 && d < NAV_AVOID_DIST_MM)
        {
            float raw_angle = PointDataProcess[i].angle;

            // 相对正前方的偏离角 [-180, 180]
            float diff = raw_angle - NAV_FRONT_ANGLE;
            if (diff > 180.0f)       diff -= 360.0f;
            else if (diff < -180.0f)  diff += 360.0f;

            // 仅统计 120° 视场角内的障碍点
            if (std::fabs(diff) <= half_fov)
            {
                angle_sum += diff;
                obs_cnt++;
            }
        }
    }

    if (obs_cnt > 0)
    {
        float avg_angle = angle_sum / obs_cnt;

        // avg_angle < 0 → 障碍物偏左 → 右转
        if (avg_angle < 0)
        {
            Motor_Spot_Right(NAV_AVOID_SPEED);
        }
        else
        {
            Motor_Spot_Left(NAV_AVOID_SPEED);
        }

        printf("[NAV] avoid obs=%d avg=%.1f\r\n", obs_cnt, avg_angle);
        return;  // 避障优先，跳过跟随
    }

    // ============================================================
    // 第二优先级：UWB 跟随（前方无障碍时执行）
    // ============================================================

    static uint32_t last_frame_count = 0;
    static uint32_t stale_calls = 0;

    // ---- 新帧检测 ----
    if (g_uwb_frame_count != last_frame_count)
    {
        last_frame_count = g_uwb_frame_count;
        stale_calls = 0;
    }
    else
    {
        stale_calls++;
    }

    // ---- 从未收到过 UWB 数据 → 停车 ----
    if (g_uwb_frame_count == 0)
    {
        Motor_Reset_ALL();
        return;
    }

    // ---- 超时保护 ----
    if (stale_calls >= NAV_UWB_TIMEOUT_MS / NAV_CALL_MS)
    {
        Motor_Reset_ALL();
        printf("[NAV] uwb timeout, stale=%d calls\r\n", (int)stale_calls);
        return;
    }

    // ---- 距离门控 ----
    float dist = g_uwb_data.distance_m;

    if (dist <= NAV_UWB_DIST_MIN_M)       // 太近（≤0.4m）
    {
        Motor_Reset_ALL();
        printf("[NAV] too close, dist=%.2fm\r\n", dist);
        return;
    }

    if (dist >= NAV_UWB_DIST_MAX_M)       // 太远（≥6m）/ 丢失
    {
        Motor_Reset_ALL();
        printf("[NAV] too far, dist=%.2fm\r\n", dist);
        return;
    }

    // ---- 差速转向跟随 ----
    float azim = g_uwb_data.azimuth_f;

    int pwmL = UWB_FOLLOW_BASE_SPEED;
    int pwmR = UWB_FOLLOW_BASE_SPEED;

    int steer = (int)(std::fabs(azim) * UWB_FOLLOW_STEER_COEFF);

    if (azim < 0.0f)
    {
        pwmR = UWB_FOLLOW_BASE_SPEED - steer;
    }
    else
    {
        pwmL = UWB_FOLLOW_BASE_SPEED - steer;
    }

    if (pwmL < 0) pwmL = 0;
    if (pwmR < 0) pwmR = 0;

    Motor_Set(1, pwmL);
    Motor_Set(2, pwmL);
    Motor_Set(3, pwmR);
    Motor_Set(4, pwmR);

    printf("[NAV] follow dist=%.2fm azim=%.1f steer=%d L=%d R=%d\r\n",
           dist, azim, steer, pwmL, pwmR);
}

