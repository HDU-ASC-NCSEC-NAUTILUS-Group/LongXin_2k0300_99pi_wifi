#include "zf_common_headfile.h"
#include <math.h>

struct pwm_info servo_pwm_info;

float servo_motor_duty = 90.0;                                                  // 舵机动作角度
float servo_motor_dir = 1;                                                      // 舵机动作状态

// 全局变量（不变）
int16_t coordinate_x = 0;
int16_t coordinate_y = 0;

// 机械臂相应数据
#define Arm_1     8.2f    // 机械臂第一个关节长度（单位：cm），根据实际情况调整
#define Arm_2     8.0f    // 机械臂第二个关节长度（单位：cm），根据实际情况调整
#define alpha     35.0f   // 角度1的起始位置与z轴的夹角
#define h         1.3f    // 抓取物块的高度（单位：cm），根据实际情况调整
#define delta_x   8.8f    // 摄像头与机械臂基座的水平距离（单位：cm），根据实际情况调整 8.8~9.0cm
#define ANGLE_STEP 0.5f    // 角度步进（越小越丝滑）
#define STATE_DELAY 500    // 状态持续时间（ms）

// 机械臂四个转角
float angle_0 = 0.0f;   // 基座旋转角度
float angle_1 = 0.0f;   // 控制第一个机械臂
float angle_2 = 0.0f;   // 控制第二个机械臂
float angle_3 = 0.0f;   // 控制末端执行器（夹爪）开合程度

float len = sqrt(real_x * real_x + real_y * real_y);  // 目标点与机械臂基座的距离

float L = len - delta_x;  // 目标点与机械臂基座的水平距离
float H = h;              // 机械臂的高度

// 机械臂控制实例
ArmControl arm_ctrl = {
    .current_state = ARM_STATE_IDLE,
    .start_flag = false,
    .stop_flag = false,

    .target_angle0 = 90.0f,  // 初始基座角度（中间位）
    .target_angle1 = 35.0f,  // 初始关节1角度
    .target_angle2 = 90.0f,  // 初始关节2角度
    .target_angle3 = 0.0f,   // 初始夹爪打开（0=最大打开）根据实际修改

    .current_angle0 = 90.0f,
    .current_angle1 = 35.0f,
    .current_angle2 = 90.0f,
    .current_angle3 = 0.0f,

    .stable_count = 0,
    .last_pixel_x = 0,
    .last_pixel_y = 0,
    .last_real_x = 0.0f,
    .last_real_y = 0.0f
};

// 根据摄像头获取的实际坐标解算angle_0、angle_1、angle_2、angle_3，控制机械臂夹取物块
void servo_control(void)
{
    angle_0 = atan2(real_x, real_y) * 180.0f / CV_PI;  // 基座旋转角度

    // 计算得到angle_1和angle_2
    int Angle_1 = asin((pow(L,2.0) + pow(H,2.0) + pow(Arm_1,2.0) - pow(Arm_2,2.0)) / (2 * Arm_1 * sqrt(pow(L,2.0) + pow(H,2.0)))) * 180.0f / CV_PI - atan2(H, L) * 180.0f / CV_PI;  // 第一个机械臂的角度
    int Angle_2 = acos((pow(L,2.0) + pow(H,2.0) + pow(Arm_2,2.0) - pow(Arm_1,2.0)) / (2 * Arm_2 * sqrt(pow(L,2.0) + pow(H,2.0)))) * 180.0f / CV_PI - atan2(H, L) * 180.0f / CV_PI;  // 第二个机械臂的角度

    angle_1 = Angle_1 + alpha;  // 调整第一个机械臂的角度
    angle_2 = angle_1 + 90.0f - alpha - Angle_2;  // 调整第二个机械臂的角度

    /*
    设置一个标志位，刚开始，angle_3 = 0.0f 夹子打开到最大
    当机械臂移动到目标位置时，angle_3 = 180.0f 夹子关闭，抓取物块
    再写一个机械臂复位代码，形成一套动作链
    */
}

// 机械臂初始化
void servo_init(void)
{
    // 获取PWM设备信息
    pwm_get_dev_info(SERVO_MOTOR1_PWM, &servo_pwm_info);
    // 设置初始角度
    pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(arm_ctrl.current_angle0));
    pwm_set_duty(SERVO_MOTOR2_PWM, (uint16)SERVO_MOTOR_DUTY(arm_ctrl.current_angle1));
    pwm_set_duty(SERVO_MOTOR3_PWM, (uint16)SERVO_MOTOR_DUTY(arm_ctrl.current_angle2));
    pwm_set_duty(SERVO_MOTOR4_PWM, (uint16)SERVO_MOTOR_DUTY(arm_ctrl.current_angle3));
    arm_ctrl.start_flag = true; // 启动状态机
}

// 平滑调整角度（避免突变）
void servo_set_angle_smooth(float *current, float target, float step)
{
    if(fabs(*current - target) < step)
    {
        *current = target;
    }
    else if(*current < target)
    {
        *current += step;
    }
    else
    {
        *current -= step;
    }
}

// 解算目标角度（根据物理坐标）
static void calc_target_angles(void)
{
    if(real_x < -999.0f || real_y < -999.0f) return; // 无有效坐标
    
    // 基座角度（angle0）：目标在基座坐标系的水平角度
    arm_ctrl.target_angle0 = atan2(real_x, real_y) * 180.0f / CV_PI + 90.0f;   // 可能需要根据实际修改
    // 限制基座角度范围
    arm_ctrl.target_angle0 = fmax(SERVO_MOTOR_L_MAX, fmin(SERVO_MOTOR_R_MAX, arm_ctrl.target_angle0));

    // 目标与基座的水平距离
    float len = sqrt(real_x * real_x + real_y * real_y) - delta_x;
    float L = len;
    float H = h;

    // 解算关节1/2角度（逆运动学）
    float sqrt_LH = sqrt(pow(L,2) + pow(H,2));
    if(sqrt_LH < fabs(Arm_1 - Arm_2) || sqrt_LH > (Arm_1 + Arm_2)) return; // 超出工作范围

    float angle1_rad = asin((pow(L,2) + pow(H,2) + pow(Arm_1,2) - pow(Arm_2,2)) / (2 * Arm_1 * sqrt_LH)) - atan2(H, L);
    arm_ctrl.target_angle1 = angle1_rad * 180.0f / CV_PI + alpha;

    float angle2_rad = acos((pow(L,2) + pow(H,2) + pow(Arm_2,2) - pow(Arm_1,2)) / (2 * Arm_2 * sqrt_LH));
    arm_ctrl.target_angle2 = arm_ctrl.target_angle1 + 90.0f - alpha - (angle2_rad * 180.0f / CV_PI);

    // 限制关节角度范围
    arm_ctrl.target_angle1 = fmax(0.0f, fmin(180.0f, arm_ctrl.target_angle1));
    arm_ctrl.target_angle2 = fmax(0.0f, fmin(180.0f, arm_ctrl.target_angle2));
    // 抓取时夹爪目标角度（180=闭合）
    arm_ctrl.target_angle3 = 180.0f;
}

// 状态机核心处理函数（需在主循环调用）
void servo_state_machine(void)
{
    if(arm_ctrl.stop_flag) // 强制停止
    {
        arm_ctrl.current_state = ARM_STATE_RESETTING;
        arm_ctrl.stop_flag = false;
    }

    switch(arm_ctrl.current_state)
    {
        case ARM_STATE_IDLE: // 空闲状态：等待启动，检测到目标则进入追踪
            if(arm_ctrl.start_flag && (coordinate_x != -1 && coordinate_y != -1))
            {
                arm_ctrl.current_state = ARM_STATE_TRACKING;
                arm_ctrl.stable_count = 0; // 重置计数
                arm_ctrl.last_pixel_x = coordinate_x; // 记录初始坐标
                arm_ctrl.last_pixel_y = coordinate_y;
            }
            break;

        case ARM_STATE_TRACKING: // 追踪状态：解算目标角度
            calc_target_angles();
            // 追踪5秒后进入移动状态
            if(coordinate_x == arm_ctrl.last_pixel_x && coordinate_y == arm_ctrl.last_pixel_y)
            {
                arm_ctrl.stable_count++;
            }
            else
            {
                arm_ctrl.stable_count = 0; // 坐标变化，清零计数
                arm_ctrl.last_pixel_x = coordinate_x; // 更新基准坐标
                arm_ctrl.last_pixel_y = coordinate_y;
            }

            // 连续稳定15次 → 切换到移动状态
            if(arm_ctrl.stable_count >= STABLE_COUNT)
            {
                arm_ctrl.current_state = ARM_STATE_MOVING;
                arm_ctrl.stable_count = 0; // 重置计数
            }
            break;

        case ARM_STATE_MOVING: // 移动状态：平滑调整到目标角度
            // 逐个关节平滑调整
            servo_set_angle_smooth(&arm_ctrl.current_angle0, arm_ctrl.target_angle0, ANGLE_STEP);
            servo_set_angle_smooth(&arm_ctrl.current_angle1, arm_ctrl.target_angle1, ANGLE_STEP);
            servo_set_angle_smooth(&arm_ctrl.current_angle2, arm_ctrl.target_angle2, ANGLE_STEP);
            // 写入PWM
            pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(arm_ctrl.current_angle0));
            pwm_set_duty(SERVO_MOTOR2_PWM, (uint16)SERVO_MOTOR_DUTY(arm_ctrl.current_angle1));
            pwm_set_duty(SERVO_MOTOR3_PWM, (uint16)SERVO_MOTOR_DUTY(arm_ctrl.current_angle2));
            // 角度到位后进入抓取状态
            if(fabs(arm_ctrl.current_angle0 - arm_ctrl.target_angle0) < 0.1f &&
               fabs(arm_ctrl.current_angle1 - arm_ctrl.target_angle1) < 0.1f &&
               fabs(arm_ctrl.current_angle2 - arm_ctrl.target_angle2) < 0.1f)
            {
                arm_ctrl.current_state = ARM_STATE_GRASPING;
            }
            break;

        case ARM_STATE_GRASPING: // 抓取状态：闭合夹爪
            servo_set_angle_smooth(&arm_ctrl.current_angle3, arm_ctrl.target_angle3, ANGLE_STEP);
            pwm_set_duty(SERVO_MOTOR4_PWM, (uint16)SERVO_MOTOR_DUTY(arm_ctrl.current_angle3));
            // 闭合完成后抬升（轻微）
            if(fabs(arm_ctrl.current_angle3 - arm_ctrl.target_angle3) < 0.1f)
            {
                arm_ctrl.target_angle1 += 5.0f; // 抬升5度
                arm_ctrl.current_state = ARM_STATE_LIFTING;
            }
            break;

        case ARM_STATE_LIFTING: // 抬升状态：保持500ms
            servo_set_angle_smooth(&arm_ctrl.current_angle1, arm_ctrl.target_angle1, ANGLE_STEP);
            pwm_set_duty(SERVO_MOTOR2_PWM, (uint16)SERVO_MOTOR_DUTY(arm_ctrl.current_angle1));
            if(fabs(arm_ctrl.current_angle1 - arm_ctrl.target_angle1) < 0.1f)
            {
                arm_ctrl.current_state = ARM_STATE_RESETTING;
            }
            break;

        case ARM_STATE_RESETTING: // 复位状态：回到初始位置，打开夹爪
            // 复位目标角度
            arm_ctrl.target_angle0 = 90.0f;
            arm_ctrl.target_angle1 = 35.0f;
            arm_ctrl.target_angle2 = 90.0f;
            arm_ctrl.target_angle3 = 0.0f;
            // 平滑复位
            servo_set_angle_smooth(&arm_ctrl.current_angle0, arm_ctrl.target_angle0, ANGLE_STEP);
            servo_set_angle_smooth(&arm_ctrl.current_angle1, arm_ctrl.target_angle1, ANGLE_STEP);
            servo_set_angle_smooth(&arm_ctrl.current_angle2, arm_ctrl.target_angle2, ANGLE_STEP);
            servo_set_angle_smooth(&arm_ctrl.current_angle3, arm_ctrl.target_angle3, ANGLE_STEP);
            // 写入PWM
            pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(arm_ctrl.current_angle0));
            pwm_set_duty(SERVO_MOTOR2_PWM, (uint16)SERVO_MOTOR_DUTY(arm_ctrl.current_angle1));
            pwm_set_duty(SERVO_MOTOR3_PWM, (uint16)SERVO_MOTOR_DUTY(arm_ctrl.current_angle2));
            pwm_set_duty(SERVO_MOTOR4_PWM, (uint16)SERVO_MOTOR_DUTY(arm_ctrl.current_angle3));
            // 复位完成回到空闲
            if(fabs(arm_ctrl.current_angle0 - 90.0f) < 0.1f &&
               fabs(arm_ctrl.current_angle1 - 35.0f) < 0.1f &&
               fabs(arm_ctrl.current_angle2 - 90.0f) < 0.1f &&
               fabs(arm_ctrl.current_angle3 - 0.0f) < 0.1f)
            {
                arm_ctrl.current_state = ARM_STATE_IDLE;
                arm_ctrl.start_flag = true; // 复位后可再次启动
            }
            break;

        default:
            arm_ctrl.current_state = ARM_STATE_IDLE;
            break;
    }
}

void servo_test(void)
{
    servo_state_machine();
}

// 在龙芯上解算出各个舵机的角度，发送到arduino上处理
static int16_t calculate_angle_0(void)
{
    angle_0 = atan2(real_x, real_y) * 180.0f / CV_PI;  // 基座旋转角度
    return (int16_t)(angle_0);
}

static int16_t calculate_angle_1(void)
{
    int Angle_1 = asin((pow(L,2.0) + pow(H,2.0) + pow(Arm_1,2.0) - pow(Arm_2,2.0)) / (2 * Arm_1 * sqrt(pow(L,2.0) + pow(H,2.0)))) * 180.0f / CV_PI - atan2(H, L) * 180.0f / CV_PI;  // 第一个机械臂的角度
    angle_1 = Angle_1 + alpha;  // 调整第一个机械臂的角度
    return (int16_t)(angle_1);
}

static int16_t calculate_angle_2(void)
{
    int Angle_2 = acos((pow(L,2.0) + pow(H,2.0) + pow(Arm_2,2.0) - pow(Arm_1,2.0)) / (2 * Arm_2 * sqrt(pow(L,2.0) + pow(H,2.0)))) * 180.0f / CV_PI - atan2(H, L) * 180.0f / CV_PI;  // 第二个机械臂的角度
    angle_2 = angle_1 + 90.0f - alpha - Angle_2;  // 调整第二个机械臂的角度
    return (int16_t)(angle_2);
}

