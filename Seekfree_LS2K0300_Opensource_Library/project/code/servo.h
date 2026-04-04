#ifndef __servo_h
#define __servo_h

// 定义主板上舵机频率  请务必注意范围 50-300
// 如果要修改，需要直接修改设备树。
#define SERVO_MOTOR_FREQ            (servo_pwm_info.freq)                       

// 在设备树中，默认设置的10000。如果要修改，需要直接修改设备树。
#define PWM_DUTY_MAX                (servo_pwm_info.duty_max)      

// 定义主板上舵机活动范围 角度                                                     
#define SERVO_MOTOR_L_MAX           (0)                       
#define SERVO_MOTOR_R_MAX           (180)      

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

// 定义驱动路径，该路径由设备树生成
#define SERVO_MOTOR1_PWM            "/dev/zf_device_pwm_servo"
#define SERVO_MOTOR2_PWM            "/dev/zf_device_pwm_servo2" // 关节2
#define SERVO_MOTOR3_PWM            "/dev/zf_device_pwm_servo3" // 关节3
#define SERVO_MOTOR4_PWM            "/dev/zf_device_pwm_servo4" // 夹爪     

// 坐标外部声明
extern int16_t coordinate_x;
extern int16_t coordinate_y;

// 从坐标转换函数获取的物理坐标
extern float real_x, real_y;

// 获取系统时间
extern int32_t time_number;

// 机械臂状态机（贴合抓取流程）
typedef enum
{
    ARM_STATE_IDLE,         // 空闲（初始状态，等待启动）
    ARM_STATE_TRACKING,     // 追踪目标（解算角度，调整基座）
    ARM_STATE_MOVING,       // 移动到目标位置（平滑调整关节角度）
    ARM_STATE_GRASPING,     // 闭合夹爪抓取
    ARM_STATE_LIFTING,      // 抓取后轻微抬升（防脱落）
    ARM_STATE_RESETTING     // 复位（回到初始位置，打开夹爪）
} ArmState;

// 机械臂控制结构体
typedef struct
{
    ArmState current_state; // 当前状态
    bool start_flag;        // 启动标志（true=启动抓取流程）
    bool stop_flag;         // 停止标志（true=强制停止）
    // 目标角度（解算后的值）
    float target_angle0;    // 基座
    float target_angle1;    // 关节1
    float target_angle2;    // 关节2
    float target_angle3;    // 夹爪
    // 当前角度（用于平滑过渡）
    float current_angle0;   
    float current_angle1;
    float current_angle2;
    float current_angle3;
    // 状态计时（控制每个状态的持续时间）
    uint32_t state_timer;
} ArmControl;

// 全局机械臂控制实例
extern ArmControl arm_ctrl;

//函数声明
void servo_init(void);        // 机械臂初始化
void servo_state_machine(void);// 状态机处理核心函数
void servo_set_angle_smooth(float *current, float target, float step); // 平滑角度调整
void servo_control(void);

#endif
