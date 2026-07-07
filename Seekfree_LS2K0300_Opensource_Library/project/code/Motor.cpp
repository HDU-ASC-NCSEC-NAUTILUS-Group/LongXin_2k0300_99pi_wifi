/********************************************************************************************************************
 * 电机控制
 ********************************************************************************************************************/
#include "zf_common_headfile.h"


struct pwm_info motor_1_pwm_info;
struct pwm_info motor_2_pwm_info;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机相关初始化
//-------------------------------------------------------------------------------------------------------------------
void Motor_Init(void)
{
    pwm_get_dev_info(init_PWM1_P65, &motor_1_pwm_info);
    pwm_get_dev_info(init_PWM2_P66, &motor_2_pwm_info);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机速度
// 使用示例     Motor_Set(1, 2500);
// 参数说明     num：指定电机编号，范围：1-4
//   #1 [][][][][] 3#
//   #1 [][][][][] 3#
//      [][][][][]
//      [][][][][]
//   #2 [][][][][] 4#
//   #2 [][][][][] 4#
// 参数说明     duty：指定对应电机侧占空比，范围：-10000~10000（注：duty本意是没有负数的）
// 备注信息     1、2电机共用PWM，3、4电机共用PWM
// 备注信息     以物理实际连接为准，并注意.h文件中的宏定义“映射”
//-------------------------------------------------------------------------------------------------------------------

void Motor_Set(int num, int duty)
{
    // 占空比溢出保护
    if (duty > DUTY_MAX){duty = DUTY_MAX;}
    else if (duty < -DUTY_MAX){duty = -DUTY_MAX;}


    // 速度设置为正向
    if (duty >= 0)
    {
        switch (num)
        {
            case 1:
                gpio_set_level(MOTOR_1_DIR, 1);                                      // DIR输出高电平
                pwm_set_duty(MOTOR_1_PWM, duty);       // 计算占空比
                break;
            case 2:
                gpio_set_level(MOTOR_2_DIR, 1);                                      // DIR输出高电平
                pwm_set_duty(MOTOR_2_PWM, duty);       // 计算占空比
                break;
            case 3:
                gpio_set_level(MOTOR_3_DIR, 1);                                      // DIR输出高电平
                pwm_set_duty(MOTOR_3_PWM, duty);       // 计算占空比
                break;
            case 4:
                gpio_set_level(MOTOR_4_DIR, 1);                                      // DIR输出高电平
                pwm_set_duty(MOTOR_4_PWM, duty);       // 计算占空比
                break;
            default:
                // 留一手，防止其他电机编号错误导致的错误
                break;
        }

    }
    // 速度设置为反向
    else
    {
        switch (num)
        {
            case 1:
                gpio_set_level(MOTOR_1_DIR, 0);                                      // DIR输出低电平
                pwm_set_duty(MOTOR_1_PWM, -duty);      // 计算占空比
                break;
            case 2:
                gpio_set_level(MOTOR_2_DIR, 0);                                      // DIR输出低电平
                pwm_set_duty(MOTOR_2_PWM, -duty);      // 计算占空比
                break;
            case 3:
                gpio_set_level(MOTOR_3_DIR, 0);                                      // DIR输出低电平
                pwm_set_duty(MOTOR_3_PWM, -duty);      // 计算占空比
                break;
            case 4:
                gpio_set_level(MOTOR_4_DIR, 0);                                      // DIR输出低电平
                pwm_set_duty(MOTOR_4_PWM, -duty);      // 计算占空比
                break;
            default:
                // 留一手，防止其他电机编号错误导致的错误
                break;
        }
    } 
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置电机速度为0
// 使用示例     Motor_Reset();
//-------------------------------------------------------------------------------------------------------------------
void Motor_Reset_ALL(void)
{
    Motor_Set(1, 0);
    Motor_Set(2, 0);
    Motor_Set(3, 0);
    Motor_Set(4, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     预制速度动作
// 参数说明     (如果有)speed：指定速度，范围：-100-100
//-------------------------------------------------------------------------------------------------------------------
// 直行
// 建议速度：20
void Motor_Move_Straight(int16_t speed)
{
    if (speed > DUTY_MAX){speed = DUTY_MAX;}
    else if (speed < -DUTY_MAX){speed = -DUTY_MAX;}
        
    Motor_Set(1, speed);
    Motor_Set(2, speed);
    Motor_Set(3, speed);
    Motor_Set(4, speed);
}

// 原地左转
// 建议速度：25
void Motor_Spot_Left(uint16_t speed)
{
    if (speed > DUTY_MAX){speed = DUTY_MAX;}
    else if (speed < -DUTY_MAX){speed = -DUTY_MAX;}
    Motor_Set(1, -speed);
    Motor_Set(2, -speed);
    Motor_Set(3, speed);
    Motor_Set(4, speed);
}

// 原地右转
// 建议速度：25
void Motor_Spot_Right(uint16_t speed)
{
    if (speed > DUTY_MAX){speed = DUTY_MAX;}
    else if (speed < -DUTY_MAX){speed = -DUTY_MAX;}
    Motor_Set(1, speed);
    Motor_Set(2, speed);
    Motor_Set(3, -speed);
    Motor_Set(4, -speed);
}

// 电机定角度转向函数
void Motor_Output_Turn(void) {
    Motor_Set(1, LeftPWM);
    Motor_Set(2, LeftPWM);    
    Motor_Set(3, RightPWM);
    Motor_Set(4, RightPWM);
}
