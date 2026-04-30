/********************************************************************************************************************
 * 电机控制
 ********************************************************************************************************************/
#include "zf_common_headfile.h"

#include "defines.h"
#include "Motor.h"


struct pwm_info motor_1_pwm_info;
struct pwm_info motor_2_pwm_info;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机相关初始化
//-------------------------------------------------------------------------------------------------------------------
void Motor_Init(void)
{
    pwm_get_dev_info(MOTOR1_PWM, &motor_1_pwm_info);
    pwm_get_dev_info(MOTOR2_PWM, &motor_2_pwm_info);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机速度
// 使用示例     Motor_Set(1, 500);
// 参数说明     num：指定电机编号，范围：1-4
// #1 [][][] 3#
// #1 [][][] 3#
//    [][][]
//    [][][]
// #2 [][][] 4#
// #2 [][][] 4#
 // PWM共用：1和4，2和3
// 参数说明     duty：指定对应电机侧占空比，范围：-10000-10000（注：duty本意是没有负数的）
// 备注信息     左侧电机共用PWM1，右侧电机共用PWM2
//-------------------------------------------------------------------------------------------------------------------

void Motor_Set(int num, int duty)
{
    // 占空比溢出保护
    if (duty > 100){duty = 100;}
    else if (duty < -100){duty = -100;}


    // 速度设置为正向
    if (duty >= 0)
    {
        switch (num)
        {
            case 1:
                gpio_set_level(MOTOR1_DIR, 1);                                      // DIR输出高电平
                pwm_set_duty(MOTOR1_PWM, duty * (MOTOR1_PWM_DUTY_MAX / 100));       // 计算占空比
                break;
            case 2:
                gpio_set_level(MOTOR2_DIR, 1);                                      // DIR输出高电平
                pwm_set_duty(MOTOR2_PWM, duty * (MOTOR2_PWM_DUTY_MAX / 100));       // 计算占空比
                break;
            case 3:
                gpio_set_level(MOTOR3_DIR, 1);                                      // DIR输出高电平
                pwm_set_duty(MOTOR2_PWM, duty * (MOTOR2_PWM_DUTY_MAX / 100));       // 计算占空比
                break;
            case 4:
                gpio_set_level(MOTOR4_DIR, 1);                                      // DIR输出高电平
                pwm_set_duty(MOTOR1_PWM, duty * (MOTOR1_PWM_DUTY_MAX / 100));       // 计算占空比
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
                gpio_set_level(MOTOR1_DIR, 0);                                      // DIR输出低电平
                pwm_set_duty(MOTOR1_PWM, -duty * (MOTOR1_PWM_DUTY_MAX / 100));      // 计算占空比
                break;
            case 2:
                gpio_set_level(MOTOR2_DIR, 0);                                      // DIR输出低电平
                pwm_set_duty(MOTOR2_PWM, -duty * (MOTOR2_PWM_DUTY_MAX / 100));      // 计算占空比
                break;
            case 3:
                gpio_set_level(MOTOR3_DIR, 0);                                      // DIR输出低电平
                pwm_set_duty(MOTOR2_PWM, -duty * (MOTOR2_PWM_DUTY_MAX / 100));      // 计算占空比
                break;
            case 4:
                gpio_set_level(MOTOR4_DIR, 0);                                      // DIR输出低电平
                pwm_set_duty(MOTOR1_PWM, -duty * (MOTOR1_PWM_DUTY_MAX / 100));      // 计算占空比
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
