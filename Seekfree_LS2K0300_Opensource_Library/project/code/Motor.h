/********************************************************************************************************************
 * 电机控制
 ********************************************************************************************************************/
#ifndef __MOTOR_H__
#define __MOTOR_H__

#define DUTY_MAX                10000

// 一级命名
// DIR
#define init_DIR_P72            "/dev/zf_driver_gpio_motor_1" // P72
#define init_DIR_P73            "/dev/zf_driver_gpio_motor_2" // P73
#define init_DIR_P88            "/dev/zf_driver_gpio_motor_3" // P88
#define init_DIR_P89            "/dev/zf_driver_gpio_motor_4" // P89
// PWM
#define init_PWM1_P65           "/dev/zf_device_pwm_motor_1" // P65
#define init_PWM2_P66           "/dev/zf_device_pwm_motor_2" // P66

// 二级映射
// DIR
#define MOTOR_1_DIR             init_DIR_P72
#define MOTOR_2_DIR             init_DIR_P73
#define MOTOR_3_DIR             init_DIR_P88
#define MOTOR_4_DIR             init_DIR_P89
// PWM
#define MOTOR_1_PWM             init_PWM2_P66
#define MOTOR_2_PWM             init_PWM2_P66
#define MOTOR_3_PWM             init_PWM1_P65
#define MOTOR_4_PWM             init_PWM1_P65

extern struct pwm_info motor_1_pwm_info;
extern struct pwm_info motor_2_pwm_info;
// 在设备树中，设置的10000。如果要修改，需要与设备树对应。
#define MOTOR1_PWM_DUTY_MAX     (motor_1_pwm_info.duty_max)        
#define MOTOR2_PWM_DUTY_MAX     (motor_2_pwm_info.duty_max)  

void    Motor_Init              (void);
void    Motor_Set               (int num, int speed);
void    Motor_Reset_ALL         (void);

void    Motor_Move_Straight     (int16_t speed);
void    Motor_Spot_Left         (uint16_t speed);
void    Motor_Spot_Right        (uint16_t speed);

void    Motor_Output_Turn       (void);

#endif