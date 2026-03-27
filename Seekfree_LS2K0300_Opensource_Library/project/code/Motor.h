/********************************************************************************************************************
 * 电机控制
 ********************************************************************************************************************/
#ifndef __MOTOR_H__
#define __MOTOR_H__


// DIR
#define MOTOR1_DIR              "/dev/zf_driver_gpio_motor_1" // P73
#define MOTOR2_DIR              "/dev/zf_driver_gpio_motor_2" // P72
#define MOTOR3_DIR              "/dev/zf_driver_gpio_motor_3" // P88
#define MOTOR4_DIR              "/dev/zf_driver_gpio_motor_4" // P89
// PWM
#define MOTOR1_PWM              "/dev/zf_device_pwm_motor_1"
#define MOTOR2_PWM              "/dev/zf_device_pwm_motor_2"

extern struct pwm_info motor_1_pwm_info;
extern struct pwm_info motor_2_pwm_info;
// 在设备树中，设置的10000。如果要修改，需要与设备树对应。
#define MOTOR1_PWM_DUTY_MAX     (motor_1_pwm_info.duty_max)        
#define MOTOR2_PWM_DUTY_MAX     (motor_2_pwm_info.duty_max)  

void    Motor_Init              (void);
void    Motor_Set               (int num, int speed);
void    Motor_Reset_ALL         (void);


#endif