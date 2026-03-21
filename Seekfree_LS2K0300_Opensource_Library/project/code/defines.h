/********************************************************************************************************************
* 通用宏定义文件
********************************************************************************************************************/

#ifndef __DEFINES_H__
#define __DEFINES_H__


// 按键宏定义
#define KEY_DEFINE_UP       "/dev/zf_driver_gpio_key_3"    //P16;S3
#define KEY_DEFINE_DOWN     "/dev/zf_driver_gpio_key_2"    //P15;S4
#define KEY_DEFINE_CONFIRM  "/dev/zf_driver_gpio_key_1"    //P14;S5
#define KEY_DEFINE_BACK     "/dev/zf_driver_gpio_key_0"    //P13;S6
// 拨码开关宏定义
#define SWITCH_DEFINE_0     "/dev/zf_driver_gpio_switch_0" //P20;S2
#define SWITCH_DEFINE_1     "/dev/zf_driver_gpio_switch_1" //P21;S2

// 蜂鸣器宏定义
#define BEEP_DEFINE         "/dev/zf_driver_gpio_beep"  

// 电机相关定义
#define MOTOR1_DEFINE_DIR   "/dev/zf_driver_gpio_motor_1"
#define MOTOR2_DEFINE_DIR   "/dev/zf_driver_gpio_motor_2"
#define MOTOR1_DEFINE_PWM   "/dev/zf_device_pwm_motor_1"
#define MOTOR2_DEFINE_PWM   "/dev/zf_device_pwm_motor_2"


#endif