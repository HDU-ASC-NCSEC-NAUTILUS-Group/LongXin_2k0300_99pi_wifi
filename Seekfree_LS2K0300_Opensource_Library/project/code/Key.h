/********************************************************************************************************************
 * 全功能非阻塞式按键
 * 原代码来自江协科技
 ********************************************************************************************************************/
#ifndef __KEY_H__
#define __KEY_H__


// 按键宏定义
#define KEY_DEFINE_UP       "/dev/zf_driver_gpio_key_3"    //P16;S3
#define KEY_DEFINE_DOWN     "/dev/zf_driver_gpio_key_2"    //P15;S4
#define KEY_DEFINE_CONFIRM  "/dev/zf_driver_gpio_key_1"    //P14;S5
#define KEY_DEFINE_BACK     "/dev/zf_driver_gpio_key_0"    //P13;S6

//用宏定义替换1/0，便于理解Key_GetState（）
#define KEY_PRESSED 			1
#define	KEY_UNPRESSED			0

//宏定义替换时间阈值
/*
说明：由于按键检测和状态机代码每隔10ms才会执行一次
所以此处的各个时间阈值最好都设置为10ms的倍数
所以即使设定时间为55ms，但实际却会是60ms
*/
#define KEY_Time_DOUBLE 		0
#define KEY_Time_LONG			200
#define KEY_Time_REPEAT			100

//宏定义调换按键数量
#define KEY_COUNT				4

//用宏定义替换按键索引号
#define KEY_NAME_UP				0	
#define KEY_NAME_DOWN			1	
#define KEY_NAME_CONFIRM		2	
#define KEY_NAME_BACK			3	

//用宏定义替换按键标志位的位掩码，使程序的意义更清晰
#define KEY_HOLD				0x01
#define KEY_DOWN				0x02
#define KEY_UP					0x04
#define KEY_SINGLE				0X08
#define KEY_DOUBLE				0x10
#define KEY_LONG				0x20
#define KEY_REPEAT				0x40

uint8_t Key_Check(uint8_t n, uint8_t Flag);
void Key_Tick(void);


#endif