/********************************************************************************************************************
 * 全功能非阻塞式按键
 * 原代码来自江协科技
 ********************************************************************************************************************/
#include "zf_common_headfile.h"

#include "defines.h"
#include "Key.h"


uint8_t Key_Flag[KEY_COUNT];//不同的位表示不同的事件标志位

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取按键状态
// 使用示例     Key_GetState(KEY_DEFINE_UP);
// 参数说明     n：指定按键
// 返回参数     KEY_PRESSED：按键按下
// 返回参数     KEY_UNPRESSED：按键未按下    
// 备注信息     Key模块内部调用
//-------------------------------------------------------------------------------------------------------------------

int8_t Key_GetState(uint8_t n)
{
	if (n == KEY_NAME_UP)
	{
		if (gpio_get_level(KEY_DEFINE_UP) == 0)//按下
		{
		    return KEY_PRESSED;//返回按下
		}
	}
	else if (n == KEY_NAME_DOWN)
	{
		if (gpio_get_level(KEY_DEFINE_DOWN) == 0)//按下
		{
		    return KEY_PRESSED;
		}
	}
	else if (n == KEY_NAME_CONFIRM)  
	{
		if (gpio_get_level(KEY_DEFINE_CONFIRM) == 0)//按下
		{
		    return KEY_PRESSED;
		}
	}
	else if (n == KEY_NAME_BACK)
	{
		if (gpio_get_level(KEY_DEFINE_BACK) == 0)//按下
		{
		    return KEY_PRESSED;
		}
	}
    return KEY_UNPRESSED;//返回未按下
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查按键标志位是否为1
// 使用示例     Key_Check(KEY_NAME_UP, KEY_SINGLE);
// 参数说明     n：指定按键（KEY_NAME_UP/KEY_NAME_DOWN/KEY_NAME_CONFIRM/KEY_NAME_BACK）
// 参数说明     Flag：指定按键标志位位
// 返回参数     1：为指定标志位
// 返回参数     0：不是
// 备注信息     Key模块内部调用
//-------------------------------------------------------------------------------------------------------------------

uint8_t Key_Check(uint8_t n, uint8_t Flag)
{
	//一个变量&位掩码
	//例如：变量&0x01，如果最低位是1，则结果为0x01
	//				   如果最低位是0，则结果为0x00
	//		变量&0x02，如果次低位是1，则结果为0x02
	//				   如果次低位是0，则结果为0x00
	//注意，指定位为1时，结果是位掩码本身(非0)，而不一定是0x01
	if (Key_Flag[n] & Flag)
	{
		if (Flag != KEY_HOLD)//检测指定位是否为KEY_HOLD
		{
			//清零KEY_HOLD；其他标志位在主程序读后清零
			Key_Flag[n] &= ~Flag;
		}
		return 1;//指定标志位是1
	}
	return 0;//指定标志位是0
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键状态机
// 备注信息     交给定时器定时中断调用，获取时间基准
//-------------------------------------------------------------------------------------------------------------------

void Key_Tick(void)//利用定时中断调用，获取通用的时间基准
{
	static uint8_t Count;//定义静态变量
	static uint8_t i;//用于遍历,按键少时可以这么干
	static uint8_t CurrState[KEY_COUNT],PrevState[KEY_COUNT];//Current,Previous
	static uint8_t S[KEY_COUNT];//状态变量，同江协状态转移图
	static uint16_t Time[KEY_COUNT];//长按/双击 计时器（此处递减计时）
	//静态变量默认值为0，函数退出后值不会丢失
	
	for (i = 0;i < KEY_COUNT; i ++)
	{
		if  (Time[i] > 0)
		{
			Time[i] --;
		}
	}
	Count++;//计数分频
	if (Count >= 2)//可过滤按键抖动，按需调整
	{
		Count = 0 ;
		
		for (i = 0;i < KEY_COUNT; i ++)
		{
			PrevState[i] = CurrState[i];//上次状态
			CurrState[i] = Key_GetState(i);//本次状态
			
			//置标志位
			//Key_Flag |= 0x01;	
			//只把最低位置一，而不影响其他位
			//Key_Flag &= 0xFE;  或者  Key_Flag &= ~0x01;
			//只把最低位清零，而不影响其他位
			//使用宏定义转换0xXX,增加可读性
			
			if (CurrState[i] == KEY_PRESSED)
			{
				Key_Flag[i] |= KEY_HOLD;//HOLD = 1
			}
			else
			{
				Key_Flag[i] &= ~KEY_HOLD;//HOLD = 0
			}
			
			if (CurrState[i] == KEY_PRESSED && PrevState[i] == KEY_UNPRESSED)
			{
				Key_Flag[i] |= KEY_DOWN;//DOWM = 1
				//按下瞬间
				//(中断程序只需置一，由主程序置零)
			}
			
			if (CurrState[i] == KEY_UNPRESSED && PrevState[i] == KEY_PRESSED)
			{
				Key_Flag[i] |= KEY_UP;//UP = 1
				//松开瞬间
			}
			
			if (S[i] == 0)//状态：空闲
			{
				if (CurrState[i] == KEY_PRESSED)//按键按下
				{
					Time[i] = KEY_Time_LONG;//设定长按时间
					S[i] = 1;
				}
			}
			else if (S[i] == 1)//状态：按键已按下
			{
				if (CurrState[i] == KEY_UNPRESSED)
				{
					Time[i] = KEY_Time_DOUBLE;//设定双击时间（到此分支，长按时间已无效）
					S[i] = 2;
				}
				else if (Time[i] == 0)//长按时间到（超时）
				{
					Time[i] = KEY_Time_REPEAT;//设定重复时间
					Key_Flag[i] |= KEY_LONG;//LONG=1
					S[i] = 4;
				}
			}
			else if (S[i] == 2)//状态：按键已松开
			{
				if (CurrState[i] == KEY_PRESSED)
				{
					Key_Flag[i] |= KEY_DOUBLE;//DOUBLE = 1
					S[i] = 3;
				}
				else if (Time[i] == 0)//双击时间到（超时）
				{
					Key_Flag[i] |= KEY_SINGLE;//SINGLE = 1
					S[i] = 0;
				}
			}
			else if (S[i] == 3)//状态：按键已双击
			{
				if (CurrState[i] == KEY_UNPRESSED)
				{
					S[i] = 0;
				}
			}
			else if (S[i] == 4)//状态：按键已长按
			{
				if (CurrState[i] == KEY_UNPRESSED)
				{
					S[i] = 0;
				}
				else if (Time[i] == 0)
				{
					Time[i] = KEY_Time_REPEAT;
					Key_Flag[i] |= KEY_REPEAT;//REPEAT = 1
					S[i] = 4;
				}
			}
		}
	}
}