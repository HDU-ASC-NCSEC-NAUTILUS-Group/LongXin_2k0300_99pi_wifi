/********************************************************************************************************************
* Debug模式菜单文件
********************************************************************************************************************/
#include "zf_common_headfile.h"

#include "defines.h"
#include "IMU_Analysis.h"
#include "Key.h"
#include "Motor.h"



/*******************************************************************************************************************/
/*[S] 菜单样式 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// [二级界面]Debug模式界面
void Debug_Page_Menu_UI(uint8_t Page)
{
	switch(Page)
	{
		// 第一页
		case 1:
			ips200_show_string(8  , 0  , "[Debug]");
			ips200_show_string(0  , 16 , "==============================");
			ips200_show_string(10 , 32 , "BUZ");
            ips200_show_string(10 , 48 , "MOTOR");
            ips200_show_string(10 , 64 , "IMU963RA");
		
			break;
	}
}

// [三级界面]蜂鸣器调试界面
void Debug_BUZ_UI(void)
{
    ips200_show_string(8  , 0  , "[DEBUG]-BUZ");
    ips200_show_string(0  , 16 , "==============================");
    ips200_show_string(10 , 32 , "OFF");
}

// [三级界面]电机调试界面
void Debug_MOTOR_UI(void)
{
    ips200_show_string(8  , 0  , "[DEBUG]-MOTOR");
    ips200_show_string(0  , 16 , "==============================");
    ips200_show_string(10 , 32 , "PWM1:#####      ENC1:###");
    ips200_show_string(10 , 48 , "PWM2:#####      ENC2:###");
    ips200_show_string(10 , 64 , "PWM3:#####      ENC3:###");
    ips200_show_string(10 , 80 , "PWM4:#####      ENC4:###");
}

// [三级界面]IMU963RA调试界面
void Debug_IMU963RA_UI(void)
{
    ips200_show_string(8  , 0  , "[DEBUG]-IMU963RA");
    ips200_show_string(0  , 16 , "==============================");
    ips200_show_string(0  , 32 , "Cali:####");
    ips200_show_string(0  , 48 , "ax:       ay:       az:       ");
    ips200_show_string(0  , 64 , "gx:       gy:       gz:       ");
    ips200_show_string(0  , 80 , "mx:       my:       mz:       ");
    ips200_show_string(0  , 96 , "Ro:       Ya:       Pi:       ");
}
/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 菜单样式 [E]*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 交互界面 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 相关函数提前声明
int Debug_BUZ           (void);
int Debug_MOTOR         (void);
int Debug_IMU963RA      (void);

// [二级界面]Debug模式界面
int Debug_Page_Menu(void)
{
    // Debug模式选项 标志位
    uint8_t Debug_Page_flag = 1;

    Debug_Page_Menu_UI(1);
    ips200_show_string(0  ,32 , ">");

    while(1)
    {
        // 存储确认键被按下时Debug_Page_flag的值的临时变量，默认为无效值0
		uint8_t Debug_Page_flag_temp = 0;
		// 上/下按键是否被按下过
		uint8_t key_pressed = 0;


        /* 按键处理*/
        if (Key_Check(KEY_NAME_UP,KEY_SINGLE))
        {
            key_pressed = 1;
            Debug_Page_flag --;
            if (Debug_Page_flag < 1)Debug_Page_flag = 3;
        }
        else if (Key_Check(KEY_NAME_DOWN,KEY_SINGLE))
        {
            key_pressed = 1;
            Debug_Page_flag ++;
            if (Debug_Page_flag > 3)Debug_Page_flag = 1;
        }
        else if (Key_Check(KEY_NAME_CONFIRM,KEY_SINGLE))
        {
            Debug_Page_flag_temp = Debug_Page_flag;
        }
        else if (Key_Check(KEY_NAME_BACK,KEY_SINGLE))    
        {
            // 返回上一级界面
            return 0;   
        }


        /* 模式跳转*/
        if (Debug_Page_flag_temp == 1)
        {
            ips200_clear();
            Debug_BUZ();
            
            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI(1);
            ips200_show_string(0  ,32 , ">");
        }
        else if (Debug_Page_flag_temp == 2)
        {
            ips200_clear();
            Debug_MOTOR();
            
            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI(1);
            ips200_show_string(0  ,48 , ">");
        }
        else if (Debug_Page_flag_temp == 3)
        {
            ips200_clear();
            Debug_IMU963RA();
            
            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI(1);
            ips200_show_string(0  ,64 , ">");
        }
        

        /* 显示更新*/
        if (key_pressed)
        {
            switch (Debug_Page_flag)
            {
                case 1:
                    ips200_clear();
                    Debug_Page_Menu_UI(1);
                    ips200_show_string(0  ,32 , ">");

                    break;
                case 2:
                    ips200_clear();
                    Debug_Page_Menu_UI(1);
                    ips200_show_string(0  ,48 , ">");

                    break;
                case 3:
                    ips200_clear();
                    Debug_Page_Menu_UI(1);
                    ips200_show_string(0  ,64 , ">");

                    break;
            }
        }
    }
}

//	####   #   #  #####
//	#   #  #   #     #  
//	####   #   #    #  
//	#   #  #   #   #   
//	####    ###   ##### 
//
// [三级界面]蜂鸣器调试
int Debug_BUZ(void)
{
    Debug_BUZ_UI();
    uint8_t BUZ_flag = 0;

    while(1)
    {
        /* 按键处理*/   
        if (Key_Check(KEY_NAME_CONFIRM,KEY_SINGLE)) 
        {
            BUZ_flag = 1 - BUZ_flag;
            if (BUZ_flag)
            {
                // 开启蜂鸣器
                gpio_set_level(BEEP_DEFINE, 0x1);
                ips200_show_string(10 , 32 , "ON ");
            }
            else
            {
                // 关闭蜂鸣器
                gpio_set_level(BEEP_DEFINE, 0x0);
                ips200_show_string(10 , 32 , "OFF");
            }
        }
        else if (Key_Check(KEY_NAME_BACK,KEY_SINGLE))
        {
            // 返回上一级界面
            return 0;   
        }
    }
}

// #   #   ###   #####   ###   #####  ####   
// ## ##  #   #    #    #   #    #    #   #  
// # # #  #   #    #    #   #    #    ####   
// #   #  #   #    #    #   #    #    #  #   
// #   #   ###     #     ###     #    #   #  
//
// [三级界面]电机调试
int Debug_MOTOR(void)
{  
    // 电机调试 标志位
    uint8_t Debug_MOTOR_flag = 1;
    // 存储确认键被按下时Debug_MOTOR_flag的值的临时变量，默认为无效值0
    uint8_t Debug_MOTOR_flag_temp = 0;

    Debug_MOTOR_UI();
    ips200_show_string(0  ,32 , ">");

    int16_t DUTY[4] = {0,0,0,0};

    ips200_Printf(50 ,32 , "%d    ", DUTY[0]); 
    ips200_Printf(50 ,48 , "%d    ", DUTY[1]); 
    ips200_Printf(50 ,64 , "%d    ", DUTY[2]); 
    ips200_Printf(50 ,80 , "%d    ", DUTY[3]); 
    // 电机位号对应示意图
    // #1 [][][] 3#
    // #1 [][][] 3#
    //    [][][]
    //    [][][]
    // #2 [][][] 4#
    // #2 [][][] 4#

    // PWM共用：1和4，2和3

    while(1)
    {
        // 上/下按键是否被按下过
        uint8_t key_pressed = 0;

        // 选择模式（无选中目标）
        if (Debug_MOTOR_flag_temp == 0)
        {              
            /* 按键处理*/       
            if (Key_Check(KEY_NAME_UP,KEY_SINGLE)) 
            {
                Debug_MOTOR_flag --;
                if (Debug_MOTOR_flag < 1)Debug_MOTOR_flag = 4;
                key_pressed = 1;
            }
            else if (Key_Check(KEY_NAME_DOWN,KEY_SINGLE)) 
            {
                Debug_MOTOR_flag ++;
                if (Debug_MOTOR_flag > 4)Debug_MOTOR_flag = 1;
                key_pressed = 1;
            }
            else if (Key_Check(KEY_NAME_CONFIRM,KEY_SINGLE))
            {
                Debug_MOTOR_flag_temp = Debug_MOTOR_flag;
            }
            else if (Key_Check(KEY_NAME_BACK,KEY_SINGLE))
            {
                // 返回上一级界面
                return 0;   
            }


            /* 光标变化*/
            if (Debug_MOTOR_flag_temp != 0)
            {
                ips200_show_string(0  ,16 + Debug_MOTOR_flag * 16, "=");
            }
        }
        // 更改模式（有选中目标）
        else if (Debug_MOTOR_flag_temp != 0)
        {
            /* 按键处理*/       
            if (Key_Check(KEY_NAME_UP,KEY_SINGLE)) 
            {
                key_pressed = 2;
                DUTY[Debug_MOTOR_flag_temp - 1] += 5;
                if (DUTY[Debug_MOTOR_flag_temp - 1] >  100)
                {
                    DUTY[Debug_MOTOR_flag_temp - 1] =  100;
                }
                Motor_Set(Debug_MOTOR_flag_temp, DUTY[Debug_MOTOR_flag_temp - 1]);                         
            }
            else if (Key_Check(KEY_NAME_DOWN,KEY_SINGLE)) 
            {
                key_pressed = 2;
                DUTY[Debug_MOTOR_flag_temp - 1] -= 5;
                if (DUTY[Debug_MOTOR_flag_temp - 1] < -100)
                {
                    DUTY[Debug_MOTOR_flag_temp - 1] = -100;
                }
                Motor_Set(Debug_MOTOR_flag_temp, DUTY[Debug_MOTOR_flag_temp - 1]);
            }
            else if ((Key_Check(KEY_NAME_CONFIRM,KEY_SINGLE)) || (Key_Check(KEY_NAME_BACK,KEY_SINGLE)))
            {
                ips200_show_string(0  ,16 + Debug_MOTOR_flag * 16, ">");
                Debug_MOTOR_flag_temp = 0;
            }
        }


        /* PWM共用的显示处理*/
        if (key_pressed == 2)
        {
            switch (Debug_MOTOR_flag_temp)
            {
                case 1:DUTY[3] = DUTY[0];break;
                case 2:DUTY[2] = DUTY[1];break;
                case 3:DUTY[1] = DUTY[2];break;
                case 4:DUTY[0] = DUTY[3];break;
            }
            ips200_Printf(50 ,32 , "%d    ", DUTY[0]); 
            ips200_Printf(50 ,48 , "%d    ", DUTY[1]); 
            ips200_Printf(50 ,64 , "%d    ", DUTY[2]); 
            ips200_Printf(50 ,80 , "%d    ", DUTY[3]); 
        }


        /* 显示更新*/
        if (key_pressed == 1)
        {
            // 清除光标，暂时用这个方法
            ips200_show_string(0  ,32 , " ");
            ips200_show_string(0  ,48 , " ");
            ips200_show_string(0  ,64 , " ");
            ips200_show_string(0  ,80 , " ");
            ips200_show_string(0  ,16 + Debug_MOTOR_flag * 16, ">");
        }
    }
}

// #####  #   #  #   #  #####  #####  #####  ####    ###   
//   #    ## ##  #   #  #   #  #          #  #   #  #   #  
//   #    # # #  #   #  #####  #####  #####  ####   #####  
//   #    #   #  #   #      #  #   #      #  #  #   #   #  
// #####  #   #   ###   #####  #####  #####  #   #  #   #  
//
// [三级界面]陀螺仪IMU963RA调试
int Debug_IMU963RA(void)
{
    Debug_IMU963RA_UI();
    
    /* 半阻塞式IMU963RA零飘此时请保持静此时请保持静止)*/
	if (IMU963RA_Calibration_Check() != 2)// 如果未校准
	{
        ips200_show_string(40 , 32 , "ING~");
	    IMU963RA_Calibration_Start();
	}
	// 半阻塞式零飘校准
	while(1)
    {
        if (IMU963RA_Calibration_Check() == 2)  // 零飘校准完成
        {
            break;  // 结束零飘校准
        }      
        // 可以考虑在这里操作OLED，但请注意OLED对时间的占用
        
        // 强制零飘校准退出
        if (Key_Check(KEY_NAME_BACK,KEY_SINGLE)) 
        {
            break;  // 中止零飘校准
        }        
    }
    ips200_show_string(40 , 32 , "DONE");

    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
    float mx = 0.0f;
    float my = 0.0f;
    float mz = 0.0f;

    while(1)
    {
        /* 按键处理*/       
//            if (Key_Check(KEY_NAME_UP,KEY_SINGLE)) 
//            {
//            }
//            else if (Key_Check(KEY_NAME_DOWN,KEY_SINGLE)) 
//            {
//            }
//            else 
            if (Key_Check(KEY_NAME_CONFIRM,KEY_SINGLE))
            {
                // 再次开启校准
                IMU963RA_Calibration_Start();
                ips200_show_string(40 , 32 , "ING~");
                // 半阻塞式零飘校准
                while(1)
                {
                    if (IMU963RA_Calibration_Check() == 2)  // 零飘校准完成
                    {
                        break;  // 结束零飘校准
                    }      
                    // 可以考虑在这里操作OLED，但请注意OLED对时间的占用
                    
                    // 强制零飘校准退出
                    if (Key_Check(KEY_NAME_BACK,KEY_SINGLE)) 
                    {
                        break;  // 中止零飘校准
                    }        
                }
                ips200_show_string(40 , 32 , "DONE");
            }
            else if (Key_Check(KEY_NAME_BACK,KEY_SINGLE))
            {
                // 返回上一级界面
                return 0;   
            }

            if (IMU963RA_analysis_enable)
            {
                imu963ra_get_data();
                IMU963RA_Analysis_Update();
                IMU963RA_analysis_enable = 0;
//                IMU963RA_Apply_Calibration(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
            }

//            ips200_Printf(24 ,48 , "%.1f  ", ax);
//            ips200_Printf(104,48 , "%.1f  ", ay);
//            ips200_Printf(184,48 , "%.1f  ", az);
//            ips200_Printf(24 ,64 , "%.1f  ", gx);
//            ips200_Printf(104,64 , "%.1f  ", gy);
//            ips200_Printf(184,64 , "%.1f  ", gz);
//            ips200_Printf(24 ,80 , "%.1f  ", mx);
//            ips200_Printf(104,80 , "%.1f  ", my);
//            ips200_Printf(184,80 , "%.1f  ", mz);
            ips200_Printf(24 ,96 , "%.1f  ", Roll_Result);
            ips200_Printf(104,96 , "%.1f  ", Yaw_Result);
            ips200_Printf(184,96 , "%.1f  ", Pitch_Result);
        
    }
}
/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 交互界面 [E]*/
/*******************************************************************************************************************/
