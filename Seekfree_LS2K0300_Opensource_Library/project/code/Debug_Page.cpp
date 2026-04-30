/********************************************************************************************************************
* Debug模式菜单文件
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "zf_driver_encoder.h"
#include "zf_device_imu963ra.h"
#include "zf_driver_file.h"

#include "defines.h"
#include "image_process.h"
#include "IMU_Analysis.h"
#include "Key.h"
#include "Motor.h"
#include "param_config.h"



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
			ips200_show_string(8  ,0  , "[Debug]");
			ips200_show_string(0  ,16 , "==============================");
			ips200_show_string(10 ,32 , "BUZ");
            ips200_show_string(10 ,48 , "MOTOR");
            ips200_show_string(10 ,64 , "IMU963RA");
            ips200_show_string(10 ,80 , "UVC-QR");
            ips200_show_string(10 ,96 , "UVC-TRACK");
		
			break;
	}
}

// [三级界面]蜂鸣器调试界面
void Debug_BUZ_UI(void)
{
    ips200_show_string(8  ,0  , "[DEBUG]-BUZ");
    ips200_show_string(0  ,16 , "==============================");
    ips200_show_string(10 ,32 , "OFF");
}

// [三级界面]电机调试界面
void Debug_MOTOR_UI(void)
{
    ips200_show_string(8  , 0  , "[DEBUG]-MOTOR");
    ips200_show_string(0  , 16 , "==============================");
    ips200_show_string(10 , 32 , "PWM1:###       ENC1:###");
    ips200_show_string(10 , 48 , "PWM2:###       ENC2:###");
    ips200_show_string(10 , 64 , "PWM3:###       ENC3:###");
    ips200_show_string(10 , 80 , "PWM4:###       ENC4:###");
    ips200_show_string(10 , 96 , "OFFset14:#");
    ips200_show_string(10 , 112, "OFFset23:#");
    // 占位符
    ips200_show_string(10 , 144, "SUM1:###       SUM2:###");
}

// [三级界面]IMU963RA调试界面
void Debug_IMU963RA_UI(void)
{
    ips200_show_string(8  ,0  , "[DEBUG]-IMU963RA");
    ips200_show_string(0  ,16 , "==============================");
    ips200_show_string(0  ,32 , "Cali:[GYRO]IDLE  [MAGN]IDLE");
    ips200_show_string(0  ,48 , "Mode:####");
    // 占位符
    // 占位符
    ips200_show_string(0  ,80 , "ax:#      ay:#      az:#      ");
    ips200_show_string(0  ,96 , "gx:#      gy:#      gz:#      ");
    ips200_show_string(0  ,112, "mx:#      my:#      mz:#      ");
    // 占位符
    // 占位符
    ips200_show_string(0  ,144, "Ro:#      Ya:#      Pi:#      ");

    // 0 关闭
    // 1 三轴
    // 2 [全欧拉角]六轴
    // 3 九轴
    // 4 [仅输出Yaw]Mag_Get_Yaw(仅磁力计+倾斜补偿)
    // 5 [仅输出Yaw]Mahony AHRS(九轴)
    // 6 [仅输出Yaw]Madgwick AHRS(九轴)
    // 7 [仅输出Yaw]TiltMagYaw(重力投影磁修正陀螺积分)
    #if   DEFINE_IMU_ANALYSIS_MODE == 0 
        ips200_show_string(40 ,48 , "OFF");
    #elif DEFINE_IMU_ANALYSIS_MODE == 1
        ips200_show_string(40 ,48 , "3Axis");
    #elif DEFINE_IMU_ANALYSIS_MODE == 2
        ips200_show_string(40 ,48 , "6Axis");
    #elif DEFINE_IMU_ANALYSIS_MODE == 3
        ips200_show_string(40 ,48 , "9Axis");
    #elif DEFINE_IMU_ANALYSIS_MODE == 4
        ips200_show_string(40 ,48 , "Yaw-1");
    #elif DEFINE_IMU_ANALYSIS_MODE == 5
        ips200_show_string(40 ,48 , "Yaw-2");
    #elif DEFINE_IMU_ANALYSIS_MODE == 6
        ips200_show_string(40 ,48 , "Yaw-3");
    #elif DEFINE_IMU_ANALYSIS_MODE == 7
        ips200_show_string(40 ,48 , "Yaw-4");
    #endif
}

// [三级界面]UVC摄像头的二维码识别调试界面
void Debug_UVC_QR_UI(void)
{
    ips200_show_string(8  ,0  , "[DEBUG]-UVC-QR");
    ips200_show_string(0  ,16 , "==============================");
    // 下面的区域将被UVC回传的图像覆盖
}

// [三级界面]UVC摄像头的跟踪调试界面
void Debug_UVC_TRACK_UI(void)
{
    ips200_show_string(8  ,0  , "[DEBUG]-UVC-TRACK");
    ips200_show_string(0  ,16 , "==============================");
    // 下面的区域将被UVC回传的图像覆盖
}
/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 菜单样式 [E]*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 界面逻辑 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 相关函数提前声明
int Debug_BUZ           (void);
int Debug_MOTOR         (void);
int Debug_IMU963RA      (void);
int Debug_UVC_QR        (void);
int Debug_UVC_TRACK     (void);


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
            if (Debug_Page_flag < 1)Debug_Page_flag = 5;
        }
        else if (Key_Check(KEY_NAME_DOWN,KEY_SINGLE))
        {
            key_pressed = 1;
            Debug_Page_flag ++;
            if (Debug_Page_flag > 5)Debug_Page_flag = 1;
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
        else if (Debug_Page_flag_temp == 4)
        {
            ips200_clear();
            Debug_UVC_QR();
            
            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI(1);
            ips200_show_string(0  ,80 , ">");
        }
        else if (Debug_Page_flag_temp == 5)
        {
            ips200_clear();
            Debug_UVC_TRACK();
            
            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI(1);
            ips200_show_string(0  ,96 , ">");
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
                case 4:
                    ips200_clear();
                    Debug_Page_Menu_UI(1);
                    ips200_show_string(0  ,80 , ">");

                    break;
                case 5:
                    ips200_clear();
                    Debug_Page_Menu_UI(1);
                    ips200_show_string(0  ,96 , ">");

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
                ips200_show_string(10 ,32 , "ON ");
            }
            else
            {
                // 关闭蜂鸣器
                gpio_set_level(BEEP_DEFINE, 0x0);
                ips200_show_string(10 ,32 , "OFF");
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

    // 出于特殊的设置，此处独立了速度的方向和大小的存储
    // 电机14，23各为一对
    // 建议认定速度为0时方向为1
    uint16_t DUTY[4] = {0,0,0,0};

    // 电机偏移量
    // 实际上，仍然电机14，23各为一对，只是方便写代码
    int8_t OFFSET[4] = {0,0,0,0};

    // 电机方向
    // 4个电机是独立方向的
    int8_t DIR[4] = {1,1,1,1};

    ips200_Printf(50 ,32 , "%d   ", (DUTY[0] + OFFSET[0]) * DIR[0]);
    ips200_Printf(50 ,48 , "%d   ", (DUTY[1] + OFFSET[1]) * DIR[1]);
    ips200_Printf(50 ,64 , "%d   ", (DUTY[2] + OFFSET[2]) * DIR[2]);
    ips200_Printf(50 ,80 , "%d   ", (DUTY[3] + OFFSET[3]) * DIR[3]);
    ips200_Printf(82 ,96 , "%d   ",OFFSET[0]); 
    ips200_Printf(82 ,112, "%d   ",OFFSET[1]); 

    // 电机位号对应示意图
    // #1 [][][] 3#
    // #1 [][][] 3#
    //    [][][]
    //    [][][]
    // #2 [][][] 4#
    // #2 [][][] 4#

    // PWM共用：1和4，2和3

    int16_t speed1 = 0;
    int16_t speed2 = 0;
    int16_t sum1 = 0;
    int16_t sum2 = 0;

    //  清零编码器数据
    speed1 = (int16)encoder_quad_get_count(ENCODER_QUAD_1);
    speed2 = (int16)encoder_quad_get_count(ENCODER_QUAD_2);

    // 确保重置
    Motor_Reset_ALL();

    while(1)
    {
        // 上/下按键是否被按下过
        uint8_t key_pressed = 0;

        /*======================================================*/
		/*[按键处理]**********************************************/
		/*======================================================*/
        // 选择模式（无选中目标）
        if (Debug_MOTOR_flag_temp == 0)
        {                   
            if (Key_Check(KEY_NAME_UP,KEY_SINGLE)) 
            {
                Debug_MOTOR_flag --;
                if (Debug_MOTOR_flag < 1)Debug_MOTOR_flag = 6;
                key_pressed = 1;
            }
            else if (Key_Check(KEY_NAME_DOWN,KEY_SINGLE)) 
            {
                Debug_MOTOR_flag ++;
                if (Debug_MOTOR_flag > 6)Debug_MOTOR_flag = 1;
                key_pressed = 1;
            }
            else if (Key_Check(KEY_NAME_CONFIRM,KEY_SINGLE))
            {
                Debug_MOTOR_flag_temp = Debug_MOTOR_flag;
            }
            else if (Key_Check(KEY_NAME_BACK,KEY_SINGLE))
            {
                // 返回上一级界面
                Motor_Reset_ALL();
                return 0;   
            }

            // 光标更新
            if (Debug_MOTOR_flag_temp != 0)
            {
                ips200_show_string(0  ,16 + Debug_MOTOR_flag * 16, "=");
            }
        }

        // 更改模式（有选中目标）
        else if (Debug_MOTOR_flag_temp != 0)
        {   
            if (Key_Check(KEY_NAME_UP,KEY_SINGLE)) 
            {
                key_pressed = 2;

                if ( 1 <= Debug_MOTOR_flag_temp && Debug_MOTOR_flag_temp <= 4)
                {
                    // 中间计算变量temp
                    int16_t temp = DIR[Debug_MOTOR_flag_temp - 1] * DUTY[Debug_MOTOR_flag_temp - 1];
                    temp += 5;
                    // 边界处理
                    if (temp > 100)
                    {
                        temp = 100;
                    }
                    else if (temp < -100)
                    {
                        temp = -100;
                    }

                    // 速度方向和大小数据更新
                    // 建议认定速度为0时方向为1
                    if (temp >= 0)
                    {
                        DIR[Debug_MOTOR_flag_temp - 1] = 1;
                    }
                    else
                    {
                        DIR[Debug_MOTOR_flag_temp - 1] = -1;
                    }
                    DUTY[Debug_MOTOR_flag_temp - 1] = temp * DIR[Debug_MOTOR_flag_temp - 1];
                }
                else
                {
                    if (Debug_MOTOR_flag_temp == 5)
                    {
                        OFFSET[0] ++;
                        OFFSET[3] ++;
                        ips200_Printf(82 ,96 , "%d   ",OFFSET[0]); 
                        Motor_Set(1, (DUTY[0] + OFFSET[0]) * DIR[0]);
                    }
                    else if (Debug_MOTOR_flag_temp == 6)
                    {
                        OFFSET[1] ++;
                        OFFSET[2] ++;
                        ips200_Printf(82 ,112 , "%d   ",OFFSET[1]); 
                        Motor_Set(2, (DUTY[1] + OFFSET[1]) * DIR[1]);
                    }
                }

                Motor_Set(Debug_MOTOR_flag_temp, (DUTY[Debug_MOTOR_flag_temp - 1] + OFFSET[Debug_MOTOR_flag_temp - 1]) * DIR[Debug_MOTOR_flag_temp - 1]);                         
            }
            else if (Key_Check(KEY_NAME_DOWN,KEY_SINGLE)) 
            {
                key_pressed = 2;

                if ( 1 <= Debug_MOTOR_flag_temp && Debug_MOTOR_flag_temp <= 4)
                {
                    // 中间计算变量temp
                    int16_t temp = DIR[Debug_MOTOR_flag_temp - 1] * DUTY[Debug_MOTOR_flag_temp - 1];
                    temp -= 5;
                    // 边界处理
                    if (temp > 100)
                    {
                        temp = 100;
                    }
                    else if (temp < -100)
                    {
                        temp = -100;
                    }

                    // 速度方向和大小数据更新
                    // 建议认定速度为0时方向为1
                    if (temp >= 0)
                    {
                        DIR[Debug_MOTOR_flag_temp - 1] = 1;
                    }
                    else
                    {
                        DIR[Debug_MOTOR_flag_temp - 1] = -1;
                    }
                    DUTY[Debug_MOTOR_flag_temp - 1] = temp * DIR[Debug_MOTOR_flag_temp - 1];
                    Motor_Set(Debug_MOTOR_flag_temp, (DUTY[Debug_MOTOR_flag_temp - 1] + OFFSET[Debug_MOTOR_flag_temp - 1]) * DIR[Debug_MOTOR_flag_temp - 1]);
                }
                else
                {
                    if (Debug_MOTOR_flag_temp == 5)
                    {
                        OFFSET[0] --;
                        OFFSET[3] --;
                        ips200_Printf(82 ,96 , "%d   ",OFFSET[0]); 
                        Motor_Set(1, (DUTY[0] + OFFSET[0]) * DIR[0]);
                    }
                    else if (Debug_MOTOR_flag_temp == 6)
                    {
                        OFFSET[1] --;
                        OFFSET[2] --;
                        ips200_Printf(82 ,112 , "%d   ",OFFSET[1]); 
                        Motor_Set(2, (DUTY[1] + OFFSET[1]) * DIR[1]);
                    }
                }
                
                                         
            }
            else if ((Key_Check(KEY_NAME_CONFIRM,KEY_SINGLE)) || (Key_Check(KEY_NAME_BACK,KEY_SINGLE)))
            {
                // 光标更新
                ips200_show_string(0  ,16 + Debug_MOTOR_flag * 16, ">");
                Debug_MOTOR_flag_temp = 0;
            }
        }
        /*======================================================*/
		/**********************************************[按键处理]*/
		/*======================================================*/


        /* 编码器数据更新*/
        if (Time_200ms_Flag)
        {
            Time_200ms_Flag = 0;
            speed1 = (int16)encoder_quad_get_count(ENCODER_QUAD_1);
            speed2 = (int16)encoder_quad_get_count(ENCODER_QUAD_2);
            sum1 += speed1;
            sum2 += speed2;

            ips200_Printf(170,48 , "%d  ", speed1); 
            ips200_Printf(170,80 , "%d  ", speed2);      
            ips200_Printf(170,144, "%d  ", sum1);
            ips200_Printf(50 ,144, "%d  ", sum2);
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
                default:break;
            }
            ips200_Printf(50 ,32 , "%d   ", (DUTY[0] + OFFSET[0]) * DIR[0]); 
            ips200_Printf(50 ,48 , "%d   ", (DUTY[1] + OFFSET[1]) * DIR[1]); 
            ips200_Printf(50 ,64 , "%d   ", (DUTY[2] + OFFSET[2]) * DIR[2]); 
            ips200_Printf(50 ,80 , "%d   ", (DUTY[3] + OFFSET[3]) * DIR[3]);
        }


        /* 显示更新*/
        if (key_pressed == 1)
        {
            // 清除光标，暂时用这个方法
            ips200_show_string(0  ,32 , " ");
            ips200_show_string(0  ,48 , " ");
            ips200_show_string(0  ,64 , " ");
            ips200_show_string(0  ,80 , " ");
            ips200_show_string(0  ,96 , " ");
            ips200_show_string(0  ,112, " ");
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
    
    // 干脆总是存在一个没有校准陀螺仪就校准陀螺仪的设定

    /* 半阻塞式IMU963RA零飘此时请保持静此时请保持静止)*/
	if (IMU_Gyro_Calib_Check(&gyro_cal) != GYRO_CALIB_STATE_DONE)// 如果未校准
	{
        ips200_show_string(40 ,32 , "#GYRO#ING~");
	    IMU_Gyro_Calib_Start(&gyro_cal);
	}
	// 半阻塞式零飘校准
	while(1)
    {
        if (IMU_Gyro_Calib_Check(&gyro_cal) == GYRO_CALIB_STATE_DONE)  // 零飘校准完成
        {
            ips200_show_string(40 ,32 , "#GYRO#DONE");
            break;  // 结束零飘校准
        }             
        if (Key_Check(KEY_NAME_BACK,KEY_SINGLE)) // 强制零飘校准退出
        {
            ips200_show_string(40 ,32 , "#GYRO#STOP");
            break;  // 中止零飘校准
        }        
    }

    // 仅仅作为显示变量，不用于计算
    float ax = 0,ay = 0,az = 0;
    float gx = 0,gy = 0,gz = 0;
    int16_t mx = 0,my = 0,mz = 0;

    // 区分选择校准陀螺仪还是磁力计
    // 0：校准陀螺仪
    // 1：校准磁力计
    uint8_t Debug_IMU963RA_flag = 0;

    while(1)
    {
        /* 按键处理*/      
        if (Key_Check(KEY_NAME_UP,KEY_SINGLE)) 
        {
            // 选中对为陀螺仪校准
            Debug_IMU963RA_flag = 0;
            ips200_show_string(40 ,32 , "#GYRO#");
            ips200_show_string(136,32 , "[MAGN]");
        }
        else if (Key_Check(KEY_NAME_DOWN,KEY_SINGLE)) 
        {
            // 选中对为磁力计校准
            Debug_IMU963RA_flag = 1;
            ips200_show_string(40 ,32 , "[GYRO]");
            ips200_show_string(136,32 , "#MAGN#");
        }
        else if (Key_Check(KEY_NAME_CONFIRM,KEY_SINGLE))
        {
            // 开启校准
            if (Debug_IMU963RA_flag == 0) // 校准陀螺仪
            {
                IMU_Gyro_Calib_Start(&gyro_cal);
                ips200_show_string(40 ,32 , "#GYRO#ING~");
                // 半阻塞式零飘校准
                while(1)
                {
                    if (IMU_Gyro_Calib_Check(&gyro_cal) == GYRO_CALIB_STATE_DONE)  // 零飘校准完成
                    {
                        ips200_show_string(40 ,32 , "#GYRO#DONE");
                        break;  // 结束零飘校准
                    }                                                             
                    if (Key_Check(KEY_NAME_BACK,KEY_SINGLE)) // 强制零飘校准退出
                    {
                        ips200_show_string(40 ,32 , "#GYRO#STOP");
                        break;  // 中止零飘校准
                    }        
                }
                IMU_Reset_Data();
                
            }
            else // 校准磁力计
            {                   
                IMU_Mag_Calib_Start(&mag_cal);
                ips200_show_string(136 ,32 , "#MAGN#ING~");
                // 半阻塞式零飘校准
                while(1)
                {
                    if (IMU_Mag_Calib_Check(&mag_cal) == MAG_CALIB_STATE_DONE)  // 零飘校准完成
                    {
                        ips200_show_string(136,32 , "#MAGN#DONE"); 
                        break;  // 结束零飘校准
                    }                                   
                    if (Key_Check(KEY_NAME_BACK,KEY_SINGLE)) // 强制零飘校准退出
                    {
                        ips200_show_string(136,32 , "#MAGN#STOP"); 
                        break;  // 中止零飘校准
                    }
                }
                IMU_Reset_Data();
                     
            }           
        }
        else if (Key_Check(KEY_NAME_BACK,KEY_SINGLE))
        {
            // 返回上一级界面
            return 0;   
        }


        /* IMU数据更新*/
        if (IMU_D_and_A_Enable)
        {
            IMU_Update_Data();
            IMU_Update_Analysis();
            IMU_D_and_A_Enable = 0;
        }


        /* 显示更新*/
        if (Time_200ms_Flag)
        {   
                IMU_Acc_Apply(&ax, &ay, &az);            
                IMU_Gyro_Apply(&gyro_cal, &gx, &gy, &gz);
                IMU_Mag_Apply(&mag_cal, &mx, &my, &mz);
            
                ips200_Printf(24 ,80 , "%.0f ", ax);
                ips200_Printf(104,80 , "%.0f ", ay);
                ips200_Printf(184,80 , "%.0f ", az);
                ips200_Printf(24 ,96 , "%.1f ", gx);
                ips200_Printf(104,96 , "%.1f ", gy);
                ips200_Printf(184,96 , "%.1f ", gz);
                ips200_Printf(24 ,112, "%d ", mx);
                ips200_Printf(104,112, "%d ", my);
                ips200_Printf(184,112, "%d ", mz);

            Time_200ms_Flag = 0;
        }
        ips200_Printf(24 ,144, "%.1f ", Roll_Result);
        ips200_Printf(104,144, "%.1f ", Yaw_Result);
        ips200_Printf(184,144, "%.1f ", Pitch_Result);
    }
}


// #   #  #   #   ####         ###   ####   
// #   #  #   #  #            #   #  #   #  
// #   #  #   #  #      ###   #   #  ####   
// #   #   # #   #            #  ##  #  #   
//  ###     #     ####         ####  #   #  
//
// [三级界面]二维码调试
int Debug_UVC_QR(void)
{
    Debug_UVC_QR_UI();

    while(1)
    {
        if (Key_Check(KEY_NAME_BACK,KEY_SINGLE))
        {
            // 恢复默认颜色
            ips200_set_pen_color(RGB565_RED);
            // 返回上一级界面
            return 0;   
        }
        
        QR_process();
    }
}

// #   #  #   #   ####         #####  ####    ###    ####  #   #  
// #   #  #   #  #               #    #   #  #   #  #      #  #   
// #   #  #   #  #       ###     #    ####   #####  #      ###    
// #   #   # #   #               #    #  #   #   #  #      #  #   
//  ###     #     ####           #    #   #  #   #   ####  #   #  
//
// [三级界面]物块跟踪调试
int Debug_UVC_TRACK(void)
{
    Debug_UVC_TRACK_UI();

    while(1)
    {
        if (Key_Check(KEY_NAME_BACK,KEY_SINGLE))
        {
            // 恢复默认颜色
            ips200_set_pen_color(RGB565_RED);
            // 返回上一级界面
            return 0;   
        }
        object_tracking();  // 红色物块跟踪显示
//        coordinate_transformation();  // 坐标转换显示
    }
}
/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 界面逻辑 [E]*/
/*******************************************************************************************************************/
