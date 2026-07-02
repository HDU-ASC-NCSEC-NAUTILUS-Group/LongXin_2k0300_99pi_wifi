/*********************************************************************************************************************
* LS2K0300 Opensourec Library 即（LS2K0300 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是LS2K0300 开源库的一部分
*
* LS2K0300 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K0300
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-02-27        大W            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"

/*******************************************************************************************************************/
/*[S] 定时中断 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

timer_fd *pit_timer_10ms;
timer_fd *pit_timer_200ms;

uint8_t number_task = 0;

void pit_callback_10ms()
{
    Key_Tick();
    IMU_D_and_A_Enable = 1;

    /* IMU数据更新*/
    // if (IMU_D_and_A_Enable)
    // {
    //     IMU_Update_Data();
    //     IMU_Update_Analysis();
    //     IMU_D_and_A_Enable = 0;
    // }

    // 雷达数据更新
    // ld_usart_task();            // 快速拉取所有可用字节并解包
    // 雷达数据处理
    // data_process();    // 处理数据，提取前方 ±50° 范围内的点

    // 雷达数据解包信息
    // if (g_lidar_frame_valid) {
    //     // 打印一帧关键信息
    //     printf("Radar: speed=%u rpm, angle %.2f->%.2f, ts=%u\n",
    //            g_lidar_frame.speed,
    //            g_lidar_frame.start_angle * 0.01f,
    //            g_lidar_frame.end_angle * 0.01f,
    //            g_lidar_frame.timestamp);

    //     for (int i = 0; i < POINT_PER_PACK; i++) {
    //         printf("  pt[%d]: %u mm, %u\n",
    //                i,
    //                g_lidar_frame.point[i].distance,
    //                g_lidar_frame.point[i].intensity);
    //     }

    //     // g_lidar_frame_valid = false;
    // }

    // 转向状态更新
    // if(Is_Angle_Turning()) {
	// 	Update_Angle_Turn();
	// }

    //Motor_Output_Turn();

    // uwb数据更新
    uwb_usart_task();    // 快速拉取所有可用 UWB 字节并解包

    // 避障模式
    // number_task++;
    // if (number_task >= 2) {
    //         number_task = 0;
    //         avoid();            // 避障函数，计算避障角度并设置电机速度
    // }

    // uwb跟随模式
    // number_task++;
    // if (number_task >= 2) {
    //         number_task = 0;
    //         uwb_follow();            // 跟随函数，读取uwb角度和距离数据并设置电机速度
    // } 

    // 混合导航模式（避障优先）
    // number_task++;
    // if (number_task >= 2) {
    //         number_task = 0;
    //         navigate_process();            // 混合导航函数，结合避障和跟随功能
    // }
}

void pit_callback_200ms()
{
    Time_200ms_Flag = 1;
}

/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 定时中断 [E]*/
/*******************************************************************************************************************/
/*******************************************************************************************************************/
/*[S] 程序退出 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 清场函数（保护性措施）
void cleanup()
{
    // 需要先停止定时器线程，后面才能稳定关闭电机，电调，舵机等
    pit_timer_10ms->stop();
    pit_timer_200ms->stop();

    printf("程序退出，执行清理操作\n");
    uwb_usart_close();
    Motor_Reset_ALL();
}
// 宣告程序退出函数
void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}
/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 程序退出 [E]*/
/*******************************************************************************************************************/

int main(int, char**) 
{
    // 注册清理函数
    // 需要定义，作为退出整个程序时的重置函数（比如关闭电机、蜂鸣器等）
    atexit(cleanup);
    // 注册SIGINT信号的处理函数
    signal(SIGINT, sigint_handler);

    // 外设初始化
    Peripheral_Init();

    // 创建10ms定时器
    pit_timer_10ms = new timer_fd(10, pit_callback_10ms);
    pit_timer_10ms->start();  
    // 创建200ms定时器
    pit_timer_200ms = new timer_fd(200, pit_callback_200ms);
    pit_timer_200ms->start();
    
    // 定向转向标准位
    // static int8_t turn_flag = 0;
    // static int8_t last_turn_flag = 0;

    while(1)
    {
        // 上下板通信：接收上板指令控制电机
        transport();
        

        // uwb_follow();         // UWB 跟随函数，根据距离和方位角调整电机速度实现跟随


        // if(Key_Check(KEY_NAME_UP,KEY_DOWN))
        // {
        //     IMU_Reset_Data();
        //     turn_flag ++;
        //     if(turn_flag > 2)
        //     {
        //         turn_flag = 0;
        //     }
        // }

        // // 仅在按键状态变化时触发一次转向
        // if(turn_flag != last_turn_flag)
        // {
        //     last_turn_flag = turn_flag;
        //     if(turn_flag == 1)
        //     {
        //         Start_Angle_Turn(30.0f);  // 右转30度
        //     }
        //     else if(turn_flag == 2)
        //     {
        //         Start_Angle_Turn(-30.0f); // 左转30度
        //     }
        // }   
    }
}
