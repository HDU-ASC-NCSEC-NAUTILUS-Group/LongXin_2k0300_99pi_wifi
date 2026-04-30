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

timer_fd *pit_timer_10ms;
timer_fd *pit_timer_200ms;

void pit_callback_10ms()
{
    Key_Tick();
    IMU_D_and_A_Enable = 1;

    ld_usart_task();            // 快速拉取所有可用字节并解包

    if (g_lidar_frame_valid) {
        // 打印一帧关键信息
        printf("Radar: speed=%u rpm, angle %.2f->%.2f, ts=%u\n",
               g_lidar_frame.speed,
               g_lidar_frame.start_angle * 0.01f,
               g_lidar_frame.end_angle * 0.01f,
               g_lidar_frame.timestamp);

        for (int i = 0; i < POINT_PER_PACK; i++) {
            printf("  pt[%d]: %u mm, %u\n",
                   i,
                   g_lidar_frame.point[i].distance,
                   g_lidar_frame.point[i].intensity);
        }

        g_lidar_frame_valid = false;
    }
}

void pit_callback_200ms()
{
    Time_200ms_Flag = 1;
}

// 清场函数（保护性措施）
void cleanup()
{
    // 需要先停止定时器线程，后面才能稳定关闭电机，电调，舵机等
    pit_timer_10ms->stop();
    pit_timer_200ms->stop();

    printf("程序退出，执行清理操作\n");
    Motor_Reset_ALL();
}
// 宣告程序退出函数
void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}

int main(int, char**) 
{
    // 注册清理函数
    // 需要定义，作为退出整个程序时的重置函数（比如关闭电机、蜂鸣器等）
    atexit(cleanup);
    // 注册SIGINT信号的处理函数
    signal(SIGINT, sigint_handler);

    // 外设初始化
    Peripheral_Init();

    if (!ld_usart_init("/dev/ttyUSB0", 230400))
        return -1;

    // 创建10ms定时器
    pit_timer_10ms = new timer_fd(10, pit_callback_10ms);
    pit_timer_10ms->start();  
    // 创建200ms定时器
    pit_timer_200ms = new timer_fd(200, pit_callback_200ms);
    pit_timer_200ms->start();  

    while(1)
    {
        Menu_Show();
        // usleep(10000);  
    }
}
