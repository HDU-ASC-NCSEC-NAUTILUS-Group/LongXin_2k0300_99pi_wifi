
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
* 2026-03-22        AI             添加红色物块识别与坐标显示
********************************************************************************************************************/
#include "zf_common_headfile.h"

extern struct pwm_info servo_pwm_info;

#define RAD_TO_DEG (180.0f / 3.14159265358979f)

timer_fd *pit_timer;

// 里面放入中断代码
void pit_callback()
{
    imu963ra_get_acc();
    imu963ra_get_gyro();
    gyro_offset_get();

    ICM_AHRSupdate(imu963ra_gyro_x, imu963ra_gyro_y, imu963ra_gyro_z, imu963ra_acc_x, imu963ra_acc_y, imu963ra_acc_z);
    imu_get_angle();
}

void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}

void cleanup()
{
    printf("程序异常退出，执行清理操作\n");
    // 此处可添加电机、电调等关闭代码
}

int main(int, char**) 
{
    // 注册清理函数和信号处理
    atexit(cleanup);
    signal(SIGINT, sigint_handler);

    // 初始化屏幕
    ips200_init("/dev/fb0");

    // 初始化UVC摄像头
    // if (uvc_camera_init("/dev/video0") < 0) {
    //     return -1;
    // }

    // 获取PWM设备信息
    // pwm_get_dev_info(SERVO_MOTOR1_PWM, &servo_pwm_info);

    // 创建一个定时器10ms周期，回调函数为pit_callback
    pit_timer = new timer_fd(10, pit_callback);
    pit_timer->start();

    // 主循环
    while (1) {
        printf("yaw = %.2f\r\n", imu_angle.yaw);
    }

    return 0;
}
