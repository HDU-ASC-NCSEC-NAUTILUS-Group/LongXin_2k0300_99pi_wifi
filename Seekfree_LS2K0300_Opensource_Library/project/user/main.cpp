#include "zf_common_headfile.h"

timer_fd *pit_timer;

void pit_callback()
{
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

void sigint_handler(int signum) {
    printf("\nCtrl+C caught, exit.\n");
    exit(0);
}

int main() {
    if (!ld_usart_init("/dev/ttyUSB0", 230400))
        return -1;

    signal(SIGINT, sigint_handler);

    pit_timer = new timer_fd(10, pit_callback);
    pit_timer->start();

    while (1) {
        usleep(10000);   // 让主线程休息，定时器自行运行
    }
}
