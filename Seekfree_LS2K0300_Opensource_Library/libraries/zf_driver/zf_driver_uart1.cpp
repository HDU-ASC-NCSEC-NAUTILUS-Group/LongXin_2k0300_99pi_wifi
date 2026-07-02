#include "zf_driver_uart1.h"
#include <stdio.h>
#include <stdarg.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <poll.h>

static int  g_uart1_fd = -1;
static char g_uart1_txbuf[256];

int uart1_init(const char *device, uint32 baudrate)
{
    g_uart1_fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (g_uart1_fd < 0) {
        fprintf(stderr, "uart1_init: open %s failed: %s\n", device, strerror(errno));
        return -1;
    }

    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    tcgetattr(g_uart1_fd, &tio);

    speed_t speed = B115200;
    if (baudrate >= 921600)      speed = B921600;
    else if (baudrate >= 460800) speed = B460800;
    else if (baudrate >= 230400) speed = B230400;
    else if (baudrate >= 115200) speed = B115200;
    else if (baudrate >= 57600)  speed = B57600;
    else if (baudrate >= 38400)  speed = B38400;
    else if (baudrate >= 19200)  speed = B19200;
    else                         speed = B9600;

    cfsetospeed(&tio, speed);
    cfsetispeed(&tio, speed);

    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |=  CS8;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag |=  CREAD | CLOCAL;

    tio.c_lflag = 0;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_oflag = 0;

    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    tcsetattr(g_uart1_fd, TCSANOW, &tio);
    tcflush(g_uart1_fd, TCIOFLUSH);

    return 0;
}

void uart1_deinit(void)
{
    if (g_uart1_fd >= 0) {
        close(g_uart1_fd);
        g_uart1_fd = -1;
    }
}

int uart1_send(const uint8 *data, uint32 len)
{
    if (g_uart1_fd < 0 || data == NULL || len == 0) return -1;

    int total = 0;
    while (total < (int)len) {
        int n = write(g_uart1_fd, data + total, len - total);
        if (n < 0) {
            if (errno == EAGAIN) continue;
            return -1;
        }
        total += n;
    }
    return total;
}

int uart1_recv(uint8 *buf, uint32 maxlen)
{
    if (g_uart1_fd < 0 || buf == NULL || maxlen == 0) return -1;

    int n = read(g_uart1_fd, buf, maxlen);
    if (n < 0) {
        if (errno == EAGAIN) return 0;
        return -1;
    }
    return n;
}

int uart1_available(void)
{
    if (g_uart1_fd < 0) return 0;

    struct pollfd pfd;
    pfd.fd     = g_uart1_fd;
    pfd.events = POLLIN;
    return (poll(&pfd, 1, 0) > 0) && (pfd.revents & POLLIN);
}

int uart1_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(g_uart1_txbuf, sizeof(g_uart1_txbuf), fmt, args);
    va_end(args);

    if (len <= 0) return -1;
    return uart1_send((const uint8*)g_uart1_txbuf, (uint32)len);
}

// 被验证可行的调用方法
// 这个测试函数可以直接替代所有main代码进行独立测试使用的
//
//
//
// #include "zf_common_headfile.h"
// #include <time.h>

// static uint32_t tick = 0;

// int main()
// {
//     if (uart1_init("/dev/ttyS1", 115200)) {
//         printf("uart1 初始化失败\n");
//         return -1;
//     }
//     printf("uart1 初始化成功, 开始双向通信\n\n");

//     while (1) {
//         // ===== 接收 =====
//         uint8_t buf[64];
//         int n = uart1_recv(buf, sizeof(buf) - 1);
//         if (n > 0) {
//             buf[n] = 0;
//             printf("收到[%dB]: %s", n, buf);
//         }

//         // ===== 每 2 秒发送一次 =====
//         tick++;
//         if (tick >= 200) {
//             tick = 0;

//             time_t now = time(NULL);
//             struct tm *t = localtime(&now);
//             uart1_printf("[%02d:%02d:%02d] board alive\n",
//                          t->tm_hour, t->tm_min, t->tm_sec);
//         }

//         system_delay_ms(10);
//     }
// }