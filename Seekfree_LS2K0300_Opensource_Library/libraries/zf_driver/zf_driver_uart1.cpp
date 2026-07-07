#include "zf_driver_uart1.h"
#include <stdio.h>
#include <stdarg.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <poll.h>


/**********************************************************/
/*[S] 基础驱动 [S]-----------------------------------------*/
/**********************************************************/

static int  g_uart1_fd = -1;
static char g_uart1_txbuf[256];

// uart1初始化
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

// uart1发送
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

// uart1接收
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

// uart1格式化发送
int uart1_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(g_uart1_txbuf, sizeof(g_uart1_txbuf), fmt, args);
    va_end(args);

    if (len <= 0) return -1;
    return uart1_send((const uint8*)g_uart1_txbuf, (uint32)len);
}

/**********************************************************/
/*-----------------------------------------[E] 基础驱动 [E]*/
/**********************************************************/

// 被验证可行的调用方法
// 这个测试函数可以直接替代所有main代码进行独立测试使用的
// 验证时期极早期，请注意
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


/**********************************************************/
/*[S] 接收二次封装 [S]--------------------------------------*/
/**********************************************************/

#define UART1_PARSE_BUF_SIZE  128

static uint8_t  rx_buf[UART1_PARSE_BUF_SIZE];
static int      rx_pos = 0;
static char     frame[UART1_PARSE_BUF_SIZE];

char* uart1_recv_frame(void)
{
    int free_len = sizeof(rx_buf) - rx_pos - 1;
    if (free_len <= 0) {
        rx_pos = 0;          // 满缓冲保护：清空恢复
        rx_buf[0] = '\0';
        free_len = sizeof(rx_buf) - 1;
    }

    int n = uart1_recv(rx_buf + rx_pos, free_len);
    if (n <= 0) return NULL;
    rx_pos += n;
    rx_buf[rx_pos] = '\0';

    char *end = strchr((char*)rx_buf, '\n');
    if (!end) return NULL;
    *end = '\0';

    char *line  = (char*)rx_buf;                 // 限定搜索范围在当前行内
    char *start = strchr(line, '[');
    char *stop  = start ? strchr(start + 1, ']') : NULL;

    if (start && stop && stop > start + 1) {     // 非空包体
        int len = stop - start - 1;
        if (len >= UART1_PARSE_BUF_SIZE) len = UART1_PARSE_BUF_SIZE - 1;
        memcpy(frame, start + 1, len);
        frame[len] = '\0';
    } else {
        frame[0] = '\0';
    }

    int remain = rx_pos - (end + 1 - (char*)rx_buf);
    if (remain > 0) {
        memmove(rx_buf, end + 1, remain);
        rx_pos = remain;
    } else {
        rx_pos = 0;
    }

    return frame[0] ? frame : NULL;
}

/**********************************************************/
/*--------------------------------------[E] 接收二次封装 [E]*/
/**********************************************************/