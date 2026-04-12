#include "uart2.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

static int uart2_fd = -1;

int uart2_init(void)
{
    uart2_fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart2_fd < 0) {
        printf("UART2 打开失败\n");
        return -1;
    }
    
    struct termios options;
    tcgetattr(uart2_fd, &options);
    
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= CREAD | CLOCAL;
    
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;
    
    if (tcsetattr(uart2_fd, TCSANOW, &options) != 0) {
        printf("UART2 配置失败\n");
        close(uart2_fd);
        uart2_fd = -1;
        return -1;
    }
    
    printf("UART2 初始化成功\n");
    return 0;
}

void uart2_send_string(const char *str)
{
    if (uart2_fd >= 0 && str != NULL) {
        write(uart2_fd, str, strlen(str));
    }
}

void uart2_send_byte(uint8_t data)
{
    if (uart2_fd >= 0) {
        write(uart2_fd, &data, 1);
    }
}

int uart2_receive_byte(void)
{
    if (uart2_fd >= 0) {
        uint8_t data;
        int ret = read(uart2_fd, &data, 1);
        if (ret == 1) {
            return data;
        }
    }
    return -1;
}

int uart2_printf(const char *format, ...)
{
    if (uart2_fd < 0 || format == NULL) {
        return -1;
    }
    
    char buffer[512];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len > 0) {
        write(uart2_fd, buffer, len);
    }
    return len;
}

void uart2_close(void)
{
    if (uart2_fd >= 0) {
        close(uart2_fd);
        uart2_fd = -1;
    }
}
