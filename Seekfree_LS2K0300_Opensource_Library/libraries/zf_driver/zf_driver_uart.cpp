// zf_driver_uart.c
#include "zf_driver_uart.h"
#include <stddef.h>

// 寄存器偏移（字节）
#define UART_DAT         0x00
#define UART_IER         0x01
#define UART_IIR         0x02
#define UART_FCR         0x02
#define UART_LCR         0x03
#define UART_MCR         0x04
#define UART_LSR         0x05
#define UART_MSR         0x06

// LSR 位定义
#define LSR_TX_EMPTY     (1 << 5)   // 发送保持寄存器空
#define LSR_RX_READY     (1 << 0)   // 接收数据就绪

// LCR 位定义
#define LCR_DLAB         (1 << 7)   // 除数锁存器访问位
#define LCR_8N1          0x03       // 8数据位，无校验，1停止位

// FCR 位定义
#define FCR_ENABLE       0x01       // 使能FIFO
#define FCR_CLEAR_RX     0x02
#define FCR_CLEAR_TX     0x04

// 使用 volatile 指针直接操作
static inline uint8_t read_reg(uint32_t base, uint8_t offset) {
    return *(volatile uint8_t*)(base + offset);
}

static inline void write_reg(uint32_t base, uint8_t offset, uint8_t value) {
    *(volatile uint8_t*)(base + offset) = value;
}

void zf_uart_init(uint32_t uart_base, uint32_t baudrate, uint32_t apb_clk_hz) {
    // 计算分频系数（整数部分）
    uint32_t divisor = apb_clk_hz / (16 * baudrate);
    // 分频系数限制在 1~65535 之间，实际最好有小数部分处理，这里简化
    if (divisor < 1) divisor = 1;
    if (divisor > 65535) divisor = 65535;

    // 1. 设置数据格式前，先使能 DLAB 访问除数锁存器
    write_reg(uart_base, UART_LCR, LCR_DLAB);      // DLAB=1

    // 2. 写入除数低8位和高8位
    write_reg(uart_base, UART_DAT, divisor & 0xFF);
    write_reg(uart_base, UART_IER, (divisor >> 8) & 0xFF);

    // 3. 设置数据格式：8N1
    write_reg(uart_base, UART_LCR, LCR_8N1);       // DLAB=0, 8N1

    // 4. 配置FIFO：使能，清除收发FIFO
    write_reg(uart_base, UART_FCR, FCR_ENABLE | FCR_CLEAR_RX | FCR_CLEAR_TX);

    // 5. 禁用中断（轮询模式）
    write_reg(uart_base, UART_IER, 0x00);

    // 6. 可选：设置MCR为默认（通常不需要）
    write_reg(uart_base, UART_MCR, 0x00);
}

void zf_uart_send_byte(uint32_t uart_base, uint8_t data) {
    // 等待发送保持寄存器空
    while (!(read_reg(uart_base, UART_LSR) & LSR_TX_EMPTY));
    write_reg(uart_base, UART_DAT, data);
}

void zf_uart_send_string(uint32_t uart_base, const char *str) {
    while (*str) {
        zf_uart_send_byte(uart_base, (uint8_t)*str++);
    }
}

bool zf_uart_available(uint32_t uart_base) {
    return (read_reg(uart_base, UART_LSR) & LSR_RX_READY) != 0;
}

uint16_t zf_uart_receive_line(uint32_t uart_base, char *buf, uint16_t max_len) {
    uint16_t idx = 0;
    while (idx < max_len - 1) {
        while (!zf_uart_available(uart_base));
        char c = (char)read_reg(uart_base, UART_DAT);
        buf[idx++] = c;
        if (c == '\n') break;
    }
    buf[idx] = '\0';
    return idx;
}