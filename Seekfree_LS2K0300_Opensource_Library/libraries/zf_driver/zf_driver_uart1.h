/*
 * UART1 驱动 — P42(RX)/P43(TX), /dev/ttyS1
 *
 * ---- 初始化 ----
 *   if (uart1_init("/dev/ttyS1", 115200))
 *       printf("uart1 初始化失败\n");
 *
 * ---- 发送 ----
 *   uart1_printf("angle=%d\n", 90);                  // 格式化发送
 *   uart1_send((uint8_t*)"hello", 5);                // 二进制发送
 *
 * ---- 接收 (非阻塞, 放主循环/定时中断里轮询, 推荐间隔 5~10ms) ----
 *   uint8_t buf[64];
 *   int n = uart1_recv(buf, sizeof(buf)-1);          // 有数据返字节数, 无数据返0
 *   if (n > 0) { buf[n]=0;  printf("收到: %s", buf); }
 *
 * ---- 带标志位的接收 (推荐, 避免频繁无效调用) ----
 *   if (uart1_available()) {                          // 先检查有无数据
 *       int n = uart1_recv(buf, sizeof(buf)-1);
 *       ...
 *   }
 *
 * ---- 关闭 ----
 *   uart1_deinit();
 */

#ifndef _zf_driver_uart1_h
#define _zf_driver_uart1_h

#include "zf_common_typedef.h"

int  uart1_init(const char *device, uint32 baudrate);
void uart1_deinit(void);
int  uart1_send(const uint8 *data, uint32 len);
int  uart1_recv(uint8 *buf, uint32 maxlen);
int  uart1_available(void);
int  uart1_printf(const char *fmt, ...);

#endif