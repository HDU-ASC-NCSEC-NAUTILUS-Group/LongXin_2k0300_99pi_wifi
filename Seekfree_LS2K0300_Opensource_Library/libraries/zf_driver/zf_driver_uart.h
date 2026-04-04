#ifndef _ZF_DRIVER_UART_H_
#define _ZF_DRIVER_UART_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void zf_uart_init(uint32_t uart_base, uint32_t baudrate, uint32_t apb_clk_hz);
void zf_uart_send_byte(uint32_t uart_base, uint8_t data);
void zf_uart_send_string(uint32_t uart_base, const char *str);
bool zf_uart_available(uint32_t uart_base);
uint16_t zf_uart_receive_line(uint32_t uart_base, char *buf, uint16_t max_len);

#ifdef __cplusplus
}
#endif

#endif