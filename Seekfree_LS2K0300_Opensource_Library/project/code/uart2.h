#ifndef _UART2_H_
#define _UART2_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

int uart2_init(void);
void uart2_send_string(const char *str);
void uart2_send_byte(uint8_t data);
int uart2_receive_byte(void);
int uart2_printf(const char *format, ...);
void uart2_close(void);

#ifdef __cplusplus
}
#endif

#endif
