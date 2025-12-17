#ifndef UART_H
#define UART_H

#include <avr/io.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void uart_init(uint32_t baud);
void uart_send(uint8_t data);
void uart_send_string(const char *str);
uint8_t uart_read(void);
uint8_t uart_available(void);

#ifdef __cplusplus
}
#endif

#endif
