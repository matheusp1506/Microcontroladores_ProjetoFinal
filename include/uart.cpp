#include "uart.h"

void uart_init(uint32_t baud)
{
    uint16_t ubrr = (F_CPU / 16 / baud) - 1;

    // Set baud rate
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr & 0xFF);

    // Enable TX and RX
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);

    // Frame: 8 data bits, no parity, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); 
}

void uart_send(uint8_t data)
{
    while (!(UCSR0A & (1 << UDRE0))); // wait until TX buffer empty
    UDR0 = data;
}

void uart_send_string(const char *str)
{
    while (*str)
        uart_send(*str++);
}

uint8_t uart_read(void)
{
    while (!(UCSR0A & (1 << RXC0))); // wait for incoming data
    return UDR0;
}

uint8_t uart_available(void)
{
    return (UCSR0A & (1 << RXC0));
}
