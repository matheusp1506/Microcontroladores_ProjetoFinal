#ifndef SPI_H
#define SPI_H

#include <avr/io.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// SPI Modes
#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3

// Data Order
#define SPI_LSB_FIRST 1
#define SPI_MSB_FIRST 0

// Clock Dividers
typedef enum {
    SPI_CLOCK_DIV4   = 0,
    SPI_CLOCK_DIV16  = 1,
    SPI_CLOCK_DIV64  = 2,
    SPI_CLOCK_DIV128 = 3,
    SPI_CLOCK_DIV2   = 4,
    SPI_CLOCK_DIV8   = 5,
    SPI_CLOCK_DIV32  = 6,
    // 7 also equals 64, but rarely needed
} spi_clock_t;

// Callback type for interrupt based transfer
typedef void (*spi_callback_t)(uint8_t receivedByte);

// API
void spi_init(spi_clock_t clockDiv, uint8_t mode, uint8_t dataOrder);

uint8_t spi_transfer(uint8_t data);          // Blocking transfer
void spi_transfer_async(uint8_t data);       // Starts async transfer via interrupt

void spi_enableInterrupt(void);
void spi_disableInterrupt(void);

void spi_setCallback(spi_callback_t cb);

#ifdef __cplusplus
}
#endif

#endif
