#include "spi.h"
#include <avr/interrupt.h>

static spi_callback_t spi_cb = 0;

void spi_init(spi_clock_t clockDiv, uint8_t mode, uint8_t dataOrder)
{
    // Configure pins:
    // SCK = PB1, MOSI = PB2, MISO = PB3, SS = PB0
    DDRB |= (1 << PB1) | (1 << PB2) | (1 << PB0);
    DDRB &= ~(1 << PB3);  // MISO as input

    // Enable SPI + Master
    SPCR = (1 << SPE) | (1 << MSTR);

    // --- Mode (CPOL/CPHA) ---
    switch(mode) {
        case SPI_MODE0: break;
        case SPI_MODE1: SPCR |= (1 << CPHA); break;
        case SPI_MODE2: SPCR |= (1 << CPOL); break;
        case SPI_MODE3: SPCR |= (1 << CPHA) | (1 << CPOL); break;
    }

    // Data order
    if(dataOrder == SPI_LSB_FIRST)
        SPCR |= (1 << DORD);

    // Clock Divider
    SPCR |= (clockDiv & 0x03);
    SPSR |= (clockDiv & 0x04) ? (1 << SPI2X) : 0;

    // Disable interrupt by default
    SPCR &= ~(1 << SPIE);
}

uint8_t spi_transfer(uint8_t data)
{
    SPDR = data;
    while(!(SPSR & (1 << SPIF)));
    return SPDR;
}

void spi_transfer_async(uint8_t data)
{
    SPCR |= (1 << SPIE);   // enable interrupt
    SPDR = data;           // start transfer
}

void spi_setCallback(spi_callback_t cb)
{
    spi_cb = cb;
}

ISR(SPI_STC_vect)
{
    uint8_t rx = SPDR;
    if(spi_cb)
        spi_cb(rx);
}

void spi_enableInterrupt(void) {
    SPCR |= (1 << SPIE);
}

void spi_disableInterrupt(void) {
    SPCR &= ~(1 << SPIE);
}