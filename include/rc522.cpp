#include "rc522.h"
#include <util/delay.h>

// Low-level register access -----------------------------------

uint8_t rc522_readReg(uint8_t reg)
{
    RC522_CS_LOW();
    spi_transfer((reg << 1) | 0x80);
    uint8_t val = spi_transfer(0x00);
    RC522_CS_HIGH();
    return val;
}

void rc522_writeReg(uint8_t reg, uint8_t val)
{
    RC522_CS_LOW();
    spi_transfer((reg << 1) & 0x7E);
    spi_transfer(val);
    RC522_CS_HIGH();
}

void rc522_setBitMask(uint8_t reg, uint8_t mask)
{
    uint8_t tmp = rc522_readReg(reg);
    rc522_writeReg(reg, tmp | mask);
}

void rc522_clearBitMask(uint8_t reg, uint8_t mask)
{
    uint8_t tmp = rc522_readReg(reg);
    rc522_writeReg(reg, tmp & (~mask));
}

// Initialization -------------------------------------------------

void rc522_init(void)
{
    RC522_CS_DDR |= (1 << RC522_CS_PIN);
    RC522_CS_HIGH();

    //spi_init();

    rc522_writeReg(RC522_REG_COMMAND, RC522_CMD_SOFT_RESET);
    _delay_ms(50);
    while(rc522_readReg(RC522_REG_COMMAND) & (1 << 4)); // wait for reset to complete

    rc522_writeReg(RC522_TMODE_REG, 0x80);
    rc522_writeReg(RC522_TPRESCALER_REG, 0xA9);
    rc522_writeReg(RC522_TRELOAD_L_REG, 0xE8);
    rc522_writeReg(RC522_TRELOAD_H_REG, 0x03);
    rc522_writeReg(RC522_TX_ASK_REG, 0x40);
    rc522_writeReg(RC522_REG_MODE, 0x3D);

    uint8_t temp = rc522_readReg(RC522_REG_TX_CONTROL);
    if ((temp & 0x03) != 0x03)
    {
        rc522_setBitMask(RC522_REG_TX_CONTROL, temp |0x03);
    }
}

// Request card ---------------------------------------------------
// Funcao feita. mas nao usada, so parte da biblioteca, nao foi testada
uint8_t rc522_request(uint8_t reqMode, uint8_t *tagType)
{
    rc522_writeReg(RC522_REG_BIT_FRAMING, 0x07);
    rc522_writeReg(RC522_REG_FIFO_LEVEL, 0x80);
    rc522_writeReg(RC522_REG_FIFO_DATA, reqMode);
    rc522_writeReg(RC522_REG_COMMAND, RC522_CMD_TRANSCEIVE);
    rc522_setBitMask(RC522_REG_BIT_FRAMING, 0x80);

    uint16_t i = 1000;
    while (i-- && !(rc522_readReg(RC522_REG_COMM_IRQ) & 0x30));

    rc522_clearBitMask(RC522_REG_BIT_FRAMING, 0x80);

    if (!(rc522_readReg(RC522_REG_COMM_IRQ) & 0x30))
        return 1; // timeout

    tagType[0] = rc522_readReg(RC522_REG_FIFO_DATA);
    tagType[1] = rc522_readReg(RC522_REG_FIFO_DATA + 1);

    return 0;
}

// Anti-collision -------------------------------------------------

uint8_t rc522_anticoll(uint8_t *uid)
{
    rc522_writeReg(RC522_REG_BIT_FRAMING, 0x00);
    rc522_writeReg(RC522_REG_FIFO_LEVEL, 0x80);
    rc522_writeReg(RC522_REG_FIFO_DATA, PICC_ANTICOLL);
    rc522_writeReg(RC522_REG_FIFO_DATA, 0x20);

    rc522_writeReg(RC522_REG_COMMAND, RC522_CMD_TRANSCEIVE);
    rc522_setBitMask(RC522_REG_BIT_FRAMING, 0x80);

    uint16_t i = 2000;
    while (i-- && !(rc522_readReg(RC522_REG_COMM_IRQ) & 0x30));

    rc522_clearBitMask(RC522_REG_BIT_FRAMING, 0x80);

    for (uint8_t j = 0; j < 5; j++)
        uid[j] = rc522_readReg(RC522_REG_FIFO_DATA);

    return 0;
}

uint8_t rc522_select(uint8_t *uid)
{
    uint8_t buffer[9];
    buffer[0] = PICC_ANTICOLL;
    buffer[1] = 0x70;

    for (int i = 0; i < 5; i++)
        buffer[i + 2] = uid[i];

    // CRC
    rc522_writeReg(RC522_REG_COMMAND, RC522_CMD_IDLE);
    rc522_writeReg(RC522_REG_FIFO_LEVEL, 0x80);

    for (uint8_t i = 0; i < 7; i++)
        rc522_writeReg(RC522_REG_FIFO_DATA, buffer[i]);

    rc522_writeReg(RC522_REG_COMMAND, RC522_CMD_CALC_CRC);

    while (!(rc522_readReg(RC522_REG_COMM_IRQ) & 0x04));

    buffer[7] = rc522_readReg(RC522_REG_CRC_RESULT_L);
    buffer[8] = rc522_readReg(RC522_REG_CRC_RESULT_H);

    // Send SELECT command
    rc522_writeReg(RC522_REG_FIFO_LEVEL, 0x80);
    for (uint8_t i = 0; i < 9; i++)
        rc522_writeReg(RC522_REG_FIFO_DATA, buffer[i]);

    rc522_writeReg(RC522_REG_COMMAND, RC522_CMD_TRANSCEIVE);
    rc522_setBitMask(RC522_REG_BIT_FRAMING, 0x80);

    while (!(rc522_readReg(RC522_REG_COMM_IRQ) & 0x30));

    rc522_clearBitMask(RC522_REG_BIT_FRAMING, 0x80);

    return 0;
}

void rc522_deactivate(void) {
    RC522_CS_HIGH();
}

void rc522_activate(void) {
    RC522_CS_LOW();
}