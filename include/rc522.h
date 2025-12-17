#ifndef RC522_H
#define RC522_H

#include <stdint.h>
#include <avr/io.h>
#include "spi.h"

// ------------------ PIN CONFIG ------------------
#define RC522_CS_PORT PORTA
#define RC522_CS_DDR  DDRA
#define RC522_CS_PIN  PA1

#define RC522_CS_LOW()   (RC522_CS_PORT &= ~(1 << RC522_CS_PIN))
#define RC522_CS_HIGH()  (RC522_CS_PORT |=  (1 << RC522_CS_PIN))

// ------------------ RC522 REGISTERS ------------------
#define RC522_REG_COMMAND       0x01
#define RC522_REG_COMM_IE       0x02
#define RC522_REG_COMM_IRQ      0x04
#define RC522_REG_FIFO_LEVEL    0x0A
#define RC522_REG_FIFO_DATA     0x09
#define RC522_REG_CONTROL       0x0C
#define RC522_REG_BIT_FRAMING   0x0D
#define RC522_REG_MODE          0x11
#define RC522_REG_TX_MODE       0x12
#define RC522_REG_RX_MODE       0x13
#define RC522_REG_TX_CONTROL    0x14
#define RC522_REG_CRC_RESULT_L  0x22
#define RC522_REG_CRC_RESULT_H  0x21
#define RC522_TMODE_REG         0x2A
#define RC522_TPRESCALER_REG    0x2B
#define RC522_TRELOAD_L_REG     0x2D
#define RC522_TRELOAD_H_REG     0x2C
#define RC522_TX_ASK_REG       0x15

// Commands
#define RC522_CMD_IDLE          0x00
#define RC522_CMD_TRANSCEIVE    0x0C
#define RC522_CMD_CALC_CRC      0x03
#define RC522_CMD_SOFT_RESET    0x0F

// Card commands
#define PICC_REQIDL    0x26
#define PICC_ANTICOLL  0x93

// ------------------ FUNCTIONS ------------------

void rc522_init(void);
uint8_t rc522_readReg(uint8_t reg);
void rc522_writeReg(uint8_t reg, uint8_t val);
void rc522_setBitMask(uint8_t reg, uint8_t mask);
void rc522_clearBitMask(uint8_t reg, uint8_t mask);

uint8_t rc522_request(uint8_t reqMode, uint8_t *tagType);
uint8_t rc522_anticoll(uint8_t *uid);
uint8_t rc522_select(uint8_t *uid);

void rc522_deactivate(void);
void rc522_activate(void);

#endif