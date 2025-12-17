#ifndef I2C_H
#define I2C_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

// ---- Configuration ----
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// I2C/TWI Status Codes (from datasheet)
#define I2C_STATUS   (TWSR & 0xF8)

// Operation types
typedef enum {
    I2C_IDLE = 0,
    I2C_WRITE,
    I2C_READ
} i2c_operation_t;

// Callback on transaction finished
typedef void (*i2c_async_callback_t)(uint8_t result);

// ---- Basic control ----
void i2c_init(uint32_t bitrate);
void i2c_enable(void);
void i2c_disable(void);

void i2c_enableAck(void);
void i2c_disableAck(void);

// ---- Blocking operations ----
uint8_t i2c_start(void);
uint8_t i2c_startRepeat(void);
void i2c_stop(void);

uint8_t i2c_write(uint8_t data);
uint8_t i2c_writeAddress(uint8_t addr);

uint8_t i2c_readAck(void);
uint8_t i2c_readNack(void);

// Low-level start functions used by MPU code (non-blocking)
// These do NOT take a callback; completion is signaled via:
//  - per-transaction callback if you used i2c_writeAsync/readAsync
//  - the global callback set with i2c_setCallback()
//  - or by polling i2c_isBusy()
uint8_t i2c_startWrite(uint8_t address, uint8_t reg, const uint8_t *data, uint8_t length);
uint8_t i2c_startRead(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t length);

// Non-blocking high-level transactions
uint8_t i2c_writeAsync(uint8_t address, uint8_t reg, const uint8_t *data, uint8_t length, i2c_async_callback_t cb);
uint8_t i2c_readAsync(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t length, i2c_async_callback_t cb);

// Global driver control
void i2c_setCallback(i2c_async_callback_t cb); // global completion callback
uint8_t i2c_isBusy(void);                       // 0 = idle, 1 = busy

// ---- Interrupt mode ----
void i2c_enableInterrupt(void);
void i2c_disableInterrupt(void);

void i2c_setCallback(i2c_async_callback_t cb);

#endif
