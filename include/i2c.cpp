#include "i2c.h"
#include <string.h>

/* Internal state */
static volatile const uint8_t *tx_buf = 0;
static volatile uint8_t *rx_buf = 0;
static volatile uint8_t tx_len = 0;
static volatile uint8_t rx_len = 0;
static volatile uint8_t idx = 0;
static volatile uint8_t address_rw = 0; // 7-bit address shifted + R/W bit
static volatile i2c_operation_t op = I2C_IDLE;

static i2c_async_callback_t trans_callback = 0; // per-transaction callback
static i2c_async_callback_t global_callback = 0; // global callback

/* Helper: start TWI as master sending START + enable interrupt */
static inline void twi_send_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
}

// ======================================================
// Initialization
// ======================================================
void i2c_init(uint32_t bitrate) {
    // Calculate TWBR register based on bitrate
    uint32_t twbr_val = ((F_CPU / bitrate) - 16) / 2;
    uint8_t prescaler = 0;

    while (twbr_val > 255 && prescaler < 3) {
        prescaler++;
        TWSR = (TWSR & 0xFC) | prescaler;
        twbr_val = (((F_CPU / bitrate) - 16) / 2) / (1 << (2 * prescaler));
    }

    TWBR = (uint8_t)twbr_val;

    i2c_enable();
}

// ======================================================
// Basic enable/disable
// ======================================================
void i2c_enable(void) {
    TWCR = (1 << TWEN);
}

void i2c_disable(void) {
    TWCR = 0;
}

// ======================================================
// ACK control
// ======================================================
void i2c_enableAck(void) {
    TWCR |= (1 << TWEA);
}

void i2c_disableAck(void) {
    TWCR &= ~(1 << TWEA);
}

// ======================================================
// Blocking operations
// ======================================================
uint8_t i2c_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return I2C_STATUS;
}

uint8_t i2c_startRepeat(void) {
    return i2c_start();
}

void i2c_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

uint8_t i2c_writeAddress(uint8_t addr) {
    return i2c_write(addr);
}

uint8_t i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return I2C_STATUS;
}

uint8_t i2c_readAck(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t i2c_readNack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

// ======================================================
// Interrupt mode support
// ======================================================
void i2c_enableInterrupt(void) {
    TWCR |= (1 << TWIE);
}

void i2c_disableInterrupt(void) {
    TWCR &= ~(1 << TWIE);
}

uint8_t i2c_isBusy(void) {
    return (op != I2C_IDLE) ? 1 : 0;
}

void i2c_setCallback(i2c_async_callback_t cb) {
    global_callback = cb;
}

uint8_t i2c_writeAsync(uint8_t address, uint8_t reg, const uint8_t *data, uint8_t length, i2c_async_callback_t cb)
{
    if (i2c_isBusy()) return 0;
    /* prepare tx buffer: we need to send register first then data */
    /* For simplicity we will store reg as first byte via a small local buffer if length small.
       To avoid dynamic allocation, if small data (<= 16) copy into static buffer; for larger,
       user should call i2c_startWrite passing an externally prepared buffer. */
    static uint8_t small_buf[32];
    if ((uint8_t)(length + 1) > sizeof(small_buf)) return 0; // too big for convenience wrapper

    small_buf[0] = reg;
    if (length) memcpy(&small_buf[1], data, length);

    tx_buf = small_buf;
    tx_len = length + 1;
    idx = 0;
    rx_buf = 0;
    rx_len = 0;

    trans_callback = cb;
    op = I2C_WRITE;
    address_rw = (address << 1) | 0; // write

    /* enable TWI interrupt and global interrupts then send START */
    TWCR |= (1 << TWIE);
    sei();
    twi_send_start();
    return 1;
}

uint8_t i2c_readAsync(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t length, i2c_async_callback_t cb)
{
    if (i2c_isBusy()) return 0;

    /* For read with register, driver does a write(reg) then repeated-start read */
    /* We'll implement this using an internal small buffer to send reg first. */
    static uint8_t reg_buf[1];
    reg_buf[0] = reg;

    tx_buf = reg_buf;
    tx_len = 1;
    idx = 0;

    rx_buf = buffer;
    rx_len = length;

    trans_callback = cb;
    op = I2C_READ;
    address_rw = (address << 1) | 0; // first send as write (R/W=0), ISR will switch to read after repeated start

    TWCR |= (1 << TWIE);
    sei();
    twi_send_start();
    return 1;
}

/* Lower-level start functions used by MPU code (no direct callback param).
   These use the same internal machinery and will call the global callback (if any)
   or the per-transaction callback (if set). */
uint8_t i2c_startWrite(uint8_t address, uint8_t reg, const uint8_t *data, uint8_t length)
{
    if (i2c_isBusy()) return 0;

    /* assemble small buffer (reg + data) like writeAsync wrapper */
    static uint8_t small_buf[32];
    if ((uint8_t)(length + 1) > sizeof(small_buf)) return 0;

    small_buf[0] = reg;
    if (length) memcpy(&small_buf[1], data, length);

    tx_buf = small_buf;
    tx_len = length + 1;
    idx = 0;
    rx_buf = 0;
    rx_len = 0;

    trans_callback = 0; // no per-transaction cb
    op = I2C_WRITE;
    address_rw = (address << 1) | 0;

    TWCR |= (1 << TWIE);
    sei();
    twi_send_start();
    return 1;
}

uint8_t i2c_startRead(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t length)
{
    if (i2c_isBusy()) return 0;

    static uint8_t reg_buf[1];
    reg_buf[0] = reg;

    tx_buf = reg_buf;
    tx_len = 1;
    idx = 0;

    rx_buf = buffer;
    rx_len = length;

    trans_callback = 0;
    op = I2C_READ;
    address_rw = (address << 1) | 0; // start as write for reg send

    TWCR |= (1 << TWIE);
    sei();
    twi_send_start();
    return 1;
}

// ======================================================
// ISR (TWI vector)
// ======================================================
/* ISR: TWI_vect for ATmega2560 */
ISR(TWI_vect)
{
    uint8_t status = I2C_STATUS;

    /* if idle, optionally forward status to global callback */
    if (op == I2C_IDLE) {
        /* let user handle raw status if they want */
        if (global_callback) global_callback(status);
        return;
    }

    switch (status) {
        /* START/REPEATED START transmitted */
        case 0x08: /* START */
        case 0x10: /* repeated START */
            /* send SLA+W or SLA+R depending on current address_rw */
            TWDR = (uint8_t)address_rw;
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
            break;

        /* SLA+W transmitted, ACK received */
        case 0x18:
            /* send first data byte */
            idx = 0;
            if (tx_len > 0) {
                TWDR = tx_buf[idx++];
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
            } else {
                /* nothing to send, success */
                goto async_success;
            }
            break;

        /* SLA+W transmitted, NACK received */
        case 0x20:
            goto async_error;

        /* Data byte transmitted, ACK received */
        case 0x28:
            if (idx < tx_len) {
                TWDR = tx_buf[idx++];
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
            } else {
                /* If operation was READ (we had sent reg), do repeated start for read */
                if (op == I2C_READ) {
                    /* send repeated START and then SLA+R in next START branch */
                    address_rw = (address_rw & 0xFE) | 1; // set R/W = 1
                    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
                } else {
                    goto async_success;
                }
            }
            break;

        /* Data byte transmitted, NACK received */
        case 0x30:
            goto async_error;

        /* SLA+R transmitted, ACK received */
        case 0x40:
            /* if more than one byte to read -> ACK after reception, otherwise NACK */
            if (rx_len > 1) {
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
            } else {
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE); // NACK after reception
            }
            idx = 0;
            break;

        /* SLA+R transmitted, NACK received */
        case 0x48:
            goto async_error;

        /* Data received, ACK returned */
        case 0x50:
            if (rx_buf && idx < rx_len) {
                rx_buf[idx++] = TWDR;
            }
            if (idx < rx_len - 1) {
                /* more bytes to come -> ACK next */
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
            } else {
                /* next byte will be last -> NACK after next reception */
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
            }
            break;

        /* Data received, NACK returned (last byte) */
        case 0x58:
            if (rx_buf && idx < rx_len) {
                rx_buf[idx++] = TWDR;
            }
            goto async_success;

        /* Arbitration lost or other error */
        case 0x38:
        default:
            goto async_error;
    }

    return;

async_success:
    /* issue STOP */
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    op = I2C_IDLE;

    /* call per-transaction callback if present */
    if (trans_callback) {
        trans_callback(0);
        trans_callback = 0;
    }
    /* call global callback if present */
    if (global_callback) global_callback(0);
    return;

async_error:
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    op = I2C_IDLE;

    if (trans_callback) {
        trans_callback(1);
        trans_callback = 0;
    }
    if (global_callback) global_callback(1);
    return;
}