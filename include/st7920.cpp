#include "st7920.h"
#include <util/delay.h>


// ------------------------------------------------
// Pin definitions
// ------------------------------------------------

// RS = Chip Select
#define RS_PC2   (1 << PC2)

// MOSI + SCLK
#define SID_PIN  (1 << PL7)
#define SCK_PIN  (1 << PL6)

#define RS_HIGH()    (PORTC |= RS_PC2)
#define RS_LOW()     (PORTC &= ~RS_PC2)

#define MOSI_HIGH()  (PORTL |=  SID_PIN)
#define MOSI_LOW()   (PORTL &= ~SID_PIN)

#define SCLK_HIGH()  (PORTL |=  SCK_PIN)
#define SCLK_LOW()   (PORTL &= ~SCK_PIN)

volatile uint8_t st7920_inExtendedMode = 0;

// ------------------------------------------------
// Internal helper: stable idle state when chip disabled
// ------------------------------------------------
static inline void st7920_idle_bus(void)
{
    // Keep MOSI and SCLK stable (L or H). We choose LOW.
    MOSI_LOW();
    SCLK_LOW();
    RS_LOW();  // chip disabled
}


// ------------------------------------------------
// Bit-bang send byte (RS already indicates data/cmd)
// ------------------------------------------------
void st7920_send_byte(uint8_t value, uint8_t isData)
{
    // ---- ENABLE CHIP ----
    RS_HIGH();  // after this point MOSI/SCLK may toggle safely

    // 7-bit Header: 1 1 1 1 1 0(Write) RSbit
    uint8_t header = 0xF8 | (isData ? 0x02 : 0);

    // Send header bits (SYNC SYNC SYNC SYNC SYNC RW RS)
    for (uint8_t mask = 0x80; mask >= 0x01; mask >>= 1)
    {
        (header & mask) ? MOSI_HIGH() : MOSI_LOW();

        SCLK_HIGH();
        _delay_us(10);
        SCLK_LOW();
    }

    // Send high nibble
    uint8_t high = value & 0xF0;
    for (uint8_t mask = 0x80; mask >= 0x01; mask >>= 1)
    {
        (high & mask) ? MOSI_HIGH() : MOSI_LOW();

        SCLK_HIGH();
        _delay_us(10);
        SCLK_LOW();
    }

    // Send low nibble
    uint8_t low = (value << 4) & 0xF0;
    for (uint8_t mask = 0x80; mask >= 0x01; mask >>= 1)
    {
        (low & mask) ? MOSI_HIGH() : MOSI_LOW();

        SCLK_HIGH();
        _delay_us(10);
        SCLK_LOW();
    }

    // required write delay
    _delay_us(50);

    // ---- DISABLE CHIP ----
    st7920_idle_bus();  // ensures MOSI/SCLK stay stable while RS=0
}

void st7920_send_bytes(const uint8_t *data, uint16_t length, uint8_t isData) {
    // ---- ENABLE CHIP ----
    RS_HIGH();  // after this point MOSI/SCLK may toggle safely

    // 7-bit Header: 1 1 1 1 1 0(Write) RSbit
    uint8_t header = 0b11111000 | (isData ? 0b00000010 : 0);

    // Send header bits (SYNC SYNC SYNC SYNC SYNC RW RS)
    for (uint8_t mask = 0x80; mask >= 0x01; mask >>= 1)
    {
        (header & mask) ? MOSI_HIGH() : MOSI_LOW();

        SCLK_HIGH();
        _delay_us(10);
        SCLK_LOW();
    }

    for(uint16_t i = 0; i < length; i++) {
        uint8_t value = data[i];
    // Send high nibble
    uint8_t high = value & 0xF0;
    for (uint8_t mask = 0x80; mask >= 0x01; mask >>= 1)
    {
        (high & mask) ? MOSI_HIGH() : MOSI_LOW();

        SCLK_HIGH();
        _delay_us(10);
        SCLK_LOW();
    }

    // Send low nibble
    uint8_t low = (value << 4) & 0xF0;
    for (uint8_t mask = 0x80; mask >= 0x01; mask >>= 1)
    {
        (low & mask) ? MOSI_HIGH() : MOSI_LOW();

        SCLK_HIGH();
        _delay_us(10);
        SCLK_LOW();
    }
    }

    // required write delay
    _delay_us(50);

    // ---- DISABLE CHIP ----
    st7920_idle_bus();  // ensures MOSI/SCLK stay stable while RS=0
}

// ------------------------------------------------
// Public API
// ------------------------------------------------
void st7920_cmd(uint8_t cmd)
{
    st7920_send_byte(cmd, 0);
}

void st7920_data(uint8_t data)
{
    st7920_send_byte(data, 1);
}

void st7920_changeMode(uint8_t extended)
{
    if(extended && !st7920_inExtendedMode) {
        st7920_cmd(DISP_FUNC_SET(1, 1, 1)); // switch to extended
        st7920_inExtendedMode = 1;
    }
    else if(!extended && st7920_inExtendedMode) {
        st7920_cmd(DISP_FUNC_SET(1, 0, 0)); // switch to basic
        st7920_inExtendedMode = 0;
    }
}

void st7920_print(const char *str)
{
    st7920_cmd(DISP_FUNC_SET(1, 0, 0));

    while (*str) {
        st7920_data(*str);
        str++;
    }
}

uint8_t st7920_isInMode(uint8_t mode)
{  
    if(mode == 0) // basic
        return (st7920_inExtendedMode == 0);
    else          // extended
        return (st7920_inExtendedMode == 1);
}

void st7920_disableCursor(void)
{
    uint8_t wasExtended = st7920_inExtendedMode;
    
    // Switch to basic mode if needed
    if(wasExtended) {
        st7920_changeMode(0);
    }
    
    // Send display ON command with cursor and blink OFF
    st7920_cmd(DISP_ON(1, 0, 0));
    
    // Return to previous mode
    if(wasExtended) {
        st7920_changeMode(1);
    }
}

// ------------------------------------------------
// Initialization
// ------------------------------------------------
void st7920_init(void)
{
    // Configure pins
    DDRC |= RS_PC2;
    DDRL |= SID_PIN | SCK_PIN;

    st7920_idle_bus();  // stable idle state at startup

    _delay_ms(50);

    st7920_cmd(DISP_FUNC_SET(1, 0, 0)); // 8-bit, basic instruction set
    _delay_ms(2);
    st7920_cmd(DISP_FUNC_SET(1, 0, 0));

    st7920_cmd(DISP_ENTRY(1, 0)); // Increment, no shift
    st7920_cmd(DISP_SHIFT(0, 0)); // No shift

    st7920_cmd(DISP_CGRAM_ADDR(0)); // Set CGRAM address to 0
    st7920_cmd(DISP_DDRAM_ADDR(0)); // Set DDRAM address to 0

    st7920_cmd(DISP_CLEAR); // Clear
    st7920_cmd(DISP_HOME);  // Home

    st7920_cmd(DISP_ON(1, 0, 0)); // Display ON
    
    _delay_ms(2);
}

