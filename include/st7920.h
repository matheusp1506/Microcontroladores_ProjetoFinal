#ifndef ST7920_H
#define ST7920_H

#include <avr/io.h>

#define DISP_CLEAR                   0x01
#define DISP_HOME                    0x02
#define DISP_ENTRY(incr,shift)       (0x04 | ((incr)?0x02:0) | ((shift)?0x01:0))
#define DISP_ON(on,cur,blink)        (0x08 | ((on)?0x04:0) | ((cur)?0x02:0) | ((blink)?0x01:0))
#define DISP_SHIFT(cur,right)        (0x10 | ((cur)?0x08:0) | ((right)?0x04:0))
#define DISP_FUNC_SET(data8,ext,rev) (0x20 | ((data8)?0x10:0) | ((ext)?0x04:0) | ((rev)?0x02:0))
#define DISP_CGRAM_ADDR(a)           (0x40 | ((a)&0x3F))
#define DISP_DDRAM_ADDR(a)           (0x80 | ((a)&0x7F))

// Extended instruction set (when EXT=1)
#define DISP_STANDBY        0x01
#define DISP_VSCROLL(s)     (0x02 | ((s)?0x01:0))
#define DISP_REVERSE(v)     (0x04 | ((v)&0x03))
#define DISP_SCROLL_ADDR(a) (0x40 | ((a)&0x3F))
#define DISP_GRAPHIC_RAM(a) (0x80 | ((a)&0x3F))
// ------------------------------------------------

void st7920_init(void);
void st7920_cmd(uint8_t cmd);
void st7920_data(uint8_t data);
void st7920_changeMode(uint8_t extended);
void st7920_send_byte(uint8_t value, uint8_t isData);
void st7920_send_bytes(const uint8_t *data, uint16_t length, uint8_t isData);
void st7920_print(const char *str);
uint8_t st7920_isInMode(uint8_t mode);
void st7920_disableCursor(void);

#endif