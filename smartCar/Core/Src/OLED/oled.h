#ifndef OLED_H
#define OLED_H

#include <stdint.h>


// the I2C address of oled
#define OLED_I2C_ADDRESS    0x78

//the resolution of oled   128*64
#define MAX_COLUMN      128
#define MAX_ROW         64

#define X_WIDTH         MAX_COLUMN
#define Y_WIDTH         MAX_ROW

#define OLED_CMD        0x00
#define OLED_DATA       0x01

typedef enum
{
    PEN_CLEAR = 0x00,
    PEN_WRITE = 0x01,
    PEN_INVERSION= 0x02,
}pen_typedef;
extern void OLED_init(void);
extern void OLED_display_on(void);
extern void OLED_display_off(void);
extern void OLED_operate_gram(pen_typedef pen);
extern void OLED_set_pos(uint8_t x, uint8_t y);
extern void OLED_draw_point(int8_t x, int8_t y, pen_typedef pen);
extern void OLED_show_string(uint8_t row, uint8_t col, uint8_t *chr);
extern void OLED_printf(uint8_t row, uint8_t col, const char *fmt,...);
extern void OLED_refresh_gram(void);
extern void OLED_LOGO(void);
#endif







