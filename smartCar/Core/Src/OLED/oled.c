#include "OLED.h" 
#include "asc.h"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>

extern I2C_HandleTypeDef hi2c1;

static uint8_t OLED_GRAM[128][8];

void oled_write_byte(uint8_t dat, uint8_t cmd)
{
    static uint8_t cmd_data[2];
    if(cmd == OLED_CMD)
    {
        cmd_data[0] = 0x00;
    }
    else
    {
        cmd_data[0] = 0x40;
    }
    cmd_data[1] = dat;
    HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDRESS, cmd_data, 2, 10);
}
//初始化OLED模块，
 void OLED_init(void)
{
    oled_write_byte(0xAE, OLED_CMD);    //display off
    oled_write_byte(0x20, OLED_CMD);    //Set Memory Addressing Mode	
    oled_write_byte(0x10, OLED_CMD);    //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    oled_write_byte(0xb0, OLED_CMD);    //Set Page Start Address for Page Addressing Mode,0-7
    oled_write_byte(0xc8, OLED_CMD);    //Set COM Output Scan Direction
    oled_write_byte(0x00, OLED_CMD);    //---set low column address
    oled_write_byte(0x10, OLED_CMD);    //---set high column address
    oled_write_byte(0x40, OLED_CMD);    //--set start line address
    oled_write_byte(0x81, OLED_CMD);    //--set contrast control register
    oled_write_byte(0xff, OLED_CMD);    //brightness 0x00~0xff
    oled_write_byte(0xa1, OLED_CMD);    //--set segment re-map 0 to 127
    oled_write_byte(0xa6, OLED_CMD);    //--set normal display
    oled_write_byte(0xa8, OLED_CMD);    //--set multiplex ratio(1 to 64)
    oled_write_byte(0x3F, OLED_CMD);    //
    oled_write_byte(0xa4, OLED_CMD);    //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    oled_write_byte(0xd3, OLED_CMD);    //-set display offset
    oled_write_byte(0x00, OLED_CMD);    //-not offset
    oled_write_byte(0xd5, OLED_CMD);    //--set display clock divide ratio/oscillator frequency
    oled_write_byte(0xf0, OLED_CMD);    //--set divide ratio
    oled_write_byte(0xd9, OLED_CMD);    //--set pre-charge period
    oled_write_byte(0x22, OLED_CMD);    //
    oled_write_byte(0xda, OLED_CMD);    //--set com pins hardware configuration
    oled_write_byte(0x12, OLED_CMD);
    oled_write_byte(0xdb, OLED_CMD);    //--set vcomh
    oled_write_byte(0x20, OLED_CMD);    //0x20,0.77xVcc
    oled_write_byte(0x8d, OLED_CMD);    //--set DC-DC enable
    oled_write_byte(0x14, OLED_CMD);    //
    oled_write_byte(0xaf, OLED_CMD);    //--turn on oled panel
}
void OLED_display_on(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x14, OLED_CMD);
    oled_write_byte(0xaf, OLED_CMD);
}
void OLED_display_off(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x10, OLED_CMD);
    oled_write_byte(0xae, OLED_CMD);
}
//操作GRAM内存(128*8char数组)
 void OLED_operate_gram(pen_typedef pen)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        for (n = 0; n < 128; n++)
        {
            if (pen == PEN_WRITE)
            {
                OLED_GRAM[n][i] = 0xff;
            }
            else if (pen == PEN_CLEAR)
            {
                OLED_GRAM[n][i] = 0x00;
            }
            else
            {
                OLED_GRAM[n][i] = 0xff - OLED_GRAM[n][i];
            }
        }
    }
}
void OLED_set_pos(uint8_t x, uint8_t y)
{
    oled_write_byte((0xb0 + y), OLED_CMD);              //set page address y
    oled_write_byte(((x&0xf0)>>4)|0x10, OLED_CMD);      //set column high address
    oled_write_byte((x&0x0f), OLED_CMD);                //set column low address
}
void OLED_draw_point(int8_t x, int8_t y, pen_typedef pen)
{
    uint8_t page = 0, row = 0;

    /* check the corrdinate */
    if ((x < 0) || (x > (X_WIDTH - 1)) || (y < 0) || (y > (Y_WIDTH - 1)))
    {
        return;
    }
    page = y / 8;
    row = y % 8;

    if (pen == PEN_WRITE)
    {
        OLED_GRAM[x][page] |= 1 << row;
    }
    else if (pen == PEN_INVERSION)
    {
        OLED_GRAM[x][page] ^= 1 << row;
    }
    else
    {
        OLED_GRAM[x][page] &= ~(1 << row);
    }
}
void OLED_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, pen_typedef pen)
{
    uint8_t col = 0, row = 0;
    uint8_t x_st = 0, x_ed = 0, y_st = 0, y_ed = 0;
    float k = 0.0f, b = 0.0f;

    if (y1 == y2)
    {
        (x1 <= x2) ? (x_st = x1):(x_st = x2);
        (x1 <= x2) ? (x_ed = x2):(x_ed = x1);

        for (col = x_st; col <= x_ed; col++)
        {
            OLED_draw_point(col, y1, pen);
        }
    }
    else if (x1 == x2)
    {
        (y1 <= y2) ? (y_st = y1):(y_st = y2);
        (y1 <= y2) ? (y_ed = y2):(y_ed = y1);

        for (row = y_st; row <= y_ed; row++)
        {
            OLED_draw_point(x1, row, pen);
        }
    }
    else
    {
        k = ((float)(y2 - y1)) / (x2 - x1);
        b = (float)y1 - k * x1;

        (x1 <= x2) ? (x_st = x1):(x_st = x2);
        (x1 <= x2) ? (x_ed = x2):(x_ed = x2);

        for (col = x_st; col <= x_ed; col++)
        {
            OLED_draw_point(col, (uint8_t)(col * k + b), pen);
        }
    }
}
void OLED_show_char(uint8_t row, uint8_t col, uint8_t chr)
{
    uint8_t x = col * 6;
    uint8_t y = row * 12;
    uint8_t temp, t, t1;
    uint8_t y0 = y;
    chr = chr - ' ';

    for (t = 0; t < 12; t++)
    {
        temp = asc2_1206[chr][t];

        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp&0x80)
                OLED_draw_point(x, y, PEN_WRITE);
            else
                OLED_draw_point(x, y, PEN_CLEAR);

            temp <<= 1;
            y++;
            if ((y - y0) == 12)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

void OLED_show_string(uint8_t row, uint8_t col, uint8_t *chr)
{
    uint8_t n =0;

    while (chr[n] != '\0')
    {
        OLED_show_char(row, col, chr[n]);
        col++;

        if (col > 20)
        {
            col = 0;
            row += 1;
        }
        n++;
    }
}
//格式输出
 void OLED_printf(uint8_t row, uint8_t col, const char *fmt,...)
{
    static uint8_t LCD_BUF[128] = {0};
    static va_list ap;
    uint8_t remain_size = 0;

    if ((row > 4) || (col > 20) )
    {
        return;
    }
    va_start(ap, fmt);

    vsprintf((char *)LCD_BUF, fmt, ap);

    va_end(ap);

    remain_size = 21 - col;

    LCD_BUF[remain_size] = '\0';

    OLED_show_string(row, col, LCD_BUF);
}

//发送数据到OLED的GRAM
  
void OLED_refresh_gram(void)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        OLED_set_pos(0, i);
        for (n = 0; n < 128; n++)
        {
            oled_write_byte(OLED_GRAM[n][i], OLED_DATA);
        }
    }
}



/*********************************************************************************
 * 函数功能： 屏幕显示
 * 入口参数： 无
 * 返回值：   无
**********************************************************************************/
void OLED_show(void)
{
  OLED_operate_gram(PEN_CLEAR);
  OLED_printf(0,0,"encoderL: %d",enc1);	
  OLED_printf(1,0,"encoderR: %d",enc2);	
  OLED_printf(2,0,"Cx: %d,Cy: %d",Cx,Cy);
  OLED_printf(3,0,"pitch: %f",pitch);
//   OLED_printf(4,0,"Cw: %d,Ch: %d",Cw,Ch);
  OLED_printf(4,0,"Light_Sens: %d",Light_Sens);
  OLED_refresh_gram();	//oled
}











