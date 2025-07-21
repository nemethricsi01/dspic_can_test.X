#include "display.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include "../gpio_driver/gpio.h"
#include "../spi_driver/spi.h"
#include "../delay.h"

uint8_t   lcd_buffer[80];//st7036 has 80 bytes of data ram



void lcd_command_data_set(void)
{
    LATBbits.LATB7 = 1;
}
void lcd_command_data_reset(void)
{
    LATBbits.LATB7 = 0;
}
void lcd_reset_set(void)
{
    LATBbits.LATB8 = 1;
}
void lcd_reset_reset(void)
{
    LATBbits.LATB8 = 0;
}


static void lcd_send_command(uint8_t command) 
{

    lcd_command_data_reset();
    for(int j = 0; j < 8; j++)
        {
            LATBbits.LATB6 = 0; // Clock low (idle)
            if(command & (1 << (7 - j)))
            {
                LATBbits.LATB5 = 1;
            }
            else
            {
                LATBbits.LATB5 = 0;
            }
            __delay_us(2);
            LATBbits.LATB6 = 1; // Clock high (active)
            __delay_us(2);
        }
        LATBbits.LATB6 = 0; // Ensure clock ends low
        __delay_us(10);
}
static void lcd_send_data(void) 
{
    lcd_command_data_set();
    for(int i = 0; i < sizeof(lcd_buffer); i++) 
    {
     for(int j = 0; j < 8; j++)
        {
            LATBbits.LATB6 = 0; // Clock low (idle)
            if(lcd_buffer[i] & (1 << (7 - j)))
            {
                LATBbits.LATB5 = 1;
            }
            else
            {
                LATBbits.LATB5 = 0;
            }
            __delay32(2);
            LATBbits.LATB6 = 1; // Clock high (active)
            __delay32(2);            
        }
        LATBbits.LATB6 = 0; // Ensure clock ends low
    }
}

static void lcd_init_(void) 
{
    lcd_reset_reset();
    __delay_ms(100);
    lcd_reset_set();
    __delay_ms(100);
    lcd_reset_reset();
    __delay_ms(100);
    lcd_reset_set();
    __delay_ms(100);
    lcd_send_command(0x38);
    __delay_ms(1);
    lcd_send_command(0x39);
    __delay_ms(1);
    lcd_send_command(0x14);
    __delay_ms(1);
    lcd_send_command(0x78);
    __delay_ms(1);
    lcd_send_command(0x5e);
    __delay_ms(1);
    lcd_send_command(0x6a);
    __delay_ms(250);
    lcd_send_command(0x0c);
    __delay_ms(1);
    lcd_send_command(0x01);
    __delay_ms(3);
    lcd_send_command(0x06);
    __delay_ms(100);
}

static void lcd_set_backlight(uint8_t brightness) 
{//4 levels of brightness
    // Set the backlight brightness
    // ...
}


void Display_Init(Display* display) 
{
    for (int i = 0; i < MAX_LINES; i++) 
    {
        memset(display->buffer[i], ' ', MAX_CHARS);
        display->buffer[i][MAX_CHARS] = '\0'; // null-terminate the string
    }
    lcd_init_();
}

void Display_Printf(Display* display, int line, const char* format, ...) 
{
    if (line < MAX_LINES) 
    {
        va_list args;
        va_start(args, format);
        vsnprintf(display->buffer[line], MAX_CHARS + 1, format, args);
        va_end(args);
    }
}

void Display_Send(Display* display) 
{
    memset(lcd_buffer,'a',80);
    for(int i = 0;i<16;i++)
    {
        
        lcd_buffer[i] = display->buffer[0][i];
    }
    for(int i = 0;i<16;i++)
    {
        
        lcd_buffer[i+40] = display->buffer[1][i];
    }
    lcd_send_data();
}