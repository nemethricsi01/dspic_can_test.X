#include "display.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include "../gpio_driver/gpio.h"
#include "../spi_driver/spi.h"
#include "../delay.h"
#include "../dma_driver/dma.h"

uint8_t   lcd_buffer[MAX_CHARS * MAX_LINES];

static uint8_t reverse(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

static void lcd_send_data(uint8_t data) {
    uint8_t temp_data = reverse(data);
    lcd_command_data_set();
    spi2_send(data);

}
static void lcd_send_command(uint8_t command) {
uint8_t temp_command = reverse(command);
    lcd_command_data_reset();
    spi2_send(command);
}
static void lcd_send_data_dma(void) {
    lcd_command_data_set();
    dma_start();
}

static void lcd_init_(void) {
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
    lcd_send_data('a');
}

static void lcd_set_backlight(uint8_t brightness) {//4 levels of brightness
    // Set the backlight brightness
    // ...
}


void Display_Init(Display* display) {
    for (int i = 0; i < MAX_LINES; i++) {
        memset(display->buffer[i], ' ', MAX_CHARS);
        display->buffer[i][MAX_CHARS] = '\0'; // null-terminate the string
    }
    spi2_init();
    spi2_enable();
    lcd_init_();
//dma1_init();
//    dma1_set_buffer(lcd_buffer,32);

}

void Display_Printf(Display* display, int line, const char* format, ...) {
    if (line < MAX_LINES) {
        va_list args;
        va_start(args, format);
        vsnprintf(display->buffer[line], MAX_CHARS + 1, format, args);
        va_end(args);
    }
}

void Display_Send(Display* display) {
    // Here you would put your SPI send code.
    // For now, we'll just print the buffer to the console.
    uint8_t ptr = 0;
    for(int i = 0;i<MAX_LINES*MAX_CHARS+1;i++)
    {
        if(display->buffer[0][i] == 0)
        {
            i++;
        }
        lcd_buffer[ptr] = display->buffer[0][i];
        ptr++;
    }
    lcd_send_data_dma();
    
    
}