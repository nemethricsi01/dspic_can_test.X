#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdarg.h>
#include <stdint.h>

#define MAX_LINES 2
#define MAX_CHARS 16

typedef struct {
    char buffer[MAX_LINES][MAX_CHARS + 1]; // +1 for null-terminator
} Display;
void lcd_command_data_set(void);
void lcd_command_data_reset(void);
void lcd_reset_set(void);
void lcd_reset_reset(void);
void Display_Init(Display* display);
void Display_Printf(Display* display, int line, const char* format, ...);
void Display_Clear(Display* display, uint8_t line);
void Display_Send(Display* display);

#endif // DISPLAY_H