#ifndef GPIO_H
#define GPIO_H
#include <xc.h>
#include <stdint.h>

void gpio_init(void);

void lcd_command_data_set(void);
void lcd_command_data_reset(void);
void lcd_reset_set(void);
void lcd_reset_reset(void);

#endif // GPIO_H
