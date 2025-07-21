#ifndef GPIO_H
#define GPIO_H
#include <xc.h>
#include <stdint.h>
#include <string.h>
#include "../delay.h"
#define NUM_ROWS 8
#define NUM_COLS 3
#define NUM_BUTTONS 24


void gpio_init(void);
void read_buttons(uint8_t *buttonarray);


#endif // GPIO_H
