#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

void timer_4_init(void);
void timer_4_start(void);
void timer_4_stop(void);
void timer_4_set_period(uint16_t period);
void timer_5_init(void);

#endif // TIMER_H