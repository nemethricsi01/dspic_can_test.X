/* 
 * File:   clock.h
 * Author: nemet
 *
 * Created on 2025. j�lius 24., 23:29
 */

#ifndef CLOCK_H
#define	CLOCK_H
#include <xc.h>
#include <stdint.h>

typedef struct {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint16_t milliseconds;
} clock_time_t;
typedef struct {
    uint8_t days;
    uint8_t months;
    uint16_t years;
} clock_date_t;

extern volatile uint64_t one_millisecond_ticker;

extern volatile uint8_t one_second_flag;

extern volatile clock_time_t system_clock;
extern volatile clock_date_t system_date;

void clock_init(void);
void clock_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds);
void clock_tick(void);  // This is called from the 1ms timer interrupt
clock_time_t clock_get_time(void);


void clock_set_date(uint16_t years, uint8_t months, uint8_t days);
clock_date_t clock_get_date(void);

#endif	/* CLOCK_H */

