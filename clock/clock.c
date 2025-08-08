#include "clock.h"


volatile clock_time_t system_clock = {0, 0, 0, 0};
volatile clock_date_t system_date = {0,0,0};

static volatile uint32_t one_second_ticker;
volatile uint8_t one_second_flag;
volatile uint64_t one_millisecond_ticker = 0;

void clock_init(void) {
    system_clock.hours = 0;
    system_clock.minutes = 0;
    system_clock.seconds = 0;
    system_clock.milliseconds = 0;
    system_date.days = 0;
    system_date.months = 0;
    system_date.years = 0ul;
}

void clock_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds) {
    system_clock.hours = hours;
    system_clock.minutes = minutes;
    system_clock.seconds = seconds;
    system_clock.milliseconds = 0;
}
void clock_set_date(uint16_t years, uint8_t months, uint8_t days)
{
    system_date.days = days;
    system_date.months = months;
    system_date.years = years;
}
void clock_tick(void) {
    one_second_ticker++;
    one_millisecond_ticker++;
    if (one_second_ticker >= 1000) {
        one_second_ticker = 0;
        one_second_flag = 1; // Set flag to indicate a second has passed
    }
    system_clock.milliseconds++;
    
    if (system_clock.milliseconds >= 1000) {
        system_clock.milliseconds = 0;
        system_clock.seconds++;
        
        if (system_clock.seconds >= 60) {
            system_clock.seconds = 0;
            system_clock.minutes++;
            
            if (system_clock.minutes >= 60) {
                system_clock.minutes = 0;
                system_clock.hours++;
                
                if (system_clock.hours >= 24) {
                    system_clock.hours = 0;
                }
            }
        }
    }
}

clock_time_t clock_get_time(void) 
{
    return system_clock;
}
clock_date_t clock_get_date(void)
{
    return system_date;
}