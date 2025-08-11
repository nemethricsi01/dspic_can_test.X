#include "clock.h"


volatile clock_time_t system_clock = {0, 0, 0, 0};
volatile clock_date_t system_date = {0,0,0};

static volatile uint32_t one_second_ticker;
volatile uint8_t one_second_flag;
volatile uint64_t one_millisecond_ticker = 0;

static uint8_t is_leap_year(uint16_t year)
{
    // Gregorian leap year rule
    return ((year % 400u) == 0u) || (((year % 4u) == 0u) && ((year % 100u) != 0u));
}

static uint8_t days_in_month(uint16_t year, uint8_t month)
{
    static const uint8_t mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (month == 0u || month > 12u) return 31u;
    if (month == 2u) return (uint8_t)(is_leap_year(year) ? 29u : 28u);
    return mdays[month - 1u];
}

static void date_tick(void)
{
    uint16_t year = system_date.years;
    uint8_t  month = system_date.months;
    uint8_t  day = system_date.days;

    // Normalize if uninitialized (0)
    if (month < 1u) month = 1u;
    if (day   < 1u) day   = 1u;

    day++;
    uint8_t dim = days_in_month(year, month);
    if (day > dim) {
        day = 1u;
        month++;
        if (month > 12u) {
            month = 1u;
            year++;
        }
    }

    system_date.years = year;
    system_date.months = month;
    system_date.days = day;
}

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
                    // Increment date at midnight
                    date_tick();
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