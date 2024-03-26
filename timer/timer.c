#include "timer.h"
#include <xc.h>

void timer_4_init(void)
{
    T4CONbits.TON = 0; // Disable Timer
    T4CONbits.TCS = 0; // Select internal instruction cycle clock
    T4CONbits.TGATE = 0; // Disable Gated Timer mode
    T4CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    T4CONbits.T32 = 0;// 16-bit timer
    TMR4 = 0x00; // Clear timer register
    PR4 = 0xFFFF; // Load the period value
    IPC6bits.T4IP = 0x05; // Set Timer 4 Interrupt Priority Level
    IFS1bits.T4IF = 0; // Clear Timer 4 Interrupt Flag
    IEC1bits.T4IE = 1; // Enable Timer 4 interrupt
    T4CONbits.TON = 1; // Start Timer

}
void timer_4_start(void);
void timer_4_stop(void);
void timer_4_set_period(uint16_t period);
