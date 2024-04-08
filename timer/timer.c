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
void timer_4_start(void)
{
    T4CONbits.TON = 1;
}
void timer_4_stop(void)
{
    T4CONbits.TON = 0;
    TMR4 = 0;
}
void timer_4_set_period(uint16_t period)
{
    PR4 = period;
}

void timer_5_init(void)
{
    T5CONbits.TON = 0; // Disable Timer
    T5CONbits.TCS = 0; // Select internal instruction cycle clock
    T5CONbits.TGATE = 0; // Disable Gated Timer mode
    T5CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    TMR5 = 0x00; // Clear timer register
    PR5 = 0x7FFF; // Load the period value
    IPC7bits.T5IP = 0x04; // Set Timer 5 Interrupt Priority Level
    IFS1bits.T5IF = 0; // Clear Timer 4 Interrupt Flag
    IEC1bits.T5IE = 1; // Enable Timer 4 interrupt
    T5CONbits.TON = 1; // Start Timer

}
void timer_5_start(void)
{
    T5CONbits.TON = 1;
}
void timer_5_stop(void)
{
    T5CONbits.TON = 0;
    TMR5 = 0;
}
void timer_5_set_period(uint16_t period)
{
    PR5 = period;
}