#include "interrupts.h"
#include <xc.h>
void __attribute__((__interrupt__, no_auto_psv)) _T4Interrupt(void) {
    //LATBbits.LATB1^= 1;
    TMR4 = 0;
    IFS1bits.T4IF = 0;// Clear Timer 4 Interrupt Flag
    // Add your code here
}
void __attribute__((__interrupt__, no_auto_psv)) _T5Interrupt(void) {
    LATBbits.LATB1^= 1;
    TMR5 = 0;
    IFS1bits.T5IF = 0;// Clear Timer 5 Interrupt Flag
    // Add your code here
}

void __attribute__((__interrupt__, no_auto_psv)) _SPI2Interrupt (void)
{

    if(IFS2bits.SPI2IF)
    {
        
        IFS2bits.SPI2IF = 0;
    }
    
}
void __attribute__((__interrupt__, no_auto_psv)) _SPI1Interrupt(void)
{

    if(IFS0bits.SPI1IF == 1)
    {
        
        IFS0bits.SPI1IF = 0;
    }
    
}