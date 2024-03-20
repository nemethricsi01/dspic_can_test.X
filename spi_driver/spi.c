#include "spi.h"
#include "xc.h"
#include "../delay.h"

void spi_init(void)
{
    SPI1CON1bits.MSTEN          = 1; // Set master mode
    SPI1CON1bits.SPRE           = 0b101; // Set secondary prescale to 3:1
    SPI1CON1bits.CKE            = 0; // Set clock edge select (0 = data changes on clock transition from active to idle)
    SPI1CON1bits.SMP            = 0; // Set input data sample phase (0 = input data sampled at middle of data output time)
    SPI1CON1bits.CKP            = 0; // Set clock polarity (0 = idle state for clock is a low level)
    SPI1CON1bits.PPRE           = 0b11; // Set primary prescale to 1:1
    SPI1CON1bits.MODE16         = 0; // Set word/byte mode (0 = byte mode)
    SPI1CON2bits.SPIBEN         = 1; // Enable enhanced buffer 
    SPI1CON2bits.FRMEN          = 0; // Disable framed mode
    SPI1STATbits.SISEL          = 0b100; // Set interrupt mode (100 = interrupt when last word is shifted out of SPIxSR and the transmit is complete)
    /*
    enhanced buffer mode is important because without it the interrupt after one transfer will not be generated(BUG?!?!?!?!)
    SISEL is set to 100 because we want to generate an interrupt when the last word is shifted out of the SPIxSR and the transmit is complete
    */
}
void spi_enable(void)
{
IFS0bits.SPI1IF = 0; // Clear the Interrupt flag
IEC0bits.SPI1IE = 1; // Enable the interrupt
SPI1STATbits.SPIEN = 1;// Enable SPI1 module
}

void spi2_init(void)
{
    SPI2CON1bits.MSTEN          = 1; // Set master mode
    SPI2CON1bits.SPRE           = 0b000; // Set secondary prescale to 3:1
    SPI2CON1bits.CKE            = 1; // Set clock edge select (0 = data changes on clock transition from active to idle)
    SPI2CON1bits.SMP            = 0; // Set input data sample phase (0 = input data sampled at middle of data output time)
    SPI2CON1bits.CKP            = 0; // Set clock polarity (0 = idle state for clock is a low level)
    SPI2CON1bits.PPRE           = 0b01; // Set primary prescale to 1:1
    SPI2CON1bits.MODE16         = 0; // Set word/byte mode (0 = byte mode)
    SPI2CON2bits.SPIBEN         = 1; // Enable enhanced buffer 
    SPI2CON2bits.FRMEN          = 0; // Disable framed mode
    SPI2STATbits.SISEL          = 0b100; // Set interrupt mode (100 = interrupt when last word is shifted out of SPIxSR and the transmit is complete)
    /*
    enhanced buffer mode is important because without it the interrupt after one transfer will not be generated(BUG?!?!?!?!)
    SISEL is set to 100 because we want to generate an interrupt when the last word is shifted out of the SPIxSR and the transmit is complete
    */
}

void spi2_enable(void)
{
//    IFS2bits.SPI2IF = 0; // Clear the Interrupt flag
//    IEC2bits.SPI2IE = 1; // Enable the interrupt
    SPI2STATbits.SPIEN = 1;// Enable SPI1 module
}

void spi2_send(uint8_t data)
{
    uint8_t  temp;
    SPI2BUF = data; // Write the data to the SPI buffer
    
}
void __attribute__((__interrupt__, no_auto_psv)) _SPI2Interrupt (void)
{

    if(IFS2bits.SPI2IF)
    {
        IFS2bits.SPI2IF = 0;
    }
    
}