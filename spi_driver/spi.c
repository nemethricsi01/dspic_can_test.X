#include "spi.h"
#include "xc.h"
#include "../delay.h"

uint8_t dummybuff[2];
void dma_init(void)
{
    DMA0CONbits.SIZE        = 1; // Set the size of the data to be transferred (1 = byte)
    DMA0CONbits.AMODE       = 0b00; // Set the addressing mode (00 = Register Indirect with Post-Increment)
    DMA0CONbits.DIR         = 1; // Set the direction of the transfer (1 = Read from DMA RAM address, write to peripheral)
    DMA0CONbits.MODE        = 1; // Set the operating mode (1 = One-shot, Ping-Pong modes disabled)
    DMA0STAL                =  __builtin_dmaoffset(dummybuff); // Set the lower bits of the starting address of the DMA RAM
    DMA0STAH                = 0x0000; // Set the upper bits of the starting address of the DMA RAM
    DMA0PAD                 = (volatile unsigned int) &SPI1BUF; // Set the address of the peripheral to be associated with DMA Channel 0
    DMA0CNT                 = 0; // Set the DMA transfer count (number of DMA transfers per DMA event)
    DMA0REQ                 = 0x000A; // Set the DMA Channel 0 IRQ Select bits (000A = SPI1)
    DMA0CONbits.CHEN        = 1; // Enable the DMA channel (1 = DMA Channel 0 is enabled)
    IEC0bits.DMA0IE         = 1; 
}

void dma_set_buffer(uint8_t *buffer, uint16_t length)
{
    
    DMA0STAL                = (uint16_t)buffer;// Set the lower bits of the starting address of the DMA RAM to the buffer address
    DMA0STAH                = 0x0000; // Set the upper bits of the starting address of the DMA RAM to 0x0000
    DMA0CNT                 = length - 1; // Set the DMA transfer count to length - 1
}
void dma_start(void)
{
    // Enable DMA channel
    DMA0CONbits.CHEN        = 1;

    // Force DMA transfer
    DMA0REQbits.FORCE       = 1;
}


void dma1_init(void)
{
    DMA1CONbits.SIZE        = 1; // Set the size of the data to be transferred (1 = byte)
    DMA1CONbits.AMODE       = 0b00; // Set the addressing mode (00 = Register Indirect with Post-Increment)
    DMA1CONbits.DIR         = 1; // Set the direction of the transfer (1 = Read from DMA RAM address, write to peripheral)
    DMA1CONbits.MODE        = 1; // Set the operating mode (1 = One-shot, Ping-Pong modes disabled)
    DMA1STAL                = 0x0000; // Set the lower bits of the starting address of the DMA RAM
    DMA1STAH                = 0x0000; // Set the upper bits of the starting address of the DMA RAM
    DMA1PAD                 = (volatile unsigned int) &SPI2BUF; // Set the address of the peripheral to be associated with DMA Channel 0
    DMA1CNT                 = 0; // Set the DMA transfer count (number of DMA transfers per DMA event)
    DMA1REQ                 = 0x0021; // Set the DMA Channel 0 IRQ Select bits (0021 = SPI2)
    DMA1CONbits.CHEN        = 1; // Enable the DMA channel (1 = DMA Channel 0 is enabled)
    IEC0bits.DMA1IE         = 1;
}

void dma1_set_buffer(uint8_t *buffer, uint16_t length)
{
    
    DMA1STAL                = (uint16_t)buffer;// Set the lower bits of the starting address of the DMA RAM to the buffer address
    DMA1STAH                = 0x0000; // Set the upper bits of the starting address of the DMA RAM to 0x0000
    DMA1CNT                 = length - 1; // Set the DMA transfer count to length - 1
}
void dma1_start(void)
{
    // Enable DMA channel
    DMA1CONbits.CHEN        = 1;

    // Force DMA transfer
    DMA1REQbits.FORCE       = 1;
}


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
//IFS0bits.SPI1IF = 0; // Clear the Interrupt flag
//IEC0bits.SPI1IE = 1; // Enable the interrupt
SPI1STATbits.SPIEN = 1;// Enable SPI1 module
}

void spi2_init(void)
{
    SPI2CON1bits.MSTEN          = 1; // Set master mode
    SPI2CON1bits.SPRE           = 0b101; // Set secondary prescale to 3:1
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
