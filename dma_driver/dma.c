#include "dma.h"
#include <xc.h>


void dma_init(void)
{
    DMA0CONbits.SIZE        = 1; // Set the size of the data to be transferred (1 = byte)
    DMA0CONbits.AMODE       = 0b00; // Set the addressing mode (00 = Register Indirect with Post-Increment)
    DMA0CONbits.DIR         = 1; // Set the direction of the transfer (1 = Read from DMA RAM address, write to peripheral)
    DMA0CONbits.MODE        = 1; // Set the operating mode (1 = One-shot, Ping-Pong modes disabled)
    DMA0STAL                = 0x0000; // Set the lower bits of the starting address of the DMA RAM
    DMA0STAH                = 0x0000; // Set the upper bits of the starting address of the DMA RAM
    DMA0PAD                 = (volatile unsigned int) &SPI1BUF; // Set the address of the peripheral to be associated with DMA Channel 0
    DMA0CNT                 = 0; // Set the DMA transfer count (number of DMA transfers per DMA event)
    DMA0REQ                 = 0x000A; // Set the DMA Channel 0 IRQ Select bits (000A = SPI1)
    DMA0CONbits.CHEN        = 1; // Enable the DMA channel (1 = DMA Channel 0 is enabled)
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
    DMA1CONbits.CHEN        = 1;

    // Force DMA transfer
    DMA1REQbits.FORCE       = 1;
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
    DMA1REQ                 = 0x000A; // Set the DMA Channel 0 IRQ Select bits (000A = SPI1)
    DMA1CONbits.CHEN        = 1; // Enable the DMA channel (1 = DMA Channel 0 is enabled)
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