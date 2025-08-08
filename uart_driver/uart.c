#include "uart.h"

#define FP 20000000
#define BAUDRATE 115200
#define BRGVAL ((FP/BAUDRATE)/4) - 1

void uart_init(void)
{
    U1MODEbits.BRGH     =   1;//high speed mode
    U1STAbits.UTXEN     =   0;//transmit disabled
    
    U1BRG               =   43; // Baud Rate setting for 115200
    U1MODEbits.UARTEN   =   1;// uart enabled
    
    IFS0bits.U1RXIF     =   0;//clear any pending interrupt
    IEC0bits.U1RXIE     =   1;//enable rx interrupt
    
    IPC2bits.U1RXIP     =   3;//add low priority for the interrupt
    
}