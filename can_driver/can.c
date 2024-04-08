#include "can.h"
unsigned int canTxBuff[4][8]__attribute__((aligned(4 * 16)));
unsigned int canRxBuff[32][8]__attribute__((aligned(32 * 16)));





void can_init(void)
{
C1CTRL1bits.REQOP = 0b100;//set config mode
while(C1CTRL1bits.OPMODE != 0b100);
C1CTRL1bits.WIN = 0;

C1CFG1 = 0x3f;//2*64, 1 jump width
C1CFG2bits.PRSEG = 0b01;//propagation segment 2xTq
C1CFG2bits.SEG1PH = 0b110;// 7xTq
C1CFG2bits.SEG2PH = 0b101;// 6xTq
C1CFG2bits.SEG2PHTS = 1;//freely selectable
C1FCTRL = 0xC01F;       // No FIFO, 32 Buffers

DMA2CONbits.SIZE = 0x0;
DMA2CONbits.DIR = 0x1;
DMA2CONbits.AMODE = 0x2;
DMA2CONbits.MODE = 0x0;
DMA2REQ = 70;
DMA2CNT = 7;
DMA2PAD = (volatile unsigned int)&C1TXD;
DMA2STAL = (unsigned int) &canTxBuff;
DMA2STAH = (unsigned int) &canTxBuff;

C1TR01CONbits.TXEN0 = 0x1;
C1TR01CONbits.TX0PRI = 0x3;
DMA2CONbits.CHEN = 0x1;

/* setup channel 2 for peripheral indirect addressing mode
    normal operation, word operation and select as Rx to peripheral */
    DMA3CON = 0x0020;
    /* setup the address of the peripheral ECAN1 (C1RXD) */
	DMA3PAD = (volatile unsigned int)&C1RXD;
 	/* Set the data block transfer size of 8 */
 	DMA3CNT = 7;
 	/* automatic DMA Rx initiation by DMA request */
	DMA3REQ = 0x0022;
	/* start adddress offset value */
	DMA3STAL=(unsigned int)(&canRxBuff);
    DMA3STAH=(unsigned int)(&canRxBuff);
	/* enable the channel */
	DMA3CONbits.CHEN=1;
    
    
    
    /* 4 CAN Messages to be buffered in DMA RAM */
	C1FCTRLbits.DMABS=0b000;

    /* Filter configuration */
	/* enable window to access the filter configuration registers */
	C1CTRL1bits.WIN = 0b1;
	/* select acceptance mask 0 filter 0 buffer 1 */
	C1FMSKSEL1bits.F0MSK = 0;

    /* setup the mask to check every bit of the standard message, the macro when called as */
    /* CAN_FILTERMASK2REG_SID(0x7FF) will write the register C1RXM0SID to include every bit in */
    /* filter comparison */
    C1RXM0SIDbits.EID = 0;
    C1RXM0SIDbits.MIDE = 0;
    C1RXM0SIDbits.SID  = 0;
    
    
    
	/* configure accpetence filter 0
	setup the filter to accept a standard id of 0x123,
	the macro when called as CAN_FILTERMASK2REG_SID(0x123) will
	write the register C1RXF0SID to accept only standard id of 0x123
	*/
	C1RXF0SIDbits.SID = 0;
    C1RXF0SIDbits.EID = 0;
	/* acceptance filter to use buffer 1 for incoming messages */
	C1BUFPNT1bits.F0BP = 0b0001;
	/* enable filter 0 */
	C1FEN1bits.FLTEN0 = 1;
    /* clear window bit to access ECAN control registers */
	C1CTRL1bits.WIN = 0;

    /* ECAN1, Buffer 1 is a Receive Buffer */
	C1TR01CONbits.TXEN1 = 0;

    /* clear the buffer and overflow flags */
	C1RXFUL1=C1RXFUL2=C1RXOVF1=C1RXOVF2=0x0000;


C1CTRL1bits.REQOP = 0;
while(C1CTRL1bits.OPMODE != 0);
C1INTEbits.RBIE = 1;
    IEC2bits.C1IE = 1;
}
void can_send(uint16_t id, uint8_t *data, uint8_t dlc)
{

}
void can_get_data_frombuff(uint8_t *data, uint8_t *dlc)
{
    
}