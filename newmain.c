/*
 * File:   newmain.c
 * Author: nemet
 *
 * Created on 2023. június 29., 1:00
 */


// DSPIC33EV256GM102 Configuration Bit Settings

// 'C' source line config statements

// FSEC
#pragma config BWRP = OFF               // Boot Segment Write-Protect Bit (Boot Segment may be written)
#pragma config BSS = DISABLED           // Boot Segment Code-Protect Level bits (No Protection (other than BWRP))
#pragma config BSS2 = OFF               // Boot Segment Control Bit (No Boot Segment)
#pragma config GWRP = OFF               // General Segment Write-Protect Bit (General Segment may be written)
#pragma config GSS = DISABLED           // General Segment Code-Protect Level bits (No Protection (other than GWRP))
#pragma config CWRP = OFF               // Configuration Segment Write-Protect Bit (Configuration Segment may be written)
#pragma config CSS = DISABLED           // Configuration Segment Code-Protect Level bits (No Protection (other than CWRP))
#pragma config AIVTDIS = DISABLE        // Alternate Interrupt Vector Table Disable Bit  (Disable Alternate Vector Table)

// FBSLIM
#pragma config BSLIM = 0x1FFF           // Boot Segment Code Flash Page Address Limit Bits (Enter Hexadecimal value)

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Initial oscillator Source Selection Bits (Primary Oscillator with PLL module (XT + PLL, HS + PLL, EC + PLL))
#pragma config IESO = OFF                // Two Speed Oscillator Start-Up Bit (Start up device with FRC,then automatically switch to user selected oscillator source)

// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Mode Select Bits (HS Crystal Oscillator mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin I/O Function Enable Bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration Bit (Allow Multiple reconfigurations)
#pragma config FCKSM = CSECME           // Clock Switching Mode Bits (Both Clock Switching and Fail-safe Clock Monitor are disabled)
#pragma config PLLKEN = ON              // PLL Lock Enable Bit (Clock switch to PLL source will wait until the PLL lock signal is valid)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler Bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler Bit (1:128)
#pragma config FWDTEN = ON_SWDTEN       // Watchdog Timer Enable Bits (WDT Enabled/Disabled (controlled using SWDTEN bit))
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable Bit (Watchdog timer in Non-Window Mode)
#pragma config WDTWIN = WIN25           // Watchdog Window Select Bits (WDT Window is 25% of WDT period)

// FPOR
#pragma config BOREN0 = OFF              // Brown Out Reset Detection Bit (BOR is Enabled)

// FICD
#pragma config ICS = PGD2               // ICD Communication Channel Select Bits (Communicate on PGEC2 and PGED2)

// FDMTINTVL
#pragma config DMTIVTL = 0xFFFF         // Lower 16 Bits of 32 Bit DMT Window Interval (Enter Hexadecimal value)

// FDMTINTVH
#pragma config DMTIVTH = 0xFFFF         // Upper 16 Bits of 32 Bit DMT Window Interval (Enter Hexadecimal value)

// FDMTCNTL
#pragma config DMTCNTL = 0xFFFF         // Lower 16 Bits of 32 Bit DMT Instruction Count Time-Out Value (Enter Hexadecimal value)

// FDMTCNTH
#pragma config DMTCNTH = 0xFFFF         // Upper 16 Bits of 32 Bit DMT Instruction Count Time-Out Value (Enter Hexadecimal value)

// FDMT
#pragma config DMTEN = ENABLE           // Dead Man Timer Enable Bit (Dead Man Timer is Enabled and cannot be disabled by software)

// FDEVOPT
#pragma config PWMLOCK = ON             // PWM Lock Enable Bit (Certain PWM registers may only be written after key sequence)
#pragma config ALTI2C1 = OFF            // Alternate I2C1 Pins Selection Bit (I2C1 mapped to SDA1/SCL1 pins)

// FALTREG
#pragma config CTXT1 = NONE             // Interrupt Priority Level (IPL) Selection Bits For Alternate Working Register Set 1 (Not Assigned)
#pragma config CTXT2 = NONE             // Interrupt Priority Level (IPL) Selection Bits For Alternate Working Register Set 2 (Not Assigned)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define FCY 20000000//40Mhz
#include "libpic30.h"
unsigned int ecan1MsgBuf[4][8]__attribute__((aligned(4 * 16)));

uint8_t ledbuff[25];

//uint32_t ledbuff[4];

void writeLed(uint32_t color)
{
    LATBbits.LATB3 = 1;
    int i;
    for( i = 23;i>=0;i--)
    {
       if(color&1)
       {
           ledbuff[i] = 0xfc;
       }
       else
       {
           ledbuff[i] = 0x80;
       }
       color = color/2;
    }
    ledbuff[24] = 0x80;
    LATBbits.LATB3 = 0;
    for(i = 0;i< 8;i++)
    {
        SPI1BUF = ledbuff[i];
    }
    while(SPI1STATbits.SRMPT != 0);
    for(i = 8;i< 16;i++)
    {
        SPI1BUF = ledbuff[i];
    }
    while(SPI1STATbits.SRMPT != 0);
    for(i = 16;i< 24;i++)
    {
        SPI1BUF = ledbuff[i];
    }
    while(SPI1STATbits.SRMPT != 0);
}
void main(void) 
{
    CLKDIVbits.PLLPRE = 3;// divide by 5 so 4Mhz
    CLKDIVbits.PLLPOST = 0b11;// div by 8
    PLLFBDbits.PLLDIV = 78;
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB8 = 0;
    
    RPOR4bits.RP43R = 0b001110;
    RPINR26bits.C1RXR = 0x2a;
    TRISBbits.TRISB10 = 1;
    TRISBbits.TRISB11 = 0;
    
    
    
    C1CTRL1bits.REQOP = 0b100;//set config mode
    while(C1CTRL1bits.OPMODE != 0b100);
    
    C1CFG1 = 0x3f;//2*64, 1 jump width
    C1CFG2bits.PRSEG = 0b01;//propagation segment 2xTq
    C1CFG2bits.SEG1PH = 0b110;// 7xTq
    C1CFG2bits.SEG2PH = 0b101;// 6xTq
    C1CFG2bits.SEG2PHTS = 1;//freely selectable
    
    
 C1CTRL1bits.WIN = 0;
 DMA0CONbits.SIZE = 0x0;
 DMA0CONbits.DIR = 0x1;
 DMA0CONbits.AMODE = 0x2;
 DMA0CONbits.MODE = 0x0;
 DMA0REQ = 70;
 DMA0CNT = 7;
 DMA0PAD = (volatile unsigned int)&C1TXD;
 DMA0STAL = (unsigned int) &ecan1MsgBuf;
 DMA0STAH = (unsigned int) &ecan1MsgBuf;
 DMA0CONbits.CHEN = 0x1;
 C1TR01CONbits.TXEN0 = 0x1;
 C1TR01CONbits.TX0PRI = 0x3;
 C1CTRL1bits.REQOP = 0;
 while(C1CTRL1bits.OPMODE != 0);
 
 ecan1MsgBuf[0][0] = 0x0;
 
 ecan1MsgBuf[0][1] = 0x0;
 ecan1MsgBuf[0][2] = 0x4;
 ecan1MsgBuf[0][3] = 0xa0a;
 ecan1MsgBuf[0][4] = 0x10a;
 ecan1MsgBuf[0][5] = 0x0;
 
 
// C1TR01CONbits.TXREQ0 = 0x1;
// while(C1TR01CONbits.TXREQ0 == 1);

 SPI1CON1bits.MSTEN = 1;
 
 
 
 
 SPI1CON1bits.SPRE = 0b110;
 SPI1CON1bits.CKE = 0;
 SPI1CON1bits.PPRE = 0b11;
 SPI1CON1bits.MODE16 = 0;
 SPI1CON2bits.SPIBEN = 1;
 
 
 
 SPI1STATbits.SPIEN = 1;
 
 
 
 
    while(1)
    {
        
        int i;
        for(i = 0;i<5;i++)
        {
            writeLed(0x000200);
        }
        __delay_ms(500);
        for(i = 0;i<5;i++)
        {
            writeLed(0x020000);
        }
        
        __delay_ms(500);
        
    }
    return;
}
