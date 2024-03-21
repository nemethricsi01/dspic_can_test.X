/*
 * File:   newmain.c
 * Author: nemet
 *
 * Created on 2023. j�nius 29., 1:00
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
#pragma config ICS = PGD1               // ICD Communication Channel Select Bits (Communicate on PGEC2 and PGED2)

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
#pragma config PWMLOCK = OFF             // PWM Lock Enable Bit (Certain PWM registers may only be written after key sequence)
#pragma config ALTI2C1 = OFF            // Alternate I2C1 Pins Selection Bit (I2C1 mapped to SDA1/SCL1 pins)

// FALTREG
#pragma config CTXT1 = NONE             // Interrupt Priority Level (IPL) Selection Bits For Alternate Working Register Set 1 (Not Assigned)
#pragma config CTXT2 = NONE             // Interrupt Priority Level (IPL) Selection Bits For Alternate Working Register Set 2 (Not Assigned)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <string.h>
#include "delay.h"
#include "ws2812_driver/ws2812_led.h"
#include "ws2812_driver/color.h"
#include "dma_driver/dma.h"
#include "spi_driver/spi.h"
#include "gpio_driver/gpio.h"
#include "display_driver/display.h"
unsigned int ecan1MsgBuf[4][8]__attribute__((aligned(4 * 16)));



#define lcd_rs LATBbits.LATB7
#define lcd_en LATBbits.LATB8

#define lcdc_start	0x30	// Clear LCD
#define lcdc_4bit	0x20	// Clear LCD

#define lcdc_clear	0x01	// Clear LCD
#define lcdc_c_home	0x02	// Cursor home
#define lcdc_em_set	0x06	// Entry mode set
#define lcdc_dp_ctrl	0x0C	// Display Control
#define lcdc_cd_sht	0x10	// Cursor or Display Shift
#define lcdc_f_set_n	0x38	// Function Set normal instruction table
#define lcdc_f_set_e	0x39	// Function Set extra instruction table

#define lcdc_bias_set	0x1C	// Bias Set
#define lcdc_set_ICON	0x40	// Set ICON Address
#define lcdc_P_I_C_set	0x52	// Power, ICON Control, Contrast set 
#define lcdc_fcon_set	0x69	// Follower control
#define lcdc_cont_set	0x74	// Contrast set 

#define lcdc_s_caddr	0x40	// Set CGRAM address
#define lcdc_s_daddr	0x80	// Set DDRAM address

#define lcdc_dp_ctrl_con		0x0E	// Display Control cursor on
#define lcdc_dp_ctrl_coff		0x0C	// Display Control cursor off

const char def_char[3][8] = {
                           0x00,0x08,0x0C,0x0E,0x0C,0x08,0x00,0x00,
                           0x00,0x02,0x06,0x0E,0x06,0x02,0x00,0x00,
                           0x05,0x0A,0x00,0x0E,0x11,0x11,0x0E,0x00,
//                           0x00,0x00,0x04,0x0A,0x0A,0x11,0x00,0x00,
//                           0x00,0x00,0x11,0x0A,0x0A,0x04,0x00,0x00,
                          };
uint8_t global;
char lcdline;
const char LCD_INIT_STRING[9] = {lcdc_f_set_e, lcdc_dp_ctrl, lcdc_em_set, lcdc_bias_set, lcdc_set_ICON, lcdc_P_I_C_set, lcdc_fcon_set, lcdc_cont_set, lcdc_f_set_n};
// Create an array of 28 LEDs
LED leds[28];

Display display;
uint32_t buttonBuff = 0;
uint32_t lastbuttons;
uint8_t buttons[8];

void lcd_send_nibble( char n ) 
	{
	LATC = (n & 0x0F) | (LATC & 0xF0);
//	__delay_us(10);
	//lcd_en = 1;
//	__delay_us(10);
	//lcd_en = 0;
	}
unsigned char reverse(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

void lcd_send_byte( char address, char n )
	{
	lcd_rs = address;
    uint8_t temp;
    temp = reverse(n);
    global = temp;
	// __delay_us(10);
	//lcd_en = 0;
//    SPI_WriteByte(n );
    SPI2BUF = n;
    // __delay_us(100);
    //lcd_en = 1;
 	// __delay_us(30);
	}

void lcd_caddr( char addr) 
	{
	char address;
	address=addr*8;
	lcd_send_byte(0,0x40|address);
	}


void lcd_setc(char cadd,char chr)
	{
	char i;
	lcd_caddr(cadd);
	for(i=0;i<8;i++)
		{lcd_send_byte(1,def_char[chr][i]);}
	}

void lcd_init() 
	{
	char i;
	lcd_rs = 0;
	//lcd_en = 0;
	// __delay_ms(40);

	for(i=0;i<9;++i)
		{
		lcd_send_byte(0,LCD_INIT_STRING[i]);
		}
	lcd_send_byte(0,0x01);
	// __delay_ms(2);
	lcd_setc(0,0);	//<
	lcd_setc(1,1);	//>
	lcd_setc(2,2);	//?
	}
void lcd_gotoxy( char cx, char cy) 
	{
	char address;
	lcdline = cy;
	switch(cy) 
		{
    	case 1 : address=0x00; break;
		case 2 : address=0x40; break;
		default : address=0x00; lcdline=1; break;
		}
	address=(address+(cx-1));
	lcd_send_byte(0,lcdc_s_daddr|address);
	}
void readButtons(void)
{   
    CNPUBbits.CNPUB9 = 1;
    CNPUBbits.CNPUB10 = 1;
    CNPUBbits.CNPUB11 = 1;
    CNPUBbits.CNPUB12 = 1;
    CNPUCbits.CNPUC6 = 1;
    CNPUCbits.CNPUC7 = 1;
    CNPUCbits.CNPUC8 = 1;
    CNPUCbits.CNPUC9 = 1;
    int x = 0;
    for(x = 0;x<3;x++)
    {
        if(x == 0)
        {
            TRISAbits.TRISA7 =  0;//OSZLOP_C
            TRISAbits.TRISA10 = 0;//OSZLOP_B
            TRISBbits.TRISB13 = 0;//OSZLOP_A
            LATAbits.LATA7 =    1;
            LATAbits.LATA10 =   1;
            LATBbits.LATB13 =   0;
        }
        if(x == 1)
        {
            TRISAbits.TRISA7 =  0;//OSZLOP_C
            TRISAbits.TRISA10 = 0;//OSZLOP_B
            TRISBbits.TRISB13 = 0;//OSZLOP_A
            LATAbits.LATA7 =    1;
            LATAbits.LATA10 =   0;
            LATBbits.LATB13 =   1;
        }
        if(x == 2)
        {
            TRISAbits.TRISA7 =  0;//OSZLOP_C
            TRISAbits.TRISA10 = 0;//OSZLOP_B
            TRISBbits.TRISB13 = 0;//OSZLOP_A
            LATAbits.LATA7 =    0;
            LATAbits.LATA10 =   1;
            LATBbits.LATB13 =   1;
        }
        buttons[0] = PORTBbits.RB9;
        buttons[1] = PORTCbits.RC6;
        buttons[2] = PORTCbits.RC7;
        buttons[3] = PORTCbits.RC8;
        buttons[4] = PORTCbits.RC9;
        buttons[5] = PORTBbits.RB10;
        buttons[6] = PORTBbits.RB11;
        buttons[7] = PORTBbits.RB12;

       uint32_t i;
       for(i = 0;i<8;i++)
       {
           if(buttons[i] == 0)
           {
               buttonBuff |= 1l<<((i*3)+x);
           }
           else
           {
            buttonBuff &= ~(1l<<((i*3)+x)); 
               //buttonBuff= 0;
           }
       }
    }
}
void main(void) 
{
    CLKDIVbits.PLLPRE = 3;// divide by 5 so 4Mhz
    CLKDIVbits.PLLPOST = 0b11;// div by 8
    PLLFBDbits.PLLDIV = 78;
    
/* Set PWM Period on Primary Time Base */
PTPER = 1000;
/* Set Phase Shift */
PHASE1 = 0;
/* Set Duty Cycles */

PDC1 = 500;
/* Set Dead Time Values */
DTR1 = 5;
ALTDTR1 = 5;
/* Set PWM Mode to Push-Pull */
IOCON1 = 0xC000;
/* Set Primary Time Base, Edge-Aligned Mode and Independent Duty Cycles */
PWMCON1 = 0x0000;
/* Configure Faults */
FCLCON1 = 0x0003;
/* 1:8 Prescaler */
PTCON2 = 0x0003;
/* Enable PWM Module */
PTCON = 0x8000;
    
    
//    __builtin_disable_interrupts();
//    __builtin_enable_interrupts(); // Enable global interrupts
    
//    RPINR26bits.C1RXR = 0x2a;//CANRX
//    TRISBbits.TRISB10 = 1;
//    TRISBbits.TRISB11 = 0;
//    TRISAbits.TRISA9 = 1;
//    
//    
//    C1CTRL1bits.REQOP = 0b100;//set config mode
//    while(C1CTRL1bits.OPMODE != 0b100);
//    
//    C1CFG1 = 0x3f;//2*64, 1 jump width
//    C1CFG2bits.PRSEG = 0b01;//propagation segment 2xTq
//    C1CFG2bits.SEG1PH = 0b110;// 7xTq
//    C1CFG2bits.SEG2PH = 0b101;// 6xTq
//    C1CFG2bits.SEG2PHTS = 1;//freely selectable
//    
//    
// C1CTRL1bits.WIN = 0;
// DMA0CONbits.SIZE = 0x0;
// DMA0CONbits.DIR = 0x1;
// DMA0CONbits.AMODE = 0x2;
// DMA0CONbits.MODE = 0x0;
// DMA0REQ = 70;
// DMA0CNT = 7;
// DMA0PAD = (volatile unsigned int)&C1TXD;
// DMA0STAL = (unsigned int) &ecan1MsgBuf;
// DMA0STAH = (unsigned int) &ecan1MsgBuf;
// DMA0CONbits.CHEN = 0x1;
// C1TR01CONbits.TXEN0 = 0x1;
// C1TR01CONbits.TX0PRI = 0x3;
// C1CTRL1bits.REQOP = 0;
// while(C1CTRL1bits.OPMODE != 0);
// 
// ecan1MsgBuf[0][0] = 0x0;
// 
// ecan1MsgBuf[0][1] = 0x0;
// ecan1MsgBuf[0][2] = 0x4;
// ecan1MsgBuf[0][3] = 0xa0a;
// ecan1MsgBuf[0][4] = 0x10a;
// ecan1MsgBuf[0][5] = 0x0;
 
 
// C1TR01CONbits.TXREQ0 = 0x1;
// while(C1TR01CONbits.TXREQ0 == 1);


 gpio_init();
  ws2812_init_leds(leds, NUM_LEDS);

 __delay_ms(1000);
Display_Init(&display);

Display_Printf(&display,0,"abcdefghijklmnop");
Display_Printf(&display,1,"qrstuwxyz1234567");
Display_Send(&display);

 int delay = 20;
 int delay_slow = 500;
// 
 

    while(1)
    { 
        LATBbits.LATB3^= 1;
        for(uint32_t i= 0;i<255;i++)
        {
        ws2812_set_color_range(leds, NUM_LEDS, 0, 0, i<<16);
        ws2812_set_color_range(leds, NUM_LEDS, 1, 1, i<<8);
        ws2812_set_color_range(leds, NUM_LEDS, 2, 2, i);
        ws2812_send_buffer(leds, NUM_LEDS);
        if(i > 10)
        {
            // __delay_ms(delay);
            __delay_ms(delay);
        }
        else
        {
            // __delay_ms(delay_slow);
            __delay_ms(delay_slow);
        }
        
        }
        for(uint32_t i= 255;i>0;i--)
        {
        ws2812_set_color_range(leds, NUM_LEDS, 0, 0, i<<16);
        ws2812_set_color_range(leds, NUM_LEDS, 1, 1, i<<8);
        ws2812_set_color_range(leds, NUM_LEDS, 2, 2, i);
        ws2812_send_buffer(leds, NUM_LEDS);
        if(i <10)
        {
            // __delay_ms(delay_slow);
            __delay_ms(delay_slow);
        }
        else
        {
            // __delay_ms(delay);
            __delay_ms(delay);
        }
        }



        // ws2812_set_color_range(leds, NUM_LEDS, 0, 2, 0x010000);
        // ws2812_send_buffer(leds, NUM_LEDS);
        // __delay_ms(1000);
        // ws2812_set_color_range(leds, NUM_LEDS, 0, 2, 0x000100);
        // ws2812_send_buffer(leds, NUM_LEDS);
        // __delay_ms(1000);
        // ws2812_set_color_range(leds, NUM_LEDS, 0, 2, 0x000001);
        // ws2812_send_buffer(leds, NUM_LEDS);
        // __delay_ms(1000);
        // ws2812_set_color_range(leds, NUM_LEDS, 0, 2, 0x050000);
        // ws2812_send_buffer(leds, NUM_LEDS);
        // __delay_ms(1000);
        // ws2812_set_color_range(leds, NUM_LEDS, 0, 2, 0x000500);
        // ws2812_send_buffer(leds, NUM_LEDS);
        // __delay_ms(1000);
        // ws2812_set_color_range(leds, NUM_LEDS, 0, 2, 0x000005);
        // ws2812_send_buffer(leds, NUM_LEDS);
        // __delay_ms(1000);
    }
    return;
}
void __attribute__((__interrupt__, no_auto_psv)) _SPI1Interrupt(void)
{

    if(IFS0bits.SPI1IF == 1)
    {
        IFS0bits.SPI1IF = 0;
    }
    
}