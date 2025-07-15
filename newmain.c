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
#include "timer/timer.h"
#include "flash_driver/flash_demo.h"
#include "can_driver/can.h"

extern unsigned int canTxBuff[4][8]__attribute__((aligned(4 * 16)));
extern unsigned int canRxBuff[32][8]__attribute__((aligned(32 * 16)));

// Create an array of 28 LEDs
LED leds[28];

Display display;
uint32_t buttonBuff = 0;
uint32_t lastbuttons;
uint8_t buttons[NUM_BUTTONS];



int main(void) 
{
    CLKDIVbits.PLLPRE = 2;// divide by 4 so 5Mhz
    CLKDIVbits.PLLPOST = 0b01;// div by 4
    PLLFBDbits.PLLDIV = 30;
    
    RCONbits.SWDTEN = 0;
    
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
IOCON1 = 0x8000;
IOCON2 = 0x0000;
IOCON3 = 0x0000;
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
    
ANSELC = 0;
    

 gpio_init();
 ws2812_init_leds(leds, NUM_LEDS);

  __delay_ms(1000);
 Display_Init(&display);

 Display_Printf(&display,0,"abcdefghijklmnop");
 Display_Printf(&display,1,"qrstuwxyz1234567");
 Display_Send(&display);
 //timer_4_init();
// timer_5_init();
//  int delay = 20;
//  int delay_slow = 500;
// 
 

 
 
//FlashDemo();



 
    while(1)
    { 
        // read_buttons(buttons);
        // __delay_ms(1);
        // int8_t index = -1;
        // for(int i = 0; i < NUM_BUTTONS; i++)
        // {
        //     if (buttons[i] == 1)
        //     {
        //         index = i;
        //         // If the button is pressed, set the corresponding LED to red
        //         ws2812_set_color_range(leds, NUM_LEDS, i, i, COLOR_RED);
        //     }
        //     else
        //     {
        //         // If the button is not pressed, set the corresponding LED to off
        //         ws2812_set_color_range(leds, NUM_LEDS, i, i, COLOR_BLACK);
        //     }
        // }
        // ws2812_send_buffer(leds, NUM_LEDS);

    for (int pos = 0; pos < 24; pos++)
    {
        // Turn all LEDs off first
        for (int i = 0; i < NUM_LEDS; i++)
        {
            ws2812_set_color_range(leds, NUM_LEDS, i, i, COLOR_BLACK);
        }
        // Set the current position to red
        ws2812_set_color_range(leds, NUM_LEDS, pos, pos, COLOR_RED);

        // Send the buffer to the LEDs
        ws2812_send_buffer(leds, NUM_LEDS);

        // Delay for visibility
        __delay_ms(10);
    }



        // Display_Printf(&display,0,"%d", index);
        // Display_Printf(&display,1,"qrstuwxyz1234567");
        // Display_Send(&display);
        /* Message was received. */

        // __delay_ms(10);
        // Display_Printf(&display,0,"abcdefghijklmnop");
        // Display_Printf(&display,1,"qrstuwxyz1234567");
        // Display_Send(&display);
//        for(uint32_t i= 0;i<255;i++)
//        {
//            ws2812_set_color_range(leds, NUM_LEDS, 0, 0, i<<16);
//            ws2812_set_color_range(leds, NUM_LEDS, 1, 1, i<<8);
//            ws2812_set_color_range(leds, NUM_LEDS, 2, 2, i);
//            ws2812_send_buffer(leds, NUM_LEDS);
//            if(i > 10)
//            {
//                __delay_ms(100);
//            }
//            else
//            {
//                __delay_ms(200);
//            }
//        }
        // for(uint32_t i= 255;i>0;i--)
        // {
        //     ws2812_set_color_range(leds, NUM_LEDS, 0, 0, i<<16);
        //     ws2812_set_color_range(leds, NUM_LEDS, 1, 1, i<<8);
        //     ws2812_set_color_range(leds, NUM_LEDS, 2, 2, i);
        //     ws2812_send_buffer(leds, NUM_LEDS);
        //     if(i <10)
        //     {
        //         __delay_ms(delay_slow);
        //     }
        //     else
        //     {
        //         __delay_ms(delay);
        //     }
        // }
    }
    return 0;
}
void __attribute__((interrupt, no_auto_psv)) _DMA2Interrupt(void)
{
    IFS1bits.DMA2IF = 0; // Clear the DMA2 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA3Interrupt(void)
{
    IFS2bits.DMA3IF = 0; // Clear the DMA3 Interrupt Flag;
}
