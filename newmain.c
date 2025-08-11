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
#pragma config DMTEN = DISABLE           // Dead Man Timer Enable Bit (Dead Man Timer is Enabled and cannot be disabled by software)

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
#include "spi_driver/spi.h"
#include "gpio_driver/gpio.h"
#include "display_driver/display.h"
#include "timer/timer.h"
#include "flash_driver/flash_demo.h"
#include "can_driver/can.h"
#include "spi_protocol/spi_protocol.h"
#include "led_logic/led_logic.h"
#include "uart_driver/uart.h"
#include "uart_driver/uart_protocol.h"
#include "clock/clock.h"
#include "ui/ui.h"

// Create an array of 28 LEDs
LED leds[28];

//Display display;
uint32_t buttonBuff = 0;
uint32_t lastbuttons;

volatile uint8_t buttons[NUM_BUTTONS] = {0};

extern uint8_t update_leds;
extern volatile uint16_t buttonchecktimer;



extern volatile uint64_t one_millisecond_ticker;

uint8_t own_address = 0; // Example own address
uint8_t own_address_set = 0;
uint8_t change_menu = 0;


uint8_t led_safety_timer = 0; // led forced refresh for safety, if something stays lit by accident

uint8_t last_ledek[LEDTOMBNUM + 1];

static const uint32_t led_color_map[8] = {
    COLOR_BLACK,    // 0
    COLOR_BLUE,     // 1  
    COLOR_GREEN,    // 2
    COLOR_CYAN,     // 3
    COLOR_RED,      // 4
    COLOR_MAGENTA,  // 5
    COLOR_YELLOW,   // 6
    COLOR_WHITE     // 7
};


extern narval_msg_t narval_msg;


static const uint8_t led_index_map[28] = 
{
    15,//1
    18,//2
    21,//3
    0,//4
    3,//5
    6,//6
    9,//7
    12,//8
    17,//9
    20,//10
    24,//11
    2,//12
    5,//13
    8,//14
    11,//15
    14,//16
    16,//17
    19,//18
    1,//19
    4,//20
    7,//21
    10,//22
    13,//23
    25,//24
    26,//25
    27,//26
    22,//27
    23//28
};
void init_arrays(void)
{
    	for(int i = 0;i < GOMBNUM/32;i++){
		actgomb[i] = 0;
		gomble[i] = 0;
		gombfel[i] = 0;
		gombdupla[i] = 0;
		gombhosszu[i] = 0;
		gomblend[i] = 0;
		gombfelnd[i] = 0;
		gomblet[i] = 0;
		gombfelt[i] = 0;
	}
	for(int i = 0;i < LEDVTBNUM ;i++){
		ledvillall[i] = 0;
	}
	for(int i = 0;i < LEDVTNUM;i++){
		ledvilltimer[i] = 0;
		//ledpwm[i] = 31;
		ledpwm[i] = 3;
	}

	for(int i = 0;i < LEDTOMBNUM;i++){
		ledek[i] = 0;
		derengkimsk[i] = 0xFF;
	}
	for(int i = 0;i < LEDTOMBNUM + 1;i++){
		ledvill[i] = 0;
		ledall[i] = 0;
	}
	for(int i = 0;i < GOMBNUM;i++){
		gombdtimer[i] = 0;
		gombhtimer[i] = 0;
	}
}


static void get_own_add(narval_msg_t *msg)
{
    if(msg->len == 4 && msg->data[3] == 0x18 && msg->ext_or_int == CMD_INT)
    {
        own_address = msg->data[0];
        own_address_set = 1;
    }
    if(msg->len == 3 && msg->data[2] == 0x1 && msg->ext_or_int == CMD_INT)
    {
        own_address = msg->data[0];
        own_address_set = 1;
    }
}
static void get_reset(narval_msg_t *msg)
{
    // Check if the message is a reset command
    if(msg->len == 4 && msg->data[3] == 0xFF && msg->ext_or_int == CMD_EXT && msg->data[1] == own_address)
    {
        ui_init();
    }
    else if(msg->len == 4 && msg->data[3] == 0xFF && msg->ext_or_int == CMD_EXT && msg->data[1] == 0xFF)
    {
        ui_init();
    }
}
static void get_menubutton(narval_msg_t *msg)
{
    // Check if the message is a menu button command which is an outgoing call to 250
    if(msg->len == 3 && (msg->data[2] == 0x1 ) && msg->ext_or_int == CMD_INT && msg->data[1] == 249)
    {
        change_menu = 1;
    }
}
static void process_clock_commands(narval_msg_t* msg) 
{
    if (msg->len != 4) 
    {
        return; // Only process 4-byte messages
    }
    
    switch (msg->data[3]) 
    {
        case 0x1: // Set time command
            clock_set_time(msg->data[0] & 0x1f,  // Hours (mask to 5 bits, 0-31)
                          msg->data[1] & 0x3f,  // Minutes (mask to 6 bits, 0-63)
                          msg->data[2] & 0x3f); // Seconds (mask to 6 bits, 0-63)
            break;
            
        case 0x2: // Set date command
            clock_set_date((uint16_t)msg->data[0] + 2000ul, // Year (add 2000)
                          msg->data[1],                      // Month
                          msg->data[2]);                     // Day
            break;
            
        default:
            // Unknown command, do nothing
            break;
    }
}



int main(void) 
{
    CLKDIVbits.PLLPRE = 2;// divide by 4 so 5Mhz
    CLKDIVbits.PLLPOST = 0b01;// div by 4
    PLLFBDbits.PLLDIV = 30;
    
    RCONbits.SWDTEN = 0;
    
    /* Set PWM Period on Primary Time Base */
    PTPER = 1000;//1:8 division, 40MHz in, 5kHz out
    /* Set Phase Shift */
    PHASE1 = 0;
    /* Set Duty Cycles */

    PDC1 = 750;
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
    
    uart_init();
    init_arrays();
    ledvilltmrenabled = 0;
    memccpy(ledek, last_ledek, 0, LEDTOMBNUM + 1);

    gpio_init();
    ws2812_init_leds(leds, NUM_LEDS);
    
    // Display_Init(&display);
    // Display_Printf(&display,0,"abcdefghijklmnop");
    // Display_Printf(&display,1,"qrstuwxyz1234567");
    // Display_Send(&display);
    spi2_init();
    spi2_enable();
    
    LATCbits.LATC5 = 1;
    SPI2BUF = 0x01;
    
    timer_4_init();
    timer_5_init();
    timer_1_init();
    timer_1_start();
    clock_init();
    ui_init();
    
 
    while(1)
    { 
        LATBbits.LATB0 = !PORTCbits.RC4; // MUTE invert

        if(narval_msg.new_data_available)
        {
            
            // Process the received UART message
            get_own_add(&narval_msg);
            process_clock_commands(&narval_msg);
            get_menubutton(&narval_msg);
            get_reset(&narval_msg);
            

            ui_update(one_millisecond_ticker, &narval_msg, own_address_set, own_address, &change_menu);

            narval_msg.new_data_available = 0;
        }
        else
        {
            // If no new data, just update the UI
            ui_update(one_millisecond_ticker, &narval_msg, own_address_set, own_address, &change_menu);
        }
        // ui_update(one_millisecond_ticker,&narval_msg,own_address_set, own_address, &change_menu);

        // if(uart_new_data_available)
        // {
        //     uart_new_data_available = 0;
            
        //     switch(narval_msg.len)
        //     {
        //         case 3:
        //         {
        //             switch(narval_msg.data[2])
        //             {
        //                 case 0x1:
        //                 {
        //                     if(narval_msg.ext_or_int == CMD_EXT)
        //                     {
        //                         // External command, process accordingly
        //                         Display_Printf(&display, 1, "\x3E%d h\xE1vja:  %d   ", narval_msg.data[0]+1, narval_msg.data[1]+1);
        //                     }
        //                     else
        //                     {
        //                         // Internal command, process accordingly
        //                         Display_Printf(&display, 1, "\x5E%d h\xE1vja:  %d   ", narval_msg.data[0]+1, narval_msg.data[1]+1);
        //                     }
        //                     break;
        //                 }
        //                 case 0x2:
        //                 {
        //                     if(narval_msg.ext_or_int == CMD_EXT)
        //                     {
        //                         // External command, process accordingly
        //                         Display_Printf(&display, 1, "\x3E%d bontja: %d   ", narval_msg.data[0]+1, narval_msg.data[1]+1);
        //                     }
        //                     else
        //                     {
        //                         // Internal command, process accordingly
        //                         Display_Printf(&display, 1, "\x5E%d bontja: %d   ", narval_msg.data[0]+1, narval_msg.data[1]+1);
        //                     }
        //                     break;
        //                 }
        //                 default:
        //                     break;
        //             }
        //             break;
        //         }
        //         case 4:
        //         {
        //            switch(narval_msg.data[3])
        //             {
        //                 case 0x1:
        //                 {
        //                     // Set clock from UART message
        //                     clock_set_time(narval_msg.data[0] & 0x1f, 
        //                                     narval_msg.data[1] & 0x3f, 
        //                                     narval_msg.data[2] & 0x3f);
        //                     break;
        //                 }
        //                 case 0x2:
        //                 {
        //                     // Set date from UART message
        //                     clock_set_date((uint16_t)narval_msg.data[0]+2000ul,
        //                                             narval_msg.data[1],
        //                                             narval_msg.data[2]);
        //                     break;
        //                 }
        //                 default:
        //                     break;
        //             } 
        //             break;
        //         }
        //         default:
        //             break;

        //     }
//            if((narval_msg.len == 4) && (narval_msg.data[3] == 0x1)) {
//                // Set clock from UART message
//                clock_set_time(narval_msg.data[0] & 0x1f, 
//                              narval_msg.data[1] & 0x3f, 
//                              narval_msg.data[2] & 0x3f);
//            }
//            if((narval_msg.len == 4) && (narval_msg.data[3] == 0x2)) 
//            {
//                // Set clock from UART message
//                clock_set_date((uint16_t)narval_msg.data[0]+2000ul,
//                narval_msg.data[1],
//                narval_msg.data[2]);
//            }
            // if((narval_msg.len == 4) && (narval_msg.data[3] == 0x1))//clock info
            // {
            //     Display_Printf(&display,0,"%d:%d:%d", narval_msg.data[0]&0x1f, narval_msg.data[1]&0x3f, narval_msg.data[2]&0x3f);
            // }
            // Display_Send(&display);
            
        // }
        
        // if(one_second_flag)
        // {
        //     one_second_flag = 0;
        //     clock_time_t current_time = clock_get_time();
        //     clock_date_t current_date = clock_get_date();
        //     Display_Printf(&display, 0, "'%02d.%02d.%02d. %2d:%02d",
        //                     current_date.years % 100,
        //                     current_date.months,
        //                     current_date.days,
        //                     current_time.hours, 
        //                     current_time.minutes);
        //     Display_Send(&display);
        // }

        


        if((led_update_timer == 0))
        {
            led_safety_timer++;
            led_update_timer = LED_UPDATE_TIME;
            
             if((memcmp(ledek, last_ledek, 20) != 0)||(led_safety_timer == 100))
             {
                memcpy(last_ledek, ledek, LEDTOMBNUM + 1);
                led_safety_timer = 0; // Reset safety timer
                /*
                the ledek array contains the state of the LEDs
                0-1 byte contains the villsav leds, coded as 0-7
                2-12 byte contains the leds, coded as 0:yellow, 1:green, 2:red, 3 is used for button originally with the shift registers
                4-11 byte contains the leds, coded as 0:yellow, 1:green, 2:red, 3 is used for button originally with the shift registers
                */ 
                
                /*
                 * byte11 contains a led that the device does not have ( button/led19 )
                 * so it needs to be skipped
                 */
                uint8_t led_idx = 0;
                // Process bytes 2-13, handling the special case for byte 11
                for(uint8_t byte_idx = 2; byte_idx <= 13; byte_idx++) 
                {
                    uint8_t temp = getledek(byte_idx);
                    
                    // Always process lower nibble
                    if(byte_idx != 11 && led_idx < 24) {  // Bounds check
                        ws2812_set_color_range(leds, NUM_LEDS, led_index_map[led_idx], 
                                            led_index_map[led_idx], led_color_map[temp & 0x7]);
                        led_idx++;
                    }
                    
                    // Process upper nibble for all bytes except byte 11
                    if(led_idx < 24) {  // Skip upper nibble for byte 11
                        ws2812_set_color_range(leds, NUM_LEDS, led_index_map[led_idx], 
                                            led_index_map[led_idx], led_color_map[(temp >> 4) & 0x7]);
                        led_idx++;
                    }
                }
                uint8_t temp = getledek(1);
                for(uint8_t i = 0; i < VILLSAV_LED_COUNT; i++)
                {
                    if((temp & (1 << i)) != 0)
                    {
                        ws2812_set_color_range(leds, NUM_LEDS, led_index_map[i+23], 
                                            led_index_map[i+23], COLOR_RED);
                    }
                    else
                    {
                        ws2812_set_color_range(leds, NUM_LEDS, led_index_map[i+23], 
                                            led_index_map[i+23], COLOR_BLACK);
                    }
                }

                ws2812_send_buffer(leds,NUM_LEDS);
            }
        }
        if(buttonchecktimer == 0)
        {
            buttonchecktimer = 20; // Reset timer
            read_buttons(buttons);
        }   
    }
    return 0;
}

