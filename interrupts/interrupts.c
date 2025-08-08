#include "interrupts.h"
#include <xc.h>

#include "../spi_driver/spi.h"

#include "..//spi_protocol/spi_protocol.h"
#include "../led_logic/led_logic.h"

#include "../ws2812_driver/ws2812_led.h"

#include "../gpio_driver/gpio.h"

#include "../clock/clock.h"

#include "../uart_driver/uart_protocol.h"


extern volatile uint8_t buttons[NUM_BUTTONS];
volatile uint16_t buttonchecktimer = 8000;





void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) {
    if(IFS0bits.T1IF == 1)
    {
        LATCbits.LATC1 ^= 1;
        clock_tick();  // Update clock every 1ms
        IFS0bits.T1IF = 0;
    }
}


void __attribute__((__interrupt__, no_auto_psv)) _T4Interrupt(void) {
    
    TMR4 = 0;
    IFS1bits.T4IF = 0;// Clear Timer 4 Interrupt Flag
    if(led_update_timer > 0)
    {
        led_update_timer--;
    }
    
    if(buttonchecktimer > 0)
    {
        buttonchecktimer--;
    }
        
        for(uint32_t button_index = 0; button_index < NUM_BUTTONS; button_index++) 
        {
            // Check if button is currently pressed
            if(buttons[button_index] == 1) 
            {
                
                if((actgomb[button_index / 32] & (1UL << (button_index & 0b00011111))) == 0)
                {
                    
                     actgomb[button_index / 32] |= (1UL << (button_index & 0b00011111));
                    gomble[button_index / 32] |= (1UL << (button_index & 0b00011111));
                    gomblet[button_index / 32] |= (1UL << (button_index & 0b00011111));
                    if(gombdtimer[button_index] != 0)
                    {	//Duplakatt
                        gombdupla[button_index / 32] |= (1UL << (button_index & 0b00011111));
                        gombdtimer[button_index] = 0;
                        
                        gomblet[button_index / 32] &= ((1UL << (button_index & 0b00011111)) ^ 0xFFFFFFFF);
                        gombfelt[button_index / 32] &= ((1UL << (button_index & 0b00011111)) ^ 0xFFFFFFFF);
                    }
                    else
                    {
                        gombdtimer[button_index] = duplatime;
                    }
                    gombhtimer[button_index] = hosszutime;
		        }
            } 
            else 
            {
                if((actgomb[button_index / 32] & (1UL << (button_index & 0b00011111))) != 0)
                {
                    actgomb[button_index / 32] &= ((1UL << (button_index  & 0b00011111)) ^ 0xFFFFFFFF);
                    gombfel[button_index / 32] |= (1UL << (button_index  & 0b00011111));
                    if(gombdtimer[button_index] == 0)
                    {
                        gombfelnd[button_index / 32] |= (1UL << (button_index  & 0b00011111));
                    }
                    else
                    {
                        gombfelt[button_index / 32] |= (1UL << (button_index  & 0b00011111));
                    }
                }
            }
        }

}
void __attribute__((__interrupt__, no_auto_psv)) _T5Interrupt(void) {
    
    TMR5 = 0;
    IFS1bits.T5IF = 0;// Clear Timer 5 Interrupt Flag
    
    process_leds();

    uint32_t i;
    
    
        for(i = 0;i < GOMBNUM;i++){
		if(gombhtimer[i] > 0){
            
			gombhtimer[i]--;
			if(gombhtimer[i] == 0){
				if((actgomb[i / 32] & (1UL << (i % 32))) != 0){  //Hossz� gombnyom�s
					gombhosszu[i / 32] |= (1UL << (i % 32));
					//ledvillcnt[GOMBNUM * 3 + 16 + 1] = 0;
					//ledvilltimer[20] = 1;	
					//ledvilltimeon[20] = 2;	
					//ledvilltimeoff[20] = 10;	
					//ledvillall[3] &= 0b11101111;	
					//ledvill[LEDTOMBNUM] |= 0b00000010;
				}
			}
		}
		if(gombdtimer[i] > 0){
            
			gombdtimer[i]--;
			if(gombdtimer[i] == 0){
				if((gomblet[i / 32] & (1UL << (i % 32))) != 0){
					gomblend[i / 32] |= (1UL << (i % 32));
					gomblet[i / 32] &= ((1UL << (i % 32)) ^ 0xFFFFFFFF);
				}
				if((gombfelt[i / 32] & (1UL << (i % 32))) != 0){
					gombfelnd[i / 32] |= (1UL << (i % 32));
					gombfelt[i / 32] &= ((1UL << (i % 32)) ^ 0xFFFFFFFF);
				}
			}
		}
	}
    
    
    
//    SendLedek();
    // Add your code here
}

void __attribute__((__interrupt__, no_auto_psv)) _SPI2Interrupt (void)
{

    if(IFS2bits.SPI2IF)
    {
        
        uint8_t received;
        received = SPI2BUF; // Read the received data
        process_spi(received); // Process the received data
        LATCbits.LATC5 ^= 1;//toggle sync pin
        IFS2bits.SPI2IF = 0;
    }
    if(IFS2bits.SPI2EIF)
    {
        
        if(SPI2STATbits.SPIROV == 1)
        {
            SPI2STATbits.SPIROV = 0;
        }
        IFS2bits.SPI2EIF = 0;
    }
    
}

void __attribute__((__interrupt__, no_auto_psv)) _SPI1Interrupt(void)
{

    if(IFS0bits.SPI1IF == 1)
    {
        
        IFS0bits.SPI1IF = 0;
    }
    if(IFS0bits.SPI1EIF == 1)
    {
        if(SPI1STATbits.SPIROV == 1)
        {
            SPI1STATbits.SPIROV = 0;
        }
        IFS0bits.SPI1EIF = 0;
    }
    
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{

    if(IFS0bits.U1RXIF == 1)
    {
        while (U1STAbits.URXDA) 
        {
            uint8_t received_byte = U1RXREG; // Read the received byte
             uart_process(received_byte);
        }
        IFS0bits.U1RXIF = 0;
    }
    
}

void __attribute__((interrupt, no_auto_psv)) _DMA2Interrupt(void)
{
    IFS1bits.DMA2IF = 0; // Clear the DMA2 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA3Interrupt(void)
{
    IFS2bits.DMA3IF = 0; // Clear the DMA3 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
    IFS0bits.DMA0IF = 0; // Clear the DMA2 Interrupt Flag;
    led_ready = 1;
    
}

void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void)
{
    IFS0bits.DMA1IF = 0; // Clear the DMA3 Interrupt Flag;
    
}