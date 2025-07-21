/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  

#define LEDTOMBNUM	38
#define GOMBNUM		72
#define LEDVTNUM	21
#define LEDVTBNUM	3


#define PWMSZAM		64
#define TMRSZAM		50



#define LED_UPDATE_TIME 50 //20 ms timer = 2.5kHz

#define VILLSAV_LED_COUNT 5


extern unsigned char ledpwm[LEDVTNUM + 4];	//0..5, 8..13: villsav; 6..7, 14..15: rotary; 16..18: gombledek; 19: pwr; 20: hatter; 21..24: Dereng1..4,
extern unsigned char ledvilltimer[LEDVTNUM];	//0..5, 8..13: villsav; 6..7, 14..15: rotary; 16..18: gombledek; 19: pwr; 20: hatter
extern unsigned char ledvilltimeon[LEDVTNUM];	//0..5, 8..13: villsav; 6..7, 14..15: rotary; 16..18: gombledek; 19: pwr; 20: hatter
extern unsigned char ledvilltimeoff[LEDVTNUM];	//0..5, 8..13: villsav; 6..7, 14..15: rotary; 16..18: gombledek; 19: pwr; 20: hatter
extern unsigned char ledvillall[LEDVTBNUM];	//0..5, 8..13: villsav; 6..7, 14..15: rotary; 16..18: gombledek; 19: pwr; 20: hatter



extern uint8_t derengkimsk[LEDTOMBNUM + 1];	//vills�v+encoder+Gombledek, pwr+h�tt�r
extern uint8_t ledek[LEDTOMBNUM + 1];			//vills�v+encoder+Gombledek, pwr+h�tt�r
extern uint8_t last_ledek[LEDTOMBNUM + 1];
extern uint8_t ledall[LEDTOMBNUM + 1];	//vills�v+encoder+Gombledek, pwr+h�tt�r
extern uint8_t ledvill[LEDTOMBNUM + 1];	//vills�v+encoder+Gombledek, pwr+h�tt�r
extern uint8_t ledvillcnt[GOMBNUM * 3 + 16 + 2];	//Vills�v+encoder, Gombledek, pwr+h�tt�r 
extern uint8_t ledvilltmrenabled;


extern unsigned char delaytimer;
extern unsigned char pwmszaml;
extern unsigned int tmrszaml;

extern uint8_t led_update_timer;




void process_leds(void);
void SendLedek(void);

#endif	/* XC_HEADER_TEMPLATE_H */

