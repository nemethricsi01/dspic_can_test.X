#include "led_logic.h"
#include <stdint.h>
#include "../ws2812_driver/ws2812_led.h"
#include "../ws2812_driver/color.h"

unsigned char ledpwm[LEDVTNUM + 4];	//0..5, 8..13: villsav; 6..7, 14..15: rotary; 16..18: gombledek; 19: pwr; 20: hatter; 21..24: Dereng1..4,
unsigned char ledvilltimer[LEDVTNUM];	//0..5, 8..13: villsav; 6..7, 14..15: rotary; 16..18: gombledek; 19: pwr; 20: hatter
unsigned char ledvilltimeon[LEDVTNUM];	//0..5, 8..13: villsav; 6..7, 14..15: rotary; 16..18: gombledek; 19: pwr; 20: hatter
unsigned char ledvilltimeoff[LEDVTNUM];	//0..5, 8..13: villsav; 6..7, 14..15: rotary; 16..18: gombledek; 19: pwr; 20: hatter
unsigned char ledvillall[LEDVTBNUM];	//0..5, 8..13: villsav; 6..7, 14..15: rotary; 16..18: gombledek; 19: pwr; 20: hatter

uint8_t derengkimsk[LEDTOMBNUM + 1];	//vills�v+encoder+Gombledek, pwr+h�tt�r
uint8_t ledek[LEDTOMBNUM + 1];			//vills�v+encoder+Gombledek, pwr+h�tt�r
uint8_t ledall[LEDTOMBNUM + 1];	//vills�v+encoder+Gombledek, pwr+h�tt�r
uint8_t ledvill[LEDTOMBNUM + 1];	//vills�v+encoder+Gombledek, pwr+h�tt�r
uint8_t ledvillcnt[GOMBNUM * 3 + 16 + 2];	//Vills�v+encoder, Gombledek, pwr+h�tt�r 
uint8_t ledvilltmrenabled;


unsigned char delaytimer;
unsigned char pwmszaml = 1;
unsigned int tmrszaml = 1;

extern unsigned char fizicnum;

uint8_t led_update_timer = 0;



void process_leds(void)
{
    uint8_t i,a;

    if(ledvilltmrenabled)
    {
		for(i = 0;i < LEDVTNUM;i++)
        {
			if(ledvilltimer[i] > 0)
            {
				ledvilltimer[i]--;
				if(ledvilltimer[i] == 0)
                {
					if((ledvillall[i / 8] & (1UL << (i % 8))) == 0)
                    {
						ledvilltimer[i] = ledvilltimeon[i];
					}
                    else
                    {
						ledvilltimer[i] = ledvilltimeoff[i];
					}
					ledvillall[i / 8] ^= (1UL << (i % 8));
					switch (i)
                    {
					case 19:
                    {	//Power led
						if((ledvill[LEDTOMBNUM] & 0b00000001) != 0)
                        {
							if((ledvillall[i / 8] & (1UL << (i % 8))) == 0)
                            {
								ledek[LEDTOMBNUM] &= 0b11111110;
							}
                            else
                            {
								ledek[LEDTOMBNUM] |= 0b00000001;
							}
						}
						if(ledvillcnt[GOMBNUM * 3 + 16] > 0)
                        {
							ledvillcnt[GOMBNUM * 3 + 16]--;
							if(ledvillcnt[GOMBNUM * 3 + 16] == 0)
                            {
								ledvill[LEDTOMBNUM] &= 0b11111110;
								if((ledall[LEDTOMBNUM] & 0b00000001)== 0)
                                {
									ledek[LEDTOMBNUM] &= 0b11111110;
								}
                                else
                                {
									ledek[LEDTOMBNUM] |= 0b00000001;
								}
							}
						}
						break;
					}
					case 20:
                    {	//H?tt?r led
						if((ledvill[LEDTOMBNUM] & 0b00000010) != 0)
                        {
							if((ledvillall[i / 8] & (0b00000001 << (i % 8))) == 0)
                            {
								ledek[LEDTOMBNUM] &= 0b11111101;
							}
                            else
                            {
								ledek[LEDTOMBNUM] |= 0b00000010;
							}
	
						}
						if(ledvillcnt[GOMBNUM * 3 + 16 + 1] > 0)
                        {
							ledvillcnt[GOMBNUM * 3 + 16 + 1]--;
							if(ledvillcnt[GOMBNUM * 3 + 16 + 1] == 0)
                            {
								ledvill[LEDTOMBNUM] &= 0b11111101;
								if((ledall[LEDTOMBNUM] & 0b00000010)== 0)
                                {
									ledek[LEDTOMBNUM] &= 0b11111101;
								}
                                else
                                {
									ledek[LEDTOMBNUM] |= 0b00000010;
								}
							}
						}
						break;
					}
					case 0 ... 15:{	//Vills?v+encoder led
						if((ledvill[(i / 8)] & (0b00000001<<(i % 8))) != 0)
                        {
							if((ledvillall[i / 8] & (0b00000001 << (i % 8))) == 0)
                            {
								ledek[(i / 8)] &= ((0b00000001<<(i % 8)) ^ 0xFF);
							}
                            else
                            {
								ledek[(i / 8)] |= (0b00000001<<(i % 8));
							}
						}
						if(ledvillcnt[i] > 0)
                        {
							ledvillcnt[i]--;
							if(ledvillcnt[i] == 0)
                            {
								ledvill[(i / 8)] &= ((0b00000001<<(i % 8)) ^ 0xFF);
								ledek[(i / 8)] &= ((0b00000001<<(i % 8)) ^ 0xFF);
								ledek[(i / 8)] |= ledall[(i / 8)] & (0b00000001<<(i % 8));	
							}
						}
						break;
					}
					case 16:
                    {	//Gombledek
						unsigned char j;
						for(j = 0;j < GOMBNUM;j++)
                        {
	
							if((ledvill[2 + (j / 2)] & (0b00000001<<(((j % 2) * 4) + i - 16))) != 0)
                            {
								if((ledvillall[i / 8] & (0b00000001 << (i % 8))) == 0)
                                {
									ledek[2 + (j / 2)] &= ((0b00000001<<(((j % 2) * 4) + i - 16)) ^ 0xFF);
								}
                                else
                                {
									ledek[2 + (j / 2)] |= (0b00000001<<(((j % 2) * 4) + i - 16));
								}
							}
							if(ledvillcnt[(j * 3) + i] > 0)
                            {
								ledvillcnt[(j * 3) + i]--;
								if(ledvillcnt[(j * 3) + i] == 0)
                                {
									ledvill[2 + (j / 2)] &= ((0b00000001<<(((j % 2) * 4) + i - 16)) ^ 0xFF);
									ledek[2 + (j / 2)] &= ((0b00000001<<(((j % 2) * 4) + i - 16)) ^ 0xFF);
									ledek[2 + (j / 2)] |= (ledall[2 + (j / 2)] & (0b00000001<<(((j % 2) * 4) + i - 16)));	
								}
							}
						}
						for(j = 0;j < (GOMBNUM >> 1);j++)
                        {
							a = ledek[j]  & 0b11101110;;
							a = a | (a >> 1);// | (a >> 2);
							a = a | (a << 1);// | (a << 2);
							a ^= 0xFF;
							derengkimsk[j] = a;
						}
						break;
					}
					case 17:
                    {	//Gombledek
						unsigned char j;
						for(j = 0;j < GOMBNUM;j++)
                        {
	
							if((ledvill[2 + (j / 2)] & (0b00000001<<(((j % 2) * 4) + i - 16))) != 0)
                            {
								if((ledvillall[i / 8] & (0b00000001 << (i % 8))) == 0)
                                {
									ledek[2 + (j / 2)] &= ((0b00000001<<(((j % 2) * 4) + i - 16)) ^ 0xFF);
								}
                                else
                                {
									ledek[2 + (j / 2)] |= (0b00000001<<(((j % 2) * 4) + i - 16));
								}
							}
							if(ledvillcnt[(j * 3) + i] > 0)
                            {
								ledvillcnt[(j * 3) + i]--;
								if(ledvillcnt[(j * 3) + i] == 0)
                                {
									ledvill[2 + (j / 2)] &= ((0b00000001<<(((j % 2) * 4) + i - 16)) ^ 0xFF);
									ledek[2 + (j / 2)] &= ((0b00000001<<(((j % 2) * 4) + i - 16)) ^ 0xFF);
									ledek[2 + (j / 2)] |= (ledall[2 + (j / 2)] & (0b00000001<<(((j % 2) * 4) + i - 16)));	
								}
							}
						}
						for(j = 0;j < (GOMBNUM >> 1);j++)
                        {
							a = ledek[j]  & 0b11101110;;
							a = a | (a >> 1);// | (a >> 2);
							a = a | (a << 1);// | (a << 2);
							a ^= 0xFF;
							derengkimsk[j] = a;
						}
						break;
					}
					case 18:
                    {	//Gombledek
						unsigned char j;
						for(j = 0;j < GOMBNUM;j++)
                        {
	
							if((ledvill[2 + (j / 2)] & (0b00000001<<(((j % 2) * 4) + i - 16))) != 0)
                            {
								if((ledvillall[i / 8] & (0b00000001 << (i % 8))) == 0)
                                {
									ledek[2 + (j / 2)] &= ((0b00000001<<(((j % 2) * 4) + i - 16)) ^ 0xFF);
								}
                                else
                                {
									ledek[2 + (j / 2)] |= (0b00000001<<(((j % 2) * 4) + i - 16));
								}
							}
							if(ledvillcnt[(j * 3) + i] > 0)
                            {
								ledvillcnt[(j * 3) + i]--;
								if(ledvillcnt[(j * 3) + i] == 0)
                                {
									ledvill[2 + (j / 2)] &= ((0b00000001<<(((j % 2) * 4) + i - 16)) ^ 0xFF);
									ledek[2 + (j / 2)] &= ((0b00000001<<(((j % 2) * 4) + i - 16)) ^ 0xFF);
									ledek[2 + (j / 2)] |= (ledall[2 + (j / 2)] & (0b00000001<<(((j % 2) * 4) + i - 16)));	
								}
							}
						}
						for(j = 0;j < (GOMBNUM >> 1);j++)
                        {
							a = ledek[j]  & 0b11101110;;
							a = a | (a >> 1);// | (a >> 2);
							a = a | (a << 1);// | (a << 2);
							a ^= 0xFF;
							derengkimsk[j] = a;
						}
						break;
					}
				}
			}
          }
        }
    }
}


void SendLedek(void)
{
	unsigned char i;
	unsigned char derengmsk;
	unsigned char ledpmask;
	uint8_t aledek[LEDTOMBNUM + 1];			//vills?v+encoder+Gombledek, pwr+h?tt?r
	    
	aledek[0] = ledek[0];
	aledek[1] = ledek[1];
	if(( pwmszaml & ledpwm[0] ) != 0){
		aledek[0] &= 0b11111110;
	}
	if(( pwmszaml & ledpwm[1] ) != 0){
		aledek[0] &= 0b11111101;
	}
	if(( pwmszaml & ledpwm[2] ) != 0){
		aledek[0] &= 0b11111011;
	}
	if(( pwmszaml & ledpwm[3] ) != 0){
		aledek[0] &= 0b11110111;
	}
	if(( pwmszaml & ledpwm[4] ) != 0){
		aledek[0] &= 0b11101111;
	}
	if(( pwmszaml & ledpwm[5] ) != 0){
		aledek[0] &= 0b11011111;
	}
	if(( pwmszaml & ledpwm[6] ) != 0){
		aledek[0] &= 0b10111111;
	}
	if(( pwmszaml & ledpwm[7] ) != 0){
		aledek[0] &= 0b01111111;
	}
	if(( pwmszaml & ledpwm[8] ) != 0){
		aledek[1] &= 0b11111110;
	}
	if(( pwmszaml & ledpwm[9] ) != 0){
		aledek[1] &= 0b11111101;
	}
	if(( pwmszaml & ledpwm[10] ) != 0){
		aledek[1] &= 0b11111011;
	}
	if(( pwmszaml & ledpwm[11] ) != 0){
		aledek[1] &= 0b11110111;
	}
	if(( pwmszaml & ledpwm[12] ) != 0){
		aledek[1] &= 0b11101111;
	}
	if(( pwmszaml & ledpwm[13] ) != 0){
		aledek[1] &= 0b11011111;
	}
	if(( pwmszaml & ledpwm[14] ) != 0){
		aledek[1] &= 0b10111111;
	}
	if(( pwmszaml & ledpwm[15] ) != 0){
		aledek[1] &= 0b01111111;
	}

	//Dereng?s
	if(( pwmszaml & ledpwm[24] ) == 0){
		aledek[0] = 0xFF;
		aledek[1] = 0xFF;
	}

	derengmsk = 0;
	if(( pwmszaml & ledpwm[21] ) == 0){
		derengmsk |= 0x11;
	}
	if(( pwmszaml & ledpwm[22] ) == 0){
		derengmsk |= 0x22;
	}
	if(( pwmszaml & ledpwm[23] ) == 0){
		derengmsk |= 0x44;
	}

	ledpmask = 0xFF;
	if(( pwmszaml & ledpwm[16] ) != 0){
		ledpmask &= 0b11101110;
	}
	if(( pwmszaml & ledpwm[17] ) != 0){
		ledpmask &= 0b11011101;
	}
	if(( pwmszaml & ledpwm[18] ) != 0){
		ledpmask &= 0b10111011;
	}

	for( i = 2; i < (GOMBNUM >> 1) + 2; i++ ){	//Gombledek
		aledek[i] = ((ledek[i] & ledpmask) | (derengmsk & derengkimsk[i]));
		//aledek[i] = ledek[i]; 
		//aledek[i] &= ledpmask;
		//aledek[i] |= (derengmsk & derengkimsk[i]);
		//Dereng?s
	}
	aledek[LEDTOMBNUM] = ledek[LEDTOMBNUM];		//pwr+h?tt?r
	if(( pwmszaml & ledpwm[19] ) != 0){
		aledek[LEDTOMBNUM] &= 0b11111110;
	}
	if(( pwmszaml & ledpwm[20] ) != 0){
		aledek[LEDTOMBNUM] &= 0b11111101;
	}
	
    //INTDisableInterrupts();

	pwmszaml--;
	if(pwmszaml == 0){
		pwmszaml = PWMSZAM;

	}
	if((fizicnum >= 22) && ((fizicnum <= 22))) {	//M36-ban eltolt led?ll?sok
	  aledek[ 0 ] = (aledek[ 0 ] & 0b11000000) | ((aledek[ 0 ] & 0b00011111) << 1) | ((aledek[ 0 ] & 0b00100000) / 32 );
	  aledek[ 1 ] = (aledek[ 1 ] & 0b11000000) | ((aledek[ 1 ] & 0b00011111) << 1) | ((aledek[ 1 ] & 0b00100000) / 32 );
	}	
}