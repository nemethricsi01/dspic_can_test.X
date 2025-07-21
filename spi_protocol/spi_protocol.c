#include <xc.h>
#include <stdint.h>
#include "../spi_driver/spi.h"
#include "spi_protocol.h"
#include "../led_logic/led_logic.h"


#define TRUE 1
#define FALSE 0




unsigned char spi1checksum;
unsigned char spi1check;

unsigned char fizicnum = 14;


unsigned long actgomb[GOMBNUM/32 + 1];
unsigned long gomble[GOMBNUM/32 + 1];
unsigned long gombfel[GOMBNUM/32 + 1];
unsigned long gombdupla[GOMBNUM/32 + 1];
unsigned long gombhosszu[GOMBNUM/32 + 1];
unsigned long gomblend[GOMBNUM/32 + 1];
unsigned long gombfelnd[GOMBNUM/32 + 1];
unsigned long gomblet[GOMBNUM/32 + 1];
unsigned long gombfelt[GOMBNUM/32 + 1];
unsigned char encoder1 = 0;
unsigned char encoder2 = 0;
unsigned char encgomb = 0;
unsigned long lasta;
unsigned long acta;
unsigned char enctimer;

unsigned char gombcnt = 0;
unsigned char gombdtimer[GOMBNUM];
unsigned char gombhtimer[GOMBNUM];
unsigned char duplatime = DUPLATIMEC;
unsigned char hosszutime = HOSSZUTIMEC;

unsigned char spi1cnt;
unsigned char spi1toread;
unsigned char spi1rxbuf[SPI1RXBUFLEN];

uint8_t a;
uint8_t galap;
uint8_t gnum8 = GOMBNUM/8;;













uint8_t getledek(uint8_t index)
{
    return ledek[index];
}
uint8_t getdereng(uint8_t index)
{
    return ledek[index];
}


void process_spi(uint8_t received_byte)
{

		if(spi1cnt < SPI1RXBUFLEN)
		{
			spi1rxbuf[spi1cnt] = received_byte;
		}
				
		if(spi1cnt == 0)
		{
			spi1check = 0x55 ^ received_byte;
			switch (received_byte)
			{
				case 0x01:{	//Ki vagy?
					spi1toread = 2;
					break;
				}
				case 0x02:{	//Gombok?
					spi1toread = ((GOMBNUM / 8) * 7) + 3 + 1 + 2;
					spi1checksum = 0;
					break;
				}
				case 0x03:{	//Gombnyom?shosszok
					spi1toread = 4;
					break;
				}
				case 0x04:{	//LED
					spi1toread = 5;
					break;
				}
				case 0x05:{	//Villtimer
					spi1toread = 7;
					break;
				}
				case 0x06:{	//Ledpwm
					spi1toread = 4;
					break;
				}
				case 0x07:{	//ICSzam
					spi1toread = 3;
					break;
				}
				case 0x10:{	//Vills?v
					spi1toread = 7;
					break;
				}
				case 0x41:{	//VilltimerOn!
					spi1toread = 2;
					ledvilltmrenabled = TRUE;
					break;
				}
				case 0x42:{	//VilltimerOff!
					spi1toread = 2;
					ledvilltmrenabled = FALSE;
					break;
				}
				default:{
					spi1toread = 2;
					break;
				}
			}
		}
		switch (spi1rxbuf[0])
		{
			case 0x02:
			{	//Gombok?
				switch (spi1cnt){
					case 0 ... ((GOMBNUM / 8) - 1):{
//                        LATCbits.LATC4 ^= 1;
						a = (actgomb[(spi1cnt) / 4] >> (((spi1cnt) % 4) * 8));
						SPI2BUF = a;
                        
						spi1checksum += a;
						break;
					}
					case (GOMBNUM / 8) ... (((GOMBNUM / 8) * 2) - 1):{
						galap = gnum8;
						a = (gomble[(spi1cnt - (galap)) / 4] >> (((spi1cnt - (galap)) % 4) * 8));
						SPI2BUF = a;
						spi1checksum += a;
						gomble[(spi1cnt - (galap)) / 4] &= ((0xFFUL << (((spi1cnt - (galap)) % 4) * 8)) ^0xFFFFFFFF);
						break;
					}
					case ((GOMBNUM / 8) * 2) ... (((GOMBNUM / 8) * 3) - 1):{
						galap = gnum8 * 2;
						a = (gombfel[(spi1cnt - (galap)) / 4] >> (((spi1cnt - (galap)) % 4) * 8));
						SPI2BUF = a;
						spi1checksum += a;
						gombfel[(spi1cnt - (galap)) / 4] &= ((0xFFUL << (((spi1cnt - (galap)) % 4) * 8)) ^0xFFFFFFFF);
						break;
					}
					case ((GOMBNUM / 8) * 3) ... (((GOMBNUM / 8) * 4) - 1):{
						galap = gnum8 * 3;
						a = (gombdupla[(spi1cnt - (galap)) / 4] >> (((spi1cnt - (galap)) % 4) * 8));
						SPI2BUF = a;
						spi1checksum += a;
						gombdupla[(spi1cnt - (galap)) / 4] &= ((0xFFUL << (((spi1cnt - (galap)) % 4) * 8)) ^0xFFFFFFFF);
						break;
					}
					case ((GOMBNUM / 8) * 4) ... (((GOMBNUM / 8) * 5) - 1):{
						galap = gnum8 * 4;
						a = (gombhosszu[(spi1cnt - (galap)) / 4] >> (((spi1cnt - (galap)) % 4) * 8));
						SPI2BUF = a;
						spi1checksum += a;
						gombhosszu[(spi1cnt - (galap)) / 4] &= ((0xFFUL << (((spi1cnt - (galap)) % 4) * 8)) ^0xFFFFFFFF);
						break;
					}
					case ((GOMBNUM / 8) * 5) ... (((GOMBNUM / 8) * 6) - 1):{
						galap = gnum8 * 5;
						a = (gomblend[(spi1cnt - (galap)) / 4] >> (((spi1cnt - (galap)) % 4) * 8));
						SPI2BUF = a;
						spi1checksum += a;
						gomblend[(spi1cnt - (galap)) / 4] &= ((0xFFUL << (((spi1cnt - (galap)) % 4) * 8)) ^0xFFFFFFFF);
						break;
					}
					case ((GOMBNUM / 8) * 6) ... (((GOMBNUM / 8) * 7) - 1):{
						galap = gnum8 * 6;
						a = (gombfelnd[(spi1cnt - (galap)) / 4] >> (((spi1cnt - (galap)) % 4) * 8));
						SPI2BUF = a;
						spi1checksum += a;
						gombfelnd[(spi1cnt - (galap)) / 4] &= ((0xFFUL << (((spi1cnt - (galap)) % 4) * 8)) ^0xFFFFFFFF);
						break;
					}
					case ((GOMBNUM / 8) * 7):{
						a = encoder1;
						SPI2BUF = a;
						spi1checksum += a;
						break;
					}
					case (((GOMBNUM / 8) * 7) + 1):{
						a = encoder2;
						SPI2BUF = a;
						spi1checksum += a;
						break;
					}
					case (((GOMBNUM / 8) * 7) + 2):{
						a = encgomb;
						SPI2BUF = a;
						spi1checksum += a;
						break;
					}
					case (((GOMBNUM / 8) * 7) + 2 + 1):{
						SPI2BUF = spi1checksum;
						break;
					}
									}
				break;
			}
			case 0x07:{	//ICSzam?
				switch (spi1cnt){
					case 0: {
                        
						SPI2BUF = fizicnum;
						break;
					}
				}
			}
			default:{
				if(spi1cnt < spi1toread - 1){
				//	SPI2BUF = 0;
				}
				break;
			}
		}
		spi1cnt++;

		if(spi1cnt == spi1toread - 1){
			SPI2BUF = spi1check;
		}
										
		if(spi1cnt == spi1toread){
			SPI2BUF = PANEL;
			spi1cnt = 0;
			switch (spi1rxbuf[0]){
				case 0x03:{	//Gombnyom?shosszok
					duplatime = spi1rxbuf[1];
					hosszutime = spi1rxbuf[2];
					break;
				}
				case 0x04:{	//LED
                    
					unsigned int ledmnum;
					if(spi1rxbuf[1] < 16){
						ledmnum = spi1rxbuf[1];
					}else if(spi1rxbuf[1] < (16 + 192)){
						ledmnum = 16 + (((spi1rxbuf[1] - 16) / 3) * 4) + ((spi1rxbuf[1] - 16) % 3);
					}else if(spi1rxbuf[1] >= (16 + 192)){
					    ledmnum = 256 + (spi1rxbuf[1] - 192);
					}

					if((spi1rxbuf[2] & 0b00000001) != 0) {
						ledall[ledmnum / 8] |= ((1UL << (ledmnum % 8)));
						if((spi1rxbuf[2] & 0b00000010) == 0){
							ledek[ledmnum / 8] |= ((1UL << (ledmnum % 8)));
							a = ledek[ledmnum / 8]  & 0b11101110;;
							a = a | (a >> 1);// | (a >> 2);
							a = a | (a << 1);// | (a << 2);
							a ^= 0xFF;
							derengkimsk[ledmnum / 8] = a;
						}
					}else{
						ledall[ledmnum / 8] &= ((1UL << (ledmnum % 8)) ^ 0xFF);
						if((spi1rxbuf[2] & 0b00000010) == 0){
							ledek[ledmnum / 8] &= ((1UL << (ledmnum % 8)) ^ 0xFF);
							a = ledek[ledmnum / 8]  & 0b11101110;;
							a = a | (a >> 1);// | (a >> 2);
							a = a | (a << 1);// | (a << 2);
							a ^= 0xFF;
							derengkimsk[ledmnum / 8] = a;
						}
					}
					if((spi1rxbuf[2] & 0b00000010) != 0){
						ledvill[ledmnum / 8] |= ((1UL << (ledmnum % 8)));
					}else{
						ledvill[ledmnum / 8] &= ((1UL << (ledmnum % 8)) ^ 0xFF);
					}

					ledvillcnt[spi1rxbuf[1]] = spi1rxbuf[3];

					break;
				}
				case 0x05:{	//Timer
					ledvilltimer[spi1rxbuf[1]] = spi1rxbuf[2];
					ledvilltimeon[spi1rxbuf[1]] = spi1rxbuf[3];
					ledvilltimeoff[spi1rxbuf[1]] = spi1rxbuf[4];
					if(spi1rxbuf[5] == 0){
						ledvillall[spi1rxbuf[1] / 8] &= ((1UL << (spi1rxbuf[1] % 8)) ^ 0xFF);
					}else{
						ledvillall[spi1rxbuf[1] / 8] |= ((1UL << (spi1rxbuf[1] % 8)));
					}
					break;
				}
				case 0x06:{	//Ledpwm
					ledpwm[spi1rxbuf[1]] = spi1rxbuf[2];
					break;
				}
				case 0x10:{//Vills?v
					if((spi1rxbuf[1] & 0b00000001) != 0) {
						ledall[0] |= 0x3F;
						ledall[1] |= 0x3F;
						if((spi1rxbuf[1] & 0b00000010) == 0){
							ledek[0] |= 0x3F;
							ledek[1] |= 0x3F;
						}
					}else{
						ledall[0] &= 0xC0;
						ledall[1] &= 0xC0;
						if((spi1rxbuf[1] & 0b00000010) == 0){
							ledek[0] &= 0xC0;
							ledek[1] &= 0xC0;
						}
					}
					if((spi1rxbuf[1] & 0b00000010) == 0){
						ledvill[0] &= 0xC0;
						ledvill[1] &= 0xC0;
					}else{
						ledvill[0] |= 0x3F;
						ledvill[1] |= 0x3F;
					}
					if((spi1rxbuf[1] & 0b00000100) == 0){
						ledvillall[0] &= 0xC0;
						ledvillall[1] &= 0xC0;
					}else{
						ledvillall[0] |= 0x3F;
						ledvillall[1] |= 0x3F;
					}
					ledvilltimer[0] = spi1rxbuf[2];
					ledvilltimeon[0] = spi1rxbuf[3];
					ledvilltimeoff[0] = spi1rxbuf[4];
					ledvillcnt[0] = spi1rxbuf[5];
					ledvilltimer[1] = spi1rxbuf[2];
					ledvilltimeon[1] = spi1rxbuf[3];
					ledvilltimeoff[1] = spi1rxbuf[4];
					ledvillcnt[1] = spi1rxbuf[5];
					ledvilltimer[2] = spi1rxbuf[2];
					ledvilltimeon[2] = spi1rxbuf[3];
					ledvilltimeoff[2] = spi1rxbuf[4];
					ledvillcnt[2] = spi1rxbuf[5];
					ledvilltimer[3] = spi1rxbuf[2];
					ledvilltimeon[3] = spi1rxbuf[3];
					ledvilltimeoff[3] = spi1rxbuf[4];
					ledvillcnt[3] = spi1rxbuf[5];
					ledvilltimer[4] = spi1rxbuf[2];
					ledvilltimeon[4] = spi1rxbuf[3];
					ledvilltimeoff[4] = spi1rxbuf[4];
					ledvillcnt[4] = spi1rxbuf[5];
					ledvilltimer[5] = spi1rxbuf[2];
					ledvilltimeon[5] = spi1rxbuf[3];
					ledvilltimeoff[5] = spi1rxbuf[4];
					ledvillcnt[5] = spi1rxbuf[5];
					ledvilltimer[8] = spi1rxbuf[2];
					ledvilltimeon[8] = spi1rxbuf[3];
					ledvilltimeoff[8] = spi1rxbuf[4];
					ledvillcnt[8] = spi1rxbuf[5];
					ledvilltimer[9] = spi1rxbuf[2];
					ledvilltimeon[9] = spi1rxbuf[3];
					ledvilltimeoff[9] = spi1rxbuf[4];
					ledvillcnt[9] = spi1rxbuf[5];
					ledvilltimer[10] = spi1rxbuf[2];
					ledvilltimeon[10] = spi1rxbuf[3];
					ledvilltimeoff[10] = spi1rxbuf[4];
					ledvillcnt[10] = spi1rxbuf[5];
					ledvilltimer[11] = spi1rxbuf[2];
					ledvilltimeon[11] = spi1rxbuf[3];
					ledvilltimeoff[11] = spi1rxbuf[4];
					ledvillcnt[11] = spi1rxbuf[5];
					ledvilltimer[12] = spi1rxbuf[2];
					ledvilltimeon[12] = spi1rxbuf[3];
					ledvilltimeoff[12] = spi1rxbuf[4];
					ledvillcnt[12] = spi1rxbuf[5];
					ledvilltimer[13] = spi1rxbuf[2];
					ledvilltimeon[13] = spi1rxbuf[3];
					ledvilltimeoff[13] = spi1rxbuf[4];
					ledvillcnt[13] = spi1rxbuf[5];
					break;
				}
			}
		}      					
	}