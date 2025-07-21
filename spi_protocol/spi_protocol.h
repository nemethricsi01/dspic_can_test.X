/* 
 * File:   spi_protocol.h
 * Author: nemet
 *
 * Created on July 17, 2025, 11:16 AM
 */

#ifndef SPI_PROTOCOL_H
#define	SPI_PROTOCOL_H

#include <xc.h>
#include <stdint.h>
#include "../led_logic/led_logic.h"


#define PANEL	0x01

#define SPI1RXBUFLEN	0x07
#define	GOMBTIME	100
#define	ALAP		0
#define	JOBBRA		1
#define	BALRA		2
#define MAXARAM		512
#define STARTTIMERMAX	15000
#define STOPTIMER2MAX	10000

#define PWMSZAM		64
#define TMRSZAM		50



#define DUPLATIMEC	12
#define HOSSZUTIMEC	50

#define ENCTIME	2



#define SPIRXBUF_SIZE 7 // Size of the SPI receive buffer

extern uint8_t spi2rxbuf[SPIRXBUF_SIZE];// Buffer for SPI2 received data
extern uint8_t spi2rxbuf_index;// Index for the SPI2 receive buffers


extern uint8_t update_leds;







extern unsigned long actgomb[GOMBNUM/32 + 1];
extern unsigned long gomble[GOMBNUM/32 + 1];
extern unsigned long gombfel[GOMBNUM/32 + 1];
extern unsigned long gombdupla[GOMBNUM/32 + 1];
extern unsigned long gombhosszu[GOMBNUM/32 + 1];
extern unsigned long gomblend[GOMBNUM/32 + 1];
extern unsigned long gombfelnd[GOMBNUM/32 + 1];
extern unsigned long gomblet[GOMBNUM/32 + 1];
extern unsigned long gombfelt[GOMBNUM/32 + 1];
extern unsigned char encoder1;
extern unsigned char encoder2;
extern unsigned char encgomb;
extern unsigned long lasta;
extern unsigned long acta;
extern unsigned char enctimer;

extern unsigned char gombcnt;
extern unsigned char gombdtimer[GOMBNUM];
extern unsigned char gombhtimer[GOMBNUM];
extern unsigned char duplatime;
extern unsigned char hosszutime;








void process_spi(uint8_t received_byte);







uint8_t getledek(uint8_t index);
uint8_t getdereng(uint8_t index);


#endif	/* SPI_PROTOCOL_H */

