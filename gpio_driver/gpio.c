#include "gpio.h"




void gpio_init(void)
{
    
    
    ANSELB = 0;
    ANSELC = 0;
    ANSELA = 0;
    TRISBbits.TRISB9 =      1;                  //sor1_3
    TRISCbits.TRISC6 =      1;                  //sor4_6
    TRISCbits.TRISC7 =      1;                  //sor7_9
    TRISCbits.TRISC8 =      1;                  //sor10_12
    TRISCbits.TRISC9 =      1;                  //sor13_15
    TRISBbits.TRISB10 =     1;                  //sor16_18
    TRISBbits.TRISB11 =     1;                  //sor19_21
    TRISBbits.TRISB12 =     1;                  //sor22_25
    
    TRISBbits.TRISB13 =     1;                  //OSZLOP_A
    TRISAbits.TRISA10 =     1;                  //OSZLOP_B
    TRISAbits.TRISA7 =      1;                  //OSZLOP_C
    
    
    TRISAbits.TRISA0 =      0;                  //VEZ1
    TRISAbits.TRISA1 =      0;                  //VEZ2
    TRISBbits.TRISB0 =      0;                  //VEZ3
    TRISBbits.TRISB1 =      0;                  //VEZ4
    
    TRISBbits.TRISB7 =      0;                  //display data/command
    TRISBbits.TRISB8 =      0;                  //display reset
    RPOR1bits.RP37R =       0b001000;           //display spi
    RPOR2bits.RP38R =       0b001001;           //display spi
    TRISBbits.TRISB5 =      0;                  //display spi
    TRISBbits.TRISB6 =      0;                  //display spi
}

void lcd_command_data_set(void)
{
    LATBbits.LATB7 = 1;
}
void lcd_command_data_reset(void)
{
    LATBbits.LATB7 = 0;
}
void lcd_reset_set(void)
{
    LATBbits.LATB8 = 1;
}
void lcd_reset_reset(void)
{
    LATBbits.LATB8 = 0;
}