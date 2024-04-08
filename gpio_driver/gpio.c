#include "gpio.h"



#define NUM_ROWS 8
#define NUM_COLS 3
#define NUM_BUTTONS 25

// Mapping from button array index to pattern index
uint8_t buttonMapping[NUM_BUTTONS] = {4, 20, 12, 5, 21, 13, 6, 22, 14, 7, 23, 15, 8, 24, 16, 1, 17, 9, 2, 18, 10, 3, 19, 11};
// Array to hold the button presses according to the pattern
uint8_t patternArray[NUM_BUTTONS];



volatile uint16_t* rowPinsTris[NUM_ROWS] = {&TRISB, &TRISC, &TRISC, &TRISC, &TRISC, &TRISB, &TRISB, &TRISB};
uint8_t rowPinsTrisBits[NUM_ROWS] = {9, 6, 7, 8, 9, 10, 11, 12};

volatile uint16_t* rowPinsLat[NUM_ROWS] = {&LATB, &LATC, &LATC, &LATC, &LATC, &LATB, &LATB, &LATB};
uint8_t rowPinsLatBits[NUM_ROWS] = {9, 6, 7, 8, 9, 10, 11, 12};

volatile uint16_t* colPinsPort[NUM_COLS] = {&PORTB, &PORTA, &PORTA};
uint8_t colPinsPortBits[NUM_COLS] = {13, 10, 7};

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

void remap_buttons(uint8_t *buttonarray)
{
    for (int i = 0; i < NUM_BUTTONS; i++)
    {
        // Subtract 1 from the mapping index because arrays are 0-based
        patternArray[buttonMapping[i] - 1] = buttonarray[i];
    }
}



static void rows_set_input(void)
{
    for (int i = 0; i < NUM_ROWS; i++)
    {
        // Set the specific bit of the TRIS register to 1
        *rowPinsTris[i] |= (1 << rowPinsTrisBits[i]);
    }
}
static void columns_set_pullup(void)
{
    CNPUBbits.CNPUB13   =       1;
    CNPUAbits.CNPUA7    =       1;
    CNPUAbits.CNPUA10   =       1;
}

void read_buttons(uint8_t *buttonarray)
{
    columns_set_pullup();
    rows_set_input();

    for (int row = 0; row < NUM_ROWS; row++)
    {
        // Set row to output
        *rowPinsLat[row] &= ~(1 << rowPinsLatBits[row]);

        // Pull low row so if any of the three buttons in the row is pressed they will pull column pins low
        *rowPinsLat[row] &= ~(1 << rowPinsLatBits[row]);

        // Set row back to input
        *rowPinsLat[row] |= (1 << rowPinsLatBits[row]);
    }
}