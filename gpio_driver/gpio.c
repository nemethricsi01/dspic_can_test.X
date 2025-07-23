#include "gpio.h"



//these indexes shows the offsets where the buttons are located in the button array right after reading them
typedef enum {
    BTN_IC_PHONE = 1,
    BTN_SPEED_DIAL = 4,
    BTN_GROUP_1 = 7,
    BTN_GROUP_2 = 10,
    BTN_CALLBACK = 13,
    BTN_CALL = 16,
    BTN_END_CALL = 19,
    BTN_EXTRA = 22,
            
} ExoticButtonIndex;
typedef enum {
    BTN_1 = 15,
    BTN_2 = 18,
    BTN_3 = 21,
    BTN_4 = 0,
    BTN_5 = 3,
    BTN_6 = 6,
    BTN_7 = 9,
    BTN_8 = 12,
    BTN_9 = 17,
    BTN_10 = 20,
    BTN_11 = 23,
    BTN_12 = 2,
    BTN_13 = 5,
    BTN_14 = 8,
    BTN_15 = 11,
    BTN_16 = 14
} NormalButtonIndex;
uint8_t buttonMapping[NUM_BUTTONS] = {  
                                        BTN_1, BTN_2, BTN_3, BTN_4, BTN_5, BTN_6, BTN_7, BTN_8,
                                        BTN_9, BTN_10, BTN_11, BTN_12, BTN_13, BTN_14, BTN_15, BTN_16,
                                        BTN_CALL, BTN_END_CALL, BTN_EXTRA, BTN_IC_PHONE, BTN_SPEED_DIAL, BTN_GROUP_1, BTN_GROUP_2, BTN_CALLBACK
                                    };

volatile uint16_t* rowPinsTris[NUM_ROWS] = {&TRISB, &TRISC, &TRISC, &TRISC, &TRISC, &TRISB, &TRISB, &TRISB};
uint8_t rowPinsTrisBits[NUM_ROWS] = {9, 6, 7, 8, 9, 10, 11, 12};

volatile uint16_t* rowPinsLat[NUM_ROWS] = {&LATB, &LATC, &LATC, &LATC, &LATC, &LATB, &LATB, &LATB};
uint8_t rowPinsLatBits[NUM_ROWS] = {9, 6, 7, 8, 9, 10, 11, 12};

volatile uint16_t* colPinsPort[NUM_COLS] = {&PORTB, &PORTA, &PORTA};
uint8_t colPinsPortBits[NUM_COLS] = {13, 10, 7};





static void row_set_input(uint8_t row)
{

        // Set the specific bit of the TRIS register to 1
        *rowPinsTris[row] |= (1 << rowPinsTrisBits[row]);
}
static void row_set_output(uint8_t row)
{
        // Set the specific bit of the TRIS register to 0
        *rowPinsTris[row] &= ~(1 << rowPinsTrisBits[row]);
}

static void row_set_low(uint8_t row)
{
    // Set the specific bit of the LAT register to 0
    *rowPinsLat[row] &= ~(1 << rowPinsLatBits[row]);
}
static void row_set_high(uint8_t row)
{
    // Set the specific bit of the LAT register to 1
    *rowPinsLat[row] |= (1 << rowPinsLatBits[row]);
}
static void columns_set_pullup(void)
{
    CNPUBbits.CNPUB13   =       1;
    CNPUAbits.CNPUA7    =       1;
    CNPUAbits.CNPUA10   =       1;
}

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
    
    
    TRISAbits.TRISA0 =      1;                  //VEZ1
    TRISAbits.TRISA1 =      1;                  //VEZ2
    TRISBbits.TRISB0 =      0;                  //VEZ3, mute out pin
    TRISBbits.TRISB1 =      1;                  //VEZ4
    
    
    
    TRISBbits.TRISB7 =      0;                  //display data/command
    TRISBbits.TRISB8 =      0;                  //display reset
    TRISBbits.TRISB5 =      0;                  //display spi
    TRISBbits.TRISB6 =      0;                  //display spi


    
    
    TRISCbits.TRISC1 =      1; // CAN tx pin
    TRISCbits.TRISC2 =      1; // CAN rx pin
    
    TRISCbits.TRISC4 =      1; // Mute in pin
    TRISCbits.TRISC5 =      0; // piggy SS pin
    TRISBbits.TRISB15 =     1; // piggy MOSI pin

    TRISAbits.TRISA9 =      1; // piggy SCK pin
    TRISBbits.TRISB4 =      0; //Piggy MISO pin
 
    RPOR1bits.RP36R = 0b001000;//Piggy MISO pin
    
    
    RPINR22bits.SCK2R = 0b0011001;//rpi25 
    RPINR22bits.SDI2R = 0b0101111;//rpi47 

    columns_set_pullup();

}



void read_buttons(uint8_t *buttonarray)
{
    
    
    __builtin_disable_interrupts();
    for (int row = 0; row < NUM_ROWS; row++)
    {
        row_set_output(row);
        row_set_low(row);
        __delay_us(1);  // or use a few NOP instructions
         for(int col = 0; col < NUM_COLS; col++)
        {
            // Read the state of the column pin
            if (!(*colPinsPort[col] & (1 << colPinsPortBits[col])))
            {
                // If the column pin is low, set the corresponding button in the array
                buttonarray[row * NUM_COLS + col] = 1;
            }
            else
            {
                // If the column pin is high, clear the corresponding button in the array
                buttonarray[row * NUM_COLS + col] = 0;
            }
        }
        row_set_input(row);
        row_set_high(row);// Set the row back to high so it can be set to low again in the next iteration
        __delay_us(1);  // or use a few NOP instructions
    }
    uint8_t tempArray[NUM_BUTTONS] = {0};
    for (int i = 0; i < NUM_BUTTONS; i++)
    {
        tempArray[i] = buttonarray[buttonMapping[i]];
    }
    for (int i = 0; i < NUM_BUTTONS; i++)
    {
        buttonarray[i] = tempArray[i];
    }
    __builtin_enable_interrupts();
    
}

