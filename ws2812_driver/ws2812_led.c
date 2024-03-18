#include "ws2812_led.h"
#include "color.h"
#include <stdint.h>
#include "../delay.h"
#include "../dma_driver/dma.h"
#include "../spi_driver/spi.h"



uint8_t ledBuffer[LED_BUFFER_SIZE]; // Buffer to store the LED data

uint8_t ws2812_init_leds(LED *leds, uint8_t num_leds)
{
    // Initialize the LEDs
    for (uint8_t i = 0; i < num_leds; i++)
    {
        leds[i].R = 0;
        leds[i].G = 0;
        leds[i].B = 0;
        leds[i].brightness = 0;
    }
    ws2812_fill_buffer(leds, num_leds, ledBuffer); // Fill the buffer with the LED data
    dma_init(); // Initialize the DMA
    dma_set_buffer(ledBuffer, sizeof(ledBuffer)); // Set the buffer for the DMA
    spi_init();// Initialize the SPI
    dma_start(); // Start the DMA and send the data to the LEDs
    spi_enable(); // Enable the SPI
    __delay_ms(2);
    return 0;// Return 0 if successful
}
uint8_t ws2812_set_color_range(LED *leds, uint8_t num_leds, uint8_t start, uint8_t end, uint32_t color)
{
    // Check if the start and end indices are valid
    if (start < num_leds && end < num_leds && start <= end)
    {
        // Set the color of the LEDs in the range [start, end]
        for (uint8_t i = start; i <= end; i++)
        {
            leds[i].R = (color >> 16) & 0xFF;
            leds[i].G = (color >> 8) & 0xFF;
            leds[i].B = color & 0xFF;
        }
        return 0; // Return 0 if successful
    }
    else
    {
        return 1; // Return 1 if unsuccessful
    }
}
void ws2812_fill_buffer(LED *leds, uint8_t num_leds, uint8_t *buffer)
{
    int i, j, k;
    uint8_t color;

    buffer[0] = 0x00; // First byte is all 0

    // Iterate over each LED
    for(i = 0; i < num_leds; i++)
    {
        // Iterate over each color (R, G, B)
        for(j = 0; j < 3; j++)
        {
            switch(j)
            {
                case 0: color = leds[i].R; break; // Red
                case 1: color = leds[i].G; break; // Green
                case 2: color = leds[i].B; break; // Blue
            }

            // Iterate over each bit of the color value
            for(k = 0; k < 8; k++)
            {
                // Convert bit to bit representation and store in buffer
                if((color >> (7 - k)) & 0x01)
                    buffer[i*24 + j*8 + k + 1] = 0b11111100; // Bit is 1
                else
                    buffer[i*24 + j*8 + k + 1] = 0b10000000; // Bit is 0
            }
        }
    }
}
void ws2812_send_buffer(LED *leds, uint8_t num_leds)
{
    ws2812_fill_buffer(leds, num_leds, ledBuffer); // Fill the buffer with the LED data
    dma_set_buffer(ledBuffer, sizeof(ledBuffer)); // Set the buffer for the DMA
    dma_start(); // Start the DMA and send the data to the LEDs
}