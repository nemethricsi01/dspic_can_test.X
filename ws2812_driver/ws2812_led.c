#include "ws2812_led.h"
#include "color.h"
uint8_t init_leds(LED *leds, uint8_t num_leds)
{
    // Initialize the LEDs
    for (uint8_t i = 0; i < num_leds; i++)
    {
        leds[i].R = BLACK;
        leds[i].G = BLACK;
        leds[i].B = BLACK;
    }
    return 0;// Return 0 if successful
}
uint8_t ws2812_set_color(LED *leds, uint8_t num_leds, uint8_t led, uint32_t color)
{
    // Set the color of the LED
    if (led < num_leds)
    {
        leds[led].R = (color >> 16) & 0xFF;
        leds[led].G = (color >> 8) & 0xFF;
        leds[led].B = color & 0xFF;
        return 0;// Return 0 if successful
    }
    else
    {
        return 1;// Return 1 if unsuccessful
    }
}