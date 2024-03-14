#ifndef WS2812_LED_H
#define WS2812_LED_H
#include <stdint.h>

// Define the struct
typedef struct {
    uint8_t R;
    uint8_t G;
    uint8_t B;
} LED;
// Create an array of 28 LEDs
LED leds[28];

uint8_t init_leds(LED *leds, uint8_t num_leds);
#endif // WS2812_LED_H