#ifndef WS2812_LED_H
#define WS2812_LED_H
#include <stdint.h>

// Define the struct
typedef struct {
    uint8_t R;
    uint8_t G;
    uint8_t B;
    uint8_t brightness;

} LED;
#define NUM_LEDS 30 // Number of LEDs
#define LED_BUFFER_SIZE (1 + 24 * NUM_LEDS) // 1: First byte is all 0, 24: 8 bits for each color (R, G, B) for each LED

/**
 * @brief Initializes the WS2812 LEDs.
 *
 * This function initializes the WS2812 LEDs by setting the RGB and brightness values of each LED to 0.
 * It then fills the buffer with the LED data, initializes the DMA and SPI, sets the buffer for the DMA,
 * starts the DMA, enables the SPI, and finally delays for 2 milliseconds.
 *
 * @param leds Pointer to an array of LED structures.
 * @param num_leds The number of LEDs in the array.
 * @return Returns 0 if successful.
 */
uint8_t ws2812_init_leds(LED *leds, uint8_t num_leds);

/**
 * Sets the color of a range of LEDs.
 *
 * This function sets the color of a range of LEDs specified by the start and end indices.
 *
 * @param leds      Pointer to the array of LED structures.
 * @param num_leds  Number of LEDs in the array.
 * @param start     Index of the first LED in the range.
 * @param end       Index of the last LED in the range.
 * @param color     Color value to set for the LEDs in the range.
 *
 * @return          The status of the operation. Returns 0 if successful, otherwise returns an error code.
 */
uint8_t ws2812_set_color_range(LED *leds, uint8_t num_leds, uint8_t start, uint8_t end, uint32_t color);


/**
 * @brief Sends the LED data to the LEDs.
 *
 * This function fills the buffer with the LED data using the `ws2812_fill_buffer` function.
 * It then sets the buffer for the DMA using the `dma_set_buffer` function and starts the DMA
 * to send the data to the LEDs using the `dma_start` function.
 *
 * @param leds Pointer to an array of LED structures.
 * @param num_leds The number of LEDs in the array.
 */
void ws2812_send_buffer(LED *leds, uint8_t num_leds);

/**
 * @brief Fills the buffer with the LED data.
 *
 * This function fills the buffer with the LED data. The first byte of the buffer is set to 0.
 * This was needed because the first bytes msb is longer than the rest of the bits.
 * Then, for each LED, it iterates over each color (R, G, B) and each bit of the color value.
 * It converts each bit to its bit representation and stores it in the buffer.
 *
 * @param leds Pointer to an array of LED structures.
 * @param num_leds The number of LEDs in the array.
 * @param buffer Pointer to the buffer to be filled with the LED data.
 */ 
void ws2812_fill_buffer(LED *leds, uint8_t num_leds, uint8_t *buffer);

#endif // WS2812_LED_H