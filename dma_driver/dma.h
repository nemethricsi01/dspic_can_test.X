#ifndef DMA_H
#define DMA_H
#include <stdint.h>
/**
 * @brief Initializes the DMA.
 *
 * This function initializes the DMA by setting the size of the data to be transferred,
 * the addressing mode, the direction of the transfer, the operating mode, the starting address
 * of the DMA RAM, the address of the peripheral to be associated with DMA Channel 0, the DMA
 * transfer count, the DMA Channel 0 IRQ Select bits, and enabling the DMA channel.
 */
void dma_init(void);
/**
 * @brief Starts the DMA transfer.
 *
 * This function starts the DMA transfer by enabling the DMA channel and forcing the DMA transfer.
 */
void dma_start(void);
/**
 * @brief Sets the buffer for the DMA.
 *
 * This function sets the buffer for the DMA by setting the starting address of the DMA RAM
 * to the buffer address and the DMA transfer count to the length of the buffer minus one.
 *
 * @param buffer Pointer to the buffer to be associated with the DMA.
 * @param length The length of the buffer.
 */
void dma_set_buffer(uint8_t *buffer, uint16_t length);
// Declare your functions and classes here
void dma1_init(void);
void dma1_set_buffer(uint8_t *buffer, uint16_t length);
void dma1_start(void);
#endif // DMA_H