#ifndef SPI_H
#define SPI_H

/**
 * @brief Initializes the SPI.
 *
 * This function initializes the SPI by setting the master mode, secondary prescale,
 * clock edge select, input data sample phase, clock polarity, primary prescale,
 * word/byte mode, enhanced buffer, framed mode, and interrupt mode.
 * 
 * The enhanced buffer mode is important because without it the interrupt after one transfer will not be generated.
 * SISEL is set to 100 because we want to generate an interrupt when the last word is shifted out of the SPIxSR and the transmit is complete.
 */
void spi_init(void);
/**
 * @brief Enables the SPI.
 *
 * This function enables the SPI by clearing the interrupt flag, enabling the interrupt, and enabling the SPI module.
 */
void spi_enable(void);




void spi2_init(void);

void spi2_enable(void);
#endif // SPI_H