/**
 * @file
 * @author Michael Ding
 * @author Tyler Gamvrelis
 *
 * Created summer 2016
 *
 * @defgroup I2C
 * @brief I2C driver
 * @{
 */

#ifndef I2C_H
#define I2C_H

/********************************* Includes **********************************/
#include <xc.h>
#include <configBits.h>

/********************************** Macros ***********************************/
// These mean different things depending on the context, see "Understanding the
// I2C bus" by Texas Instruments for more details
#define ACK  0 /**< Acknowledge     */
#define NACK 1 /**< Not acknowledge */

/************************ Public Function Prototypes *************************/
/**
 * @brief Initializes the MSSP module for I2C mode. All configuration register
 *        bits are written to because operating in SPI mode could change them
 * @param clockFreq The frequency at which data is to be transferred via the
 *        I2C bus
 * @note The argument is used to generate the baud rate according to the
 *       formula clock = FOSC / (4 * (SSPADD + 1)). Because the argument
 *       sets the 7 bits of control signals in the SSPADD register, the
 *       following are the limitations on the value of clockFreq for
 *       FOSC = 40 MHz: Minimum: 78125, Maximum: 10000000
 */
void I2C_Master_Init(const unsigned long clockFreq);

/**
 * @brief Initiates Start condition on SDA and SCL pins. Automatically cleared
 *        by hardware
 */
void I2C_Master_Start(void);

/**
 * @brief Initiates Repeated Start condition on SDA and SCL pins. Automatically
 *        cleared by hardware
 */
void I2C_Master_RepeatedStart(void);

/**
 * @brief Initiates Stop condition on SDA and SCL pins. Automatically cleared
 *        by hardware
 */
void I2C_Master_Stop(void);

/** @brief Writes a byte to the slave device currently being addressed */
void I2C_Master_Write(unsigned byteToWrite);

/**
 * @brief Reads a byte from the slave device currently being addressed
 * @param ackBit The acknowledge bit
 *        -# ackBit == 0 --> acknowledge bit sent; ready for next bit
 *        -# ackBit == 1 --> no acknowledge bit (NACK); done reading data
 * @return The byte received
 */
unsigned char I2C_Master_Read(unsigned char ackBit);

/**
 * @}
 */

#endif /* I2C_H */