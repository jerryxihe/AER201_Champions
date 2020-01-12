/**
 * @file
 * @author Michael Ding
 * @author Tyler Gamvrelis
 *
 * Created on August 4, 2016, 3:22 PM
 *
 * @ingroup I2C
 */

/********************************* Includes **********************************/
#include "I2C.h"

/***************************** Private Functions *****************************/
/**
 * @brief Private function used to poll the MSSP module status. This function
 *        exits when the I2C module is idle.
 * @details The static keyword makes it so that files besides I2C.c cannot
 *          "see" this function
 */
static inline void I2C_Master_Wait(){
    // Wait while:
    //   1. A transmit is in progress (SSPSTAT & 0x04)
    //   2. A Start/Repeated Start/Stop/Acknowledge sequence has not yet been
    //      cleared by hardware
    while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F)){
        continue;
    }
}

/***************************** Public Functions ******************************/
void I2C_Master_Init(const unsigned long clockFreq){
    // Disable the MSSP module
    SSPCON1bits.SSPEN = 0;
    
    // Force data and clock pin data directions
    TRISCbits.TRISC3 = 1; // SCL (clock) pin
    TRISCbits.TRISC4 = 1; // SDA (data) pin
    
    // See section 17.4.6 in the PIC18F4620 datasheet for master mode details.
    // Below, the baud rate is configured by writing to the SSPADD<6:0>
    // according to the formula given on page 172
    SSPADD = (_XTAL_FREQ / (4 * clockFreq)) - 1;
    
    // See PIC18F4620 datasheet, section 17.4 for I2C configuration
    SSPSTAT = 0b10000000; // Disable slew rate control for cleaner signals

    // Clear errors & enable the serial port in master mode
    SSPCON1 = 0b00101000;

    // Set entire I2C operation to idle
    SSPCON2 = 0b00000000;
}

void I2C_Master_Start(void){    
    I2C_Master_Wait(); // Ensure I2C module is idle
    SSPCON2bits.SEN = 1; // Initiate Start condition
}

void I2C_Master_RepeatedStart(void){
    I2C_Master_Wait(); // Ensure I2C module is idle
    SSPCON2bits.RSEN = 1; // Initiate Repeated Start condition
}

void I2C_Master_Stop(void){
    I2C_Master_Wait(); // Ensure I2C module is idle
    SSPCON2bits.PEN = 1; // Initiate Stop condition
}

void I2C_Master_Write(unsigned byteToWrite){
    I2C_Master_Wait(); // Ensure I2C module is idle

    // Write byte to the serial port buffer for transmission
    SSPBUF = byteToWrite;
}

unsigned char I2C_Master_Read(unsigned char ackBit){
    I2C_Master_Wait(); // Ensure I2C module is idle
    SSPCON2bits.RCEN = 1; // Enable receive mode for I2C module

    I2C_Master_Wait(); // Wait until receive buffer is full

    // Read received byte from the serial port buffer
    unsigned char receivedByte = SSPBUF;

    I2C_Master_Wait(); // Ensure I2C module is idle
    SSPCON2bits.ACKDT = ackBit; // Acknowledge data bit
    SSPCON2bits.ACKEN = 1; // Initiate acknowledge bit transmission sequence

    return receivedByte;
}