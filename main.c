//
// main.c - Trinamic TMC2130 stepper driver I2C <> SPI bridge for up to 8 drivers
//
// Target: MSP430G2553
//
// v0.1 / 2018-11-11 / Io Engineering / Terje
//

/*

Copyright (c) 2018, Terje Io
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

· Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

· Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

· Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "string.h"

#include "config.h"
#include "trinamic/TMC2130_I2C_map.h"

typedef struct {
    uint8_t cs_pin;
    uint8_t addr;
    uint8_t payload[5];
} trinamic2130_t;

typedef enum {
    I2C_Idle,
    I2C_TXMode,
    I2C_TXData,
    I2C_TXStop,
    I2C_RXData,
    I2C_RXStop
} i2c_state;

typedef enum {
    CMD_Idle,
    CMD_ReadRegister,
    I2C_WriteRegister
} i2c_cmd;

typedef struct {
    volatile i2c_state state;
    volatile i2c_cmd cmd;
    uint16_t rx_count;
    uint8_t *rx_data;
    volatile uint16_t tx_count;
    uint8_t *tx_data;
    uint8_t data[5];
} i2c_trans_t;

static i2c_trans_t i2c;

static trinamic2130_t driver[3], *active;

static void writeRegister (void)
{
    uint8_t *payload = active->payload;

    CS_PORT_OUT &= ~active->cs_pin;

    __delay_cycles(2);

    while(UCA0STAT & UCBUSY);   // Wait for buffer
    UCA0TXBUF = *payload++;     // Transmit payload byte

    while(UCA0STAT & UCBUSY);   // Wait for buffer
    UCA0TXBUF = *payload++;     // Transmit payload byte

    while(UCA0STAT & UCBUSY);   // Wait for buffer
    UCA0TXBUF = *payload++;     // Transmit payload byte

    while(UCA0STAT & UCBUSY);   // Wait for buffer
    UCA0TXBUF = *payload++;     // Transmit payload byte

    while(UCA0STAT & UCBUSY);   // Wait for buffer
    UCA0TXBUF = *payload++;     // Transmit payload byte

    while(UCA0STAT & UCBUSY);   // Wait for transaction to complete
    UCA0RXBUF;                  // Dummy read to clear flags

    __delay_cycles(2);

    CS_PORT_OUT |= active->cs_pin;
}

static void readRegister (void)
{
    uint8_t *payload = active->payload;

    // Transmit dummy read transaction so driver can prepare response

    CS_PORT_OUT &= ~active->cs_pin;

    UCA0TXBUF = payload[0];     // Send register address, increment data pointer and
    while(UCA0STAT & UCBUSY);   // wait for transmission to finish

    UCA0TXBUF = 0;              // Send dummy payload data, increment data pointer and
    while(UCA0STAT & UCBUSY);   // wait for transmission to finish

    UCA0TXBUF = 0;              // Send dummy payload data, increment data pointer and
    while(UCA0STAT & UCBUSY);   // wait for transmission to finish

    UCA0TXBUF = 0;              // Send dummy payload data, increment data pointer and
    while(UCA0STAT & UCBUSY);   // wait for transmission to finish

    UCA0TXBUF = 0;              // Send dummy payload data, increment data pointer and
    while(UCA0STAT & UCBUSY);   // wait for transmission to finish

    __delay_cycles(2);

    CS_PORT_OUT |= active->cs_pin;

    __delay_cycles(2);

    // Transmit actual read transaction

    CS_PORT_OUT &= ~active->cs_pin;

    UCA0TXBUF = payload[0];     // Send register address, increment data pointer and
    while(UCA0STAT & UCBUSY);   // wait for transmission to finish
    *payload++ = UCA0RXBUF;     // Read and save response (status register)

    i2c.tx_count = 0;           // we now have valid data for I2C read to start...

    UCA0TXBUF = 0;              // Send dummy payload data, increment data pointer and
    while(UCA0STAT & UCBUSY);   // wait for transmission to finish
    *payload++ = UCA0RXBUF;     // Read and save response

    UCA0TXBUF = 0;              // Send dummy payload data, increment data pointer and
    while(UCA0STAT & UCBUSY);   // wait for transmission to finish
    *payload++ = UCA0RXBUF;     // Read and save response

    UCA0TXBUF = 0;              // Send dummy payload data, increment data pointer and
    while(UCA0STAT & UCBUSY);   // wait for transmission to finish
    *payload++ = UCA0RXBUF;     // Read and save response

    UCA0TXBUF = 0;              // Send dummy payload data, increment data pointer and
    while(UCA0STAT & UCBUSY);   // wait for transmission to finish
    *payload++ = UCA0RXBUF;     // Read and save response

    CS_PORT_OUT |= active->cs_pin;
}

void main (void)
{
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
    DCOCTL = CALDCO_16MHZ;      // Set DCO for 16 MHz using
    BCSCTL1 = CALBC1_16MHZ;     // calibration registers
//    BCSCTL2 &= ~(DIVS1|DIVS0);

    SPI_PORT_SEL |= MOSI_PIN|MISO_PIN|SCLK_PIN;
    SPI_PORT_SEL2 |= MOSI_PIN|MISO_PIN|SCLK_PIN;

    CS_PORT_DIR |= CS_PIN_MASK;

    driver[0].cs_pin = CS_PIN_X;
    driver[1].cs_pin = CS_PIN_Y;
    driver[2].cs_pin = CS_PIN_Z;

    active = &driver[0];

    SPI_CTL1 = UCSWRST|UCSSEL_2;
    SPI_CTL0 = UCCKPH|UCMSB|UCMST|UCMODE_0|UCSYNC;
    SPI_BR0 = 4;
    SPI_BR1 = 0;
    SPI_CTL1 &= ~UCSWRST;

    i2c.state = I2C_Idle;
    i2c.cmd = CMD_Idle;
    i2c.tx_count = 5; // Safe value in case a out-of-order read is issued

    I2C_PORT_SEL |= SDA_PIN|SDC_PIN;
    I2C_PORT_SEL2 |= SDA_PIN|SDC_PIN;

    IE2 = 0;                        // Disable TX interrupt
    UCB0CTL1 |= UCSWRST;            // Enable SW reset
    UCB0CTL0 = UCMODE_3 + UCSYNC;   // I2C Slave, synchronous mode
    UCB0I2COA = I2C_ADR_I2CBRIDGE;  // Set own address
    UCB0CTL1 &= ~UCSWRST;           // Clear SW reset, resume operation
    UCB0I2CIE |= UCSTPIE|UCSTTIE;   // Enable STP (Stop) and STT (Start) interrupts
    IE2 |= UCB0RXIE|UCB0TXIE;       // Enable RX interrupt

    _EINT();                        // Enable interrupts

    while(true) {

        LPM0;

        switch(i2c.cmd) {

            case CMD_ReadRegister:
                readRegister();
                i2c.cmd = CMD_Idle;
                break;

            case I2C_WriteRegister:
                writeRegister();
                i2c.cmd = CMD_Idle;
                break;
        }
    }
}

#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
    uint8_t ifg = IFG2, ie = IE2;

    if(IFG2 & UCB0TXIFG)
        UCB0TXBUF = (i2c.tx_count++ < 5) ? *i2c.tx_data++ : 0xFF;    // Transmit register content

    if(IFG2 & UCB0RXIFG) {
        if(i2c.rx_count < 5) {
            *i2c.rx_data++ = UCB0RXBUF;
            if(++i2c.rx_count == 1) {
                active = &driver[(i2c.data[0] >> 5) & 0x03];             // set active driver TODO: add range check?
                i2c.data[0] = TMC2130_I2C_regmap[(i2c.data[0] & 0x1FU)]; // restore original register address
                active->addr = (i2c.data[0] & 0x7F);
                if(!(i2c.data[0] & 0x80)) {
                    active->payload[0] = i2c.data[0];
                    active->payload[1] = 0;
                    active->payload[2] = 0;
                    active->payload[3] = 0;
                    active->payload[4] = 0;
                    i2c.tx_data = active->payload;
                    i2c.cmd = CMD_ReadRegister;
                    LPM0_EXIT;
                }
            }
        }
    }
}

#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
{
    uint8_t intstate = UCB0STAT;

    if(intstate & UCSTTIFG) {
        i2c.rx_count = 0;
        i2c.rx_data = i2c.data;
    }

    if(intstate & UCSTPIFG) {
        i2c.tx_count = 5; // Safe value in case a out-of-order read is issued
        if(i2c.rx_count == 5 && i2c.data[0] & 0x80) {
            memcpy(active->payload, i2c.data, sizeof(active->payload));
            i2c.cmd = I2C_WriteRegister;
            LPM0_EXIT;
        }
    }

    UCB0STAT &= ~intstate;      // Clear interrupt flags
}
