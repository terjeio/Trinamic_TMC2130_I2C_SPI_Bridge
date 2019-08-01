//
// main.c - Trinamic TMC2130 stepper driver I2C <> SPI bridge for up to 8 drivers (currently supports 3)
//
// Target: MSP430G2553
//
// v0.0.3 / 2019-07-23 / Io Engineering / Terje
//

/*

Copyright (c) 2018-2019, Terje Io
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

// http://e2e.ti.com/support/microcontrollers/msp430/f/166/t/369949

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "string.h"

#include "config.h"
#include "trinamic/TMC2130_I2C_map.h"

typedef struct {
    uint8_t otpw_count;
    uint8_t otpw_max;
    uint8_t cs_pin;
    TMC2130_datagram_t reg;
    TMC2130_drv_status_dgr_t status;
    TMC2130_status_t response;
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
    CMD_Idle = 0,
    CMD_ReadRegister = 0x01,
    I2C_WriteRegister = 0x02,
    I2C_WriteEnable = 0x04,
    I2C_Diag1Event = 0x08,
    I2C_PollEvent = 0x10 // every 500 mS
} i2c_cmd;

typedef struct {
    volatile i2c_state state;
    volatile i2c_cmd cmd;
    uint16_t rx_count;
    uint8_t *rx_data;
    volatile uint16_t tx_count;
    volatile bool rx_pending;
    uint8_t *tx_data;
    uint8_t data[5];
} i2c_trans_t;

static i2c_trans_t i2c;
static trinamic2130_t driver[NUM_AXIS], *active;
static tmc_axes_t diag1_event = {0};
static TMCI2C_status_t drvstat = {0};
static TMCI2C_monitor_status_reg_t diag_data = {0};
static TMCI2C_enable_dgr_t axes = {
   .addr = TMC_I2CReg_ENABLE,
   .reg.value = 0
};

// Notify host about DIAG1 event
static void diag1Event (void)
{
    DIAG1_IRQ_DIR |= DIAG1_IRQ_PIN;
    DIAG1_IRQ_OUT &= ~DIAG1_IRQ_PIN;

    drvstat.stalled.mask = diag1_event.mask;

    diag1_event.mask = 0;

    __delay_cycles(32); // ~1uS IRQ pulse

    DIAG1_IRQ_OUT |= DIAG1_IRQ_PIN;
    DIAG1_IRQ_DIR &= ~DIAG1_IRQ_PIN;
}

// Write local driver enable outputs and monitoring mask
static void writeEnable (trinamic2130_t *driver)
{
    // Byte reverse data value as endianess differ
    // using axes.reg.value = driver->reg.payload.value fails
    axes.reg.data[0] = driver->reg.payload.data[3];
    axes.reg.data[1] = driver->reg.payload.data[2];
    axes.reg.data[2] = driver->reg.payload.data[1];
    axes.reg.data[3] = driver->reg.payload.data[0];

    if(axes.reg.enable.x)
        ENA_PORT_OUT &= ~ENA_PIN_X;
    else
        ENA_PORT_OUT |= ENA_PIN_X;

    if(axes.reg.enable.y)
        ENA_PORT_OUT &= ~ENA_PIN_Y;
    else
        ENA_PORT_OUT |= ENA_PIN_Y;

    if(axes.reg.enable.z)
        ENA_PORT_OUT &= ~ENA_PIN_Z;
    else
        ENA_PORT_OUT |= ENA_PIN_Z;

    __delay_cycles(200);
}

// Write driver register
static void writeRegister (trinamic2130_t *driver)
{
    uint8_t *payload = (uint8_t *)&(driver->reg.payload.data);

    CS_PORT_OUT &= ~driver->cs_pin;

    __delay_cycles(2);

    while(UCA0STAT & UCBUSY);
    UCA0TXBUF = driver->reg.addr.value;

    while(UCA0STAT & UCBUSY);
    UCA0TXBUF = *payload++;

    while(UCA0STAT & UCBUSY);
    UCA0TXBUF = *payload++;

    while(UCA0STAT & UCBUSY);
    UCA0TXBUF = *payload++;

    while(UCA0STAT & UCBUSY);
    UCA0TXBUF = *payload++;

    while(UCA0STAT & UCBUSY);
    UCA0RXBUF;

    __delay_cycles(2);

    CS_PORT_OUT |= driver->cs_pin;
}

// Read TMC or local register and transmit back to master
// NOTE: register structs are not readable by debugger due to differing endianness
static void readRegister (trinamic2130_t *driver)
{
    uint8_t *payload = (uint8_t *)&(driver->reg);

    if(driver->reg.addr.reg == TMC_I2CReg_MON_STATE) {

        i2c.data[0] = drvstat.value;
        i2c.data[1] = diag_data.data[3];
        i2c.data[2] = diag_data.data[2];
        i2c.data[3] = diag_data.data[1];
        i2c.data[4] = diag_data.data[0];
        i2c.tx_data = i2c.data;
        i2c.tx_count = 0;           // we now have valid data for I2C read to start...
        UCB0I2CIE |= UCSTTIE;

    } else {

        // Transmit dummy read transaction so driver can prepare response

        CS_PORT_OUT &= ~driver->cs_pin;

        UCA0TXBUF = driver->reg.addr.value;
        while(UCA0STAT & UCBUSY);

        UCA0TXBUF = 0;
        while(UCA0STAT & UCBUSY);

        UCA0TXBUF = 0;
        while(UCA0STAT & UCBUSY);

        UCA0TXBUF = 0;
        while(UCA0STAT & UCBUSY);

        UCA0TXBUF = 0;
        while(UCA0STAT & UCBUSY);

        __delay_cycles(2);

        CS_PORT_OUT |= driver->cs_pin;

        __delay_cycles(2);

        // Transmit actual read transaction

        CS_PORT_OUT &= ~driver->cs_pin;

        UCA0TXBUF = driver->reg.addr.value;
        while(UCA0STAT & UCBUSY);
        *payload++ = UCA0RXBUF;

//        i2c.tx_count = 0; // We now have valid data for I2C read to start, I2C is slower than SPI so it will not catch up
        UCB0I2CIE |= UCSTTIE;

        UCA0TXBUF = 0;
        while(UCA0STAT & UCBUSY);
        *payload++ = UCA0RXBUF;

        UCA0TXBUF = 0;
        while(UCA0STAT & UCBUSY);
        *payload++ = UCA0RXBUF;

        UCA0TXBUF = 0;
        while(UCA0STAT & UCBUSY);
        *payload++ = UCA0RXBUF;

        UCA0TXBUF = 0;
        while(UCA0STAT & UCBUSY);
        *payload++ = UCA0RXBUF;

        CS_PORT_OUT |= driver->cs_pin;
    }
}

// Read TMC register byte reversed for internal use
static void readRegisterBR (trinamic2130_t *driver, TMC2130_datagram_t *datagram)
{
    uint8_t *payload = (uint8_t *)&(datagram->payload) + 3;

    if(datagram->addr.reg == TMC_I2CReg_ENABLE) {

        i2c.tx_data = (uint8_t *)&axes.reg.value;
        i2c.tx_count = 0;           // we now have valid data for I2C read to start...

    } else {

        // Transmit dummy read transaction so driver can prepare response

        CS_PORT_OUT &= ~driver->cs_pin;

        UCA0TXBUF = datagram->addr.value;
        while(UCA0STAT & UCBUSY);

        UCA0TXBUF = 0;
        while(UCA0STAT & UCBUSY);

        UCA0TXBUF = 0;
        while(UCA0STAT & UCBUSY);

        UCA0TXBUF = 0;
        while(UCA0STAT & UCBUSY);

        UCA0TXBUF = 0;
        while(UCA0STAT & UCBUSY);

        __delay_cycles(2);

        CS_PORT_OUT |= driver->cs_pin;

        __delay_cycles(2);

        // Transmit actual read transaction

        CS_PORT_OUT &= ~driver->cs_pin;

        UCA0TXBUF = datagram->addr.value;
        while(UCA0STAT & UCBUSY);
        driver->response.value = UCA0RXBUF;

        UCA0TXBUF = 0;
        while(UCA0STAT & UCBUSY);
        *payload-- = UCA0RXBUF;

        UCA0TXBUF = 0;
        while(UCA0STAT & UCBUSY);
        *payload-- = UCA0RXBUF;

        UCA0TXBUF = 0;
        while(UCA0STAT & UCBUSY);
        *payload-- = UCA0RXBUF;

        UCA0TXBUF = 0;
        while(UCA0STAT & UCBUSY);
        *payload-- = UCA0RXBUF;

        CS_PORT_OUT |= driver->cs_pin;
    }
}

// Poll for driver status and report via interrupt request to driver if attention is required
// Poll rate is 500 mS
static void pollEvent (void)
{
    uint16_t idx = NUM_AXIS, mask = 0x01 << (NUM_AXIS - 1);
    bool fatal = false, warning = false;

    diag_data.ot.mask = 0;
    diag_data.otpw.mask = 0;
    diag_data.otpw_cnt.mask = 0;
    diag_data.error.mask = 0;

    do {
        idx--;
        if(axes.reg.monitor.mask & mask) {

            readRegisterBR(&driver[idx], (TMC2130_datagram_t *)&(driver[idx].status));

            if(driver[idx].status.reg.ot) {
                diag_data.ot.mask |= mask;
                fatal = true; // TODO: break here since this is a fatal error?
            }

            if(driver[idx].status.reg.otpw) {
                diag_data.otpw.mask |= mask;
                if(driver[idx].otpw_count == 0)
                    warning = true;
                if(++driver[idx].otpw_count == driver[idx].otpw_max) {
                    warning = true;
                    diag_data.otpw_cnt.mask |= mask;
                }
            } else
                driver[idx].otpw_count = 0;

            // Check olb, ola, s2gb, s2gb and driver_error flag for errors
            if((driver[idx].status.reg.value & 0x78000000UL) || driver[idx].response.driver_error) {
                diag_data.error.mask |= mask;
//                fatal = true; // TODO: break here since this is a fatal error?
            }
        }

        mask >>= 1;

    } while(idx);

    if(fatal) {

        DIAG1_IRQ_DIR |= DIAG1_IRQ_PIN;
        DIAG1_IRQ_OUT &= ~DIAG1_IRQ_PIN;

        __delay_cycles(32); // ~1uS IRQ pulse

        DIAG1_IRQ_OUT |= DIAG1_IRQ_PIN;
        DIAG1_IRQ_DIR &= ~DIAG1_IRQ_PIN;

    }

    if(warning) {

        WARN_IRQ_DIR |= DIAG1_IRQ_PIN;
        WARN_IRQ_OUT &= ~DIAG1_IRQ_PIN;

        __delay_cycles(32); // ~1uS IRQ pulse

        WARN_IRQ_OUT |= DIAG1_IRQ_PIN;
        WARN_IRQ_DIR &= ~DIAG1_IRQ_PIN;
    }
}

void main (void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    DCOCTL = CALDCO_16MHZ;      // Set DCO for 16 MHz using
    BCSCTL1 = CALBC1_16MHZ;     // calibration registers
//    BCSCTL2 &= ~(DIVS1|DIVS0);

    SPI_PORT_SEL |= MOSI_PIN|MISO_PIN|SCLK_PIN;
    SPI_PORT_SEL2 |= MOSI_PIN|MISO_PIN|SCLK_PIN;

    CS_PORT_DIR |= CS_PIN_MASK;
    CS_PORT_OUT |= CS_PIN_MASK;
    P2SEL &= ~BIT6;
    ENA_PORT_DIR |= ENA_PIN_MASK;
    ENA_PORT_OUT &= ~ENA_PIN_MASK;

    DIAG1_X_OUT |= DIAG1_X_PIN;
    DIAG1_X_REN |= DIAG1_X_PIN;
    DIAG1_X_IES |= DIAG1_X_PIN;
    __delay_cycles(25);
    DIAG1_X_IFG &= ~DIAG1_X_PIN;
    DIAG1_X_IE  |= DIAG1_X_PIN;

    DIAG1_Y_OUT |= DIAG1_Y_PIN;
    DIAG1_Y_REN |= DIAG1_Y_PIN;
    DIAG1_Y_IES |= DIAG1_Y_PIN;
    __delay_cycles(25);
    DIAG1_Y_IFG &= ~DIAG1_Y_PIN;
    DIAG1_Y_IE  |= DIAG1_Y_PIN;

#ifdef DIAG1_Z_SEL
    DIAG1_Z_SEL &= ~DIAG1_Z_PIN;
#endif
    DIAG1_Z_OUT |= DIAG1_Z_PIN;
    DIAG1_Z_REN |= DIAG1_Z_PIN;
    DIAG1_Z_IES |= DIAG1_Z_PIN;
    __delay_cycles(25);
    DIAG1_Z_IFG &= ~DIAG1_Z_PIN;
    DIAG1_Z_IE  |= DIAG1_Z_PIN;

    TA0CCR0 = 15999;                // 1ms timer
    TA0CCTL0 = CCIE;                // Enable CCR0 interrupt|                            // Start TA0 in up mode
    TA0CTL = TASSEL1|TACLR|MC0;     // bind to SMCLK and clear TA

    uint16_t idx;
    for(idx = 0; idx < NUM_AXIS; idx++) {
        driver[idx].status.addr.reg = TMC2130Reg_DRV_STATUS;
        driver[idx].otpw_max = 4;
    }

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

    uint8_t ifg = IFG2, ie = IE2;

    while(true) {

        LPM0;

        if (i2c.cmd & CMD_ReadRegister)
            readRegister(active);

        if (i2c.cmd & I2C_WriteRegister)
            writeRegister(active);

        if (i2c.cmd & I2C_WriteEnable)
            writeEnable(active);

        if (i2c.cmd & I2C_Diag1Event)
            diag1Event();

        if (i2c.cmd &  I2C_PollEvent)
            pollEvent();

        i2c.cmd = CMD_Idle;
    }
}

#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
    if(IFG2 & UCB0TXIFG)
        UCB0TXBUF = *i2c.tx_data++;    // Transmit register content

    if(IFG2 & UCB0RXIFG) {
        if(i2c.rx_count < 5) {
            *i2c.rx_data++ = UCB0RXBUF;
            if(++i2c.rx_count == 1) {
                uint16_t idx = (i2c.data[0] >> 5) & 0x03;
                active = &driver[idx];             // set active driver TODO: add range check?
                active->reg.addr.value = TMC2130_I2C_regmap[(i2c.data[0] & 0x1FU)]; // restore original register address;
                i2c.rx_data = (uint8_t *)&(active->reg.payload);
                if(!active->reg.addr.write) {
                    UCB0I2CIE &= ~UCSTTIE;
                    i2c.tx_data = (uint8_t *)&(active->reg);
                    i2c.cmd |= CMD_ReadRegister;
                    LPM0_EXIT;
                }
            }
        } else
            UCB0RXBUF;
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
        UCB0I2CIE |= UCSTTIE;
        if(i2c.rx_count == 5 && active->reg.addr.write) {
            i2c.cmd |= active->reg.addr.idx == TMC_I2CReg_ENABLE ? I2C_WriteEnable : I2C_WriteRegister;
            LPM0_EXIT;
        }
    }

    UCB0STAT &= ~intstate;      // Clear interrupt flags
}

#pragma vector=PORT1_VECTOR
__interrupt void P1_ISR(void)
{
    diag1_event.x = (DIAG1_X_IFG & DIAG1_X_PIN) != 0;
    diag1_event.y = (DIAG1_Y_IFG & DIAG1_Y_PIN) != 0;

    P1IFG = 0;
    i2c.cmd |= I2C_Diag1Event;

    LPM0_EXIT;
}

#pragma vector=PORT2_VECTOR
__interrupt void P2_ISR(void)
{
    diag1_event.z = (DIAG1_Z_IFG & DIAG1_Z_PIN) == 0;

    P2IFG = 0;
    i2c.cmd |= I2C_Diag1Event;

    LPM0_EXIT;
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void CCR0_ISR(void)
{
    static volatile uint16_t msec = 0;

    msec++;

    if(msec == 499) {
        i2c.cmd |= I2C_PollEvent;
        msec = 0;
        LPM0_EXIT;
    }
}
