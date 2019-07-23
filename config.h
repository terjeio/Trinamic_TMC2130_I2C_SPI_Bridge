//
// config.h - Trinamic TMC2130 stepper driver I2C <> SPI bridge for up to 8 drivers
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

#include <msp430.h>

#ifndef _TMCBRIDGE_CONFIG_H_
#define _TMCBRIDGE_CONFIG_H_

#include <stdint.h>

#include "portmacros.h"

#define NUM_AXIS 3
#define I2C_ADR_I2CBRIDGE    0x47

#define SPI A0
#define SPI_CTL0 usciCTL(SPI, 0)
#define SPI_CTL1 usciCTL(SPI, 1)
#define SPI_BR0 usciBR(SPI, 0)
#define SPI_BR1 usciBR(SPI, 1)
#define SPI_IE usciIE(SPI)
#define SPI_IFG usciIFG(SPI)
#define SPI_IV usciIV(SPI)
#define SPI_RXBUF usciRXBUF(SPI)
#define SPI_TXBUF usciTXBUF(SPI)
#define SPI_STAT usciSTAT(SPI)
#define SPI_INTV usciInt(SPI)

#define SPI_PORT 1
#define MISO_PIN BIT1
#define MOSI_PIN BIT2
#define SCLK_PIN BIT4
#define SPI_PORT_SEL portSel(SPI_PORT,)
#define SPI_PORT_SEL2 portSel(SPI_PORT, 2)

#define CS_PORT 2
#define CS_PIN_X BIT1
#define CS_PIN_Y BIT2
#define CS_PIN_Z BIT3
#define CS_PIN_MASK (CS_PIN_X|CS_PIN_Y|CS_PIN_Z)

#define CS_PORT_OUT   portOut(CS_PORT)
#define CS_PORT_DIR   portDir(CS_PORT)

#define ENA_PORT 2
#define ENA_PIN_X BIT6
#define ENA_PIN_Y BIT5
#define ENA_PIN_Z BIT4
#define ENA_PIN_MASK (ENA_PIN_X|ENA_PIN_Y|ENA_PIN_Z)

#define ENA_PORT_OUT   portOut(ENA_PORT)
#define ENA_PORT_DIR   portDir(ENA_PORT)

#define WARN_IRQ_PORT   2
#define WARN_IRQ_PIN    BIT0
#define WARN_IRQ_OUT    portOut(WARN_IRQ_PORT)
#define WARN_IRQ_DIR    portDir(WARN_IRQ_PORT)

#define DIAG1_IRQ_PORT  1
#define DIAG1_IRQ_PIN   BIT0
#define DIAG1_IRQ_OUT   portOut(DIAG1_IRQ_PORT)
#define DIAG1_IRQ_DIR   portDir(DIAG1_IRQ_PORT)

#define DIAG1_X_PORT    1
#define DIAG1_X_PIN     BIT5
#define DIAG1_X_IN      portIn(DIAG1_X_PORT)
#define DIAG1_X_OUT     portOut(DIAG1_X_PORT)
#define DIAG1_X_IE      portIe(DIAG1_X_PORT)
#define DIAG1_X_IFG     portIfg(DIAG1_X_PORT)
#define DIAG1_X_IES     portEs(DIAG1_X_PORT)
#define DIAG1_X_REN     portRen(DIAG1_X_PORT)

#define DIAG1_Y_PORT    1
#define DIAG1_Y_PIN     BIT3
#define DIAG1_Y_IN      portIn(DIAG1_Y_PORT)
#define DIAG1_Y_OUT     portOut(DIAG1_Y_PORT)
#define DIAG1_Y_IE      portIe(DIAG1_Y_PORT)
#define DIAG1_Y_IFG     portIfg(DIAG1_Y_PORT)
#define DIAG1_Y_IES     portEs(DIAG1_Y_PORT)
#define DIAG1_Y_REN     portRen(DIAG1_Y_PORT)

#define DIAG1_Z_PORT    2
#define DIAG1_Z_PIN     BIT7
#define DIAG1_Z_IN      portIn(DIAG1_Z_PORT)
#define DIAG1_Z_OUT     portOut(DIAG1_Z_PORT)
#define DIAG1_Z_IE      portIe(DIAG1_Z_PORT)
#define DIAG1_Z_IFG     portIfg(DIAG1_Z_PORT)
#define DIAG1_Z_IES     portEs(DIAG1_Z_PORT)
#define DIAG1_Z_REN     portRen(DIAG1_Z_PORT)
#define DIAG1_Z_SEL     portSel(DIAG1_Z_PORT,)

#define I2C B0
#define I2C_CTL0 usciCTL(I2C, 0)
#define I2C_CTL1 usciCTL(I2C, 1)
#define I2C_BR0 usciBR(I2C, 0)
#define I2C_BR1 usciBR(I2C, 1)
#define I2C_IE usciIE(I2C)
#define I2C_IFG usciIFG(I2C)
#define I2C_IV usciIV(I2C)
#define I2C_RXBUF usciRXBUF(I2C)
#define I2C_TXBUF usciTXBUF(I2C)
#define I2C_STAT usciSTAT(I2C)
#define I2C_INTV usciInt(I2C)
#define I2C_SADDR usciSADDR(I2C)

#define I2C_PORT 1
#define SDA_PIN BIT7
#define SDC_PIN BIT6
#define I2C_PORT_SEL portSel(I2C_PORT,)
#define I2C_PORT_SEL2 portSel(I2C_PORT, 2)

#endif /* _OLEDDRIVER_H_ */
