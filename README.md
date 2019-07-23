## Trinamic TMC2130 SPI stepper driver I2C bridge

Based on a Texas Instruments MSP430G2553 processor.

Message format supports up to 8 drivers, current code for up to 3.

Extension datagrams may be used to enable drivers and monitoring and return monitor/stall status.

DIAG1 signals from the drivers are combined into single signal to master \(fault)\ as is driver errors and overtemperature.
A second signal \(warning)\ to master is available for overtemperature prewarning and overtemperature prewarning max count reached.
The fault and warning signals are open drain active low signals and may be used to trigger interrupts in the master.

---
2019-07-23
