#ifndef I2C_TEST_H
#define I2C_TEST_H

/* -----------------------------------------------------------------------
 * i2c_test.h
 * I2C device test
 *
 * I2C4: SCL=PC6, SDA=PC7, 100 kHz
 * LM75B   — temp sensor,    addr 0x48
 * ADXL345 — accelerometer,  addr 0x53 (SDO=GND) or 0x1D (SDO=VCC)
 * SHT45   — temp/humidity,  addr 0x44
 * ----------------------------------------------------------------------- */

#include <stdint.h>

void I2C_Test_Init(void);  /* scan bus, init devices, print results */
void I2C_Test_Run(void);   /* read all sensors every 1s             */

#endif /* I2C_TEST_H */
