#ifndef I2C_TEST_H
#define I2C_TEST_H

/* -----------------------------------------------------------------------
 * i2c_test.h
 * I2C device test
 *
 * I2C4: SCL=PC6, SDA=PC7, 100 kHz
 * LM75B   — on-board temp sensor,     addr 0x49 (A0=VCC)
 * SHT45   — EL axis temp/humidity,    addr 0x44
 * ADXL345 — EL axis accelerometer,    addr 0x53 (SDO=GND) or 0x1D (SDO=VCC)
 * SHT31   — AZ axis temp/humidity,    addr 0x45
 * ----------------------------------------------------------------------- */

#include <stdint.h>

void I2C_Test_Init(void);  /* scan bus, init devices, print results */
void I2C_Test_Run(void);   /* read all sensors every 1s             */

void I2C_Query_Trigger(void);  /* request a sensor sweep (REG_I2C_CMD)     */
void I2C_Query_Run(void);      /* main-loop tick: runs the sweep, fills    */
                               /* REG_I2C_* result registers               */

#endif /* I2C_TEST_H */
