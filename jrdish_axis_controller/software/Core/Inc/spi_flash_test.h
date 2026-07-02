#ifndef SPI_FLASH_TEST_H
#define SPI_FLASH_TEST_H

#include <stdint.h>

/* Result of JEDEC ID read */
typedef struct {
    uint8_t manufacturer;
    uint8_t memType;
    uint8_t capacity;
    uint8_t valid;
} Flash_ID_t;

Flash_ID_t SpiFlash_ReadID(void);
uint8_t    SpiFlash_Test(void);

/* Non-blocking command interface called by Modbus layer */
void Flash_ExecCmd(uint16_t cmd);   /* called on FLASH_CMD write       */
void Flash_Run(void);               /* called every App_Run() iteration */

#endif /* SPI_FLASH_TEST_H */

