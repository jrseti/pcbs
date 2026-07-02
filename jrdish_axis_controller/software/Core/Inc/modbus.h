#ifndef MODBUS_H
#define MODBUS_H

/* -----------------------------------------------------------------------
 * modbus.h
 * Modbus RTU slave driver — public API
 * ----------------------------------------------------------------------- */

#include <stdint.h>
#include "modbus_regs.h"

void     Modbus_Init(void);
void     Modbus_Run(void);
void     Modbus_UART_RxByte(void);  /* called from HAL_UART_RxCpltCallback */

/* Register access (used by app layer to update status registers) */
uint16_t Modbus_GetReg(uint16_t addr);
void     Modbus_SetReg(uint16_t addr, uint16_t value);

/* Convenience helpers for 32-bit values */
void     Modbus_SetReg32(uint16_t addrHi, uint32_t value);
uint32_t Modbus_GetReg32(uint16_t addrHi);

/* Convenience helpers for float32 values */
void     Modbus_SetRegFloat(uint16_t addrHi, float value);
float    Modbus_GetRegFloat(uint16_t addrHi);

#endif /* MODBUS_H */
