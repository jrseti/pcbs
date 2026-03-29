#ifndef APP_H
#define APP_H

/* app.h — MCU Test Board application header
 * STM32G431KBT6TR  |  ModBus-RTU over USART1
 * Rev 1.1
 *
 * Pin assignments (from IOC):
 *   PA0  — ENC_A      (TIM2 CH1, encoder)
 *   PA1  — ENC_B      (TIM2 CH2, encoder)
 *   PA2  — ENC_Z      (EXTI2, rising edge, index pulse)
 *   PA3  — LIMIT_SW1  (GPIO Input, pull-up, active LOW via opto)
 *   PA4  — LIMIT_SW2  (GPIO Input, pull-up, active LOW via opto)
 *   PA6  — ENABLE     (GPIO Output, motor enable → opto)
 *   PA7  — DIR        (GPIO Output, motor direction → opto)
 *   PA8  — I2C3_SCL
 *   PA9  — ADDR0      (GPIO Input, pull-up, SW1 bit0)
 *   PA10 — ADDR1      (GPIO Input, pull-up, SW1 bit1)
 *   PA11 — ADDR2      (GPIO Input, pull-up, SW1 bit2)
 *   PA12 — STATUS_LED (GPIO Output)
 *   PA13 — SWDIO
 *   PA14 — SWCLK
 *   PB0  — PUL        (TIM3 CH3 PWM, motor pulse)
 *   PB3  — SWO
 *   PB5  — I2C3_SDA
 *   PB6  — USART1_TX
 *   PB7  — USART1_RX
 */

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Version
 * ========================================================================= */
#define FW_VERSION_MAJOR  1
#define FW_VERSION_MINOR  1
#define FW_VERSION        ((FW_VERSION_MAJOR << 8) | FW_VERSION_MINOR)

/* =========================================================================
 * ModBus register addresses (must match PROTOCOL.md and Pi host)
 * ========================================================================= */

/* Encoder configuration (R/W) */
#define REG_ENC_PPR           0x0000u
#define REG_ENC_LIM1_POS_HI  0x0001u
#define REG_ENC_LIM1_POS_LO  0x0002u
#define REG_ENC_LIM2_POS_HI  0x0003u
#define REG_ENC_LIM2_POS_LO  0x0004u
#define REG_ENC_PUSH_EN       0x0005u
#define REG_ENC_PUSH_THRESH   0x0006u

/* Encoder status (Read-only) */
#define REG_ENC_POS_HI        0x0010u
#define REG_ENC_POS_LO        0x0011u
#define REG_ENC_VELOCITY      0x0012u
#define REG_ENC_STATUS        0x0013u

/* Motor control (R/W, MOT_STATUS read-only) */
#define REG_MOT_ENABLE        0x0020u
#define REG_MOT_DIR           0x0021u
#define REG_MOT_PWM_FREQ      0x0022u
#define REG_MOT_PWM_DUTY      0x0023u
#define REG_MOT_STATUS        0x0024u

/* SHT45 (Read-only) */
#define REG_SHT45_TEMP        0x0030u
#define REG_SHT45_RH          0x0031u
#define REG_SHT45_STATUS      0x0032u

/* ADXL345 (Read-only) */
#define REG_ADXL_X            0x0038u
#define REG_ADXL_Y            0x0039u
#define REG_ADXL_Z            0x003Au
#define REG_ADXL_STATUS       0x003Bu

/* System registers */
#define REG_SYS_ADDR          0x00F0u
#define REG_SYS_UPTIME_HI     0x00F1u
#define REG_SYS_UPTIME_LO     0x00F2u
#define REG_SYS_FW_VER        0x00F3u
#define REG_SYS_RESET         0x00F4u

/* Total register count (highest address + 1) */
#define REG_MAP_SIZE          0x00F5u

/* =========================================================================
 * ModBus function codes
 * ========================================================================= */
#define FC_READ_HOLDING       0x03u
#define FC_WRITE_SINGLE       0x06u
#define FC_WRITE_MULTIPLE     0x10u
#define FC_PUSH_NOTIFY        0x41u  /* Custom: MCU → Pi unsolicited */

/* Exception code = FC | 0x80 */
#define FC_EXCEPTION_MASK     0x80u

#define MB_EX_ILLEGAL_FUNC    0x01u
#define MB_EX_ILLEGAL_ADDR    0x02u
#define MB_EX_ILLEGAL_DATA    0x03u
#define MB_EX_DEVICE_FAILURE  0x04u

/* =========================================================================
 * ENC_STATUS bitfield
 * ========================================================================= */
#define ENC_STATUS_AT_LIMIT1  (1u << 0)
#define ENC_STATUS_AT_LIMIT2  (1u << 1)
#define ENC_STATUS_INDEX_SEEN (1u << 2)
#define ENC_STATUS_OVERFLOW   (1u << 3)

/* =========================================================================
 * MOT_STATUS bitfield
 * ========================================================================= */
#define MOT_STATUS_ENABLED    (1u << 0)
#define MOT_STATUS_RUNNING    (1u << 1)
#define MOT_STATUS_DIR_CW     (1u << 2)

/* =========================================================================
 * I2C sensor addresses
 * ========================================================================= */
#define SHT45_I2C_ADDR        (0x44u << 1)   /* 7-bit addr shifted for HAL */
#define ADXL345_I2C_ADDR      (0x53u << 1)   /* SDO/ALT pulled low         */

/* I2C sensor read status codes (stored in REG_xxx_STATUS) */
#define SENSOR_STATUS_OK      0u
#define SENSOR_STATUS_CRC_ERR 1u
#define SENSOR_STATUS_TIMEOUT 2u

/* =========================================================================
 * Timing / limits
 * ========================================================================= */
#define MODBUS_BAUD           115200u
#define MODBUS_RX_BUF_SIZE    128u
#define MODBUS_TX_BUF_SIZE    128u
#define MODBUS_FRAME_TIMEOUT_MS  5u   /* 3.5-char gap at 115200 ~ 0.3ms; use 5ms */

#define SENSOR_POLL_INTERVAL_MS  500u  /* Read I2C sensors every 500 ms */
#define VELOCITY_INTERVAL_MS     100u  /* Compute velocity every 100 ms  */
#define PWM_FREQ_MIN_HZ          1u
#define PWM_FREQ_MAX_HZ          50000u
#define PWM_TIMER_CLK_HZ         10000000u  /* TIM3 after prescaler: 170MHz/(16+1)≈10MHz */

#define SOFT_RESET_MAGIC         0xDEADu

/* =========================================================================
 * Register map (global, accessible from all modules)
 * ========================================================================= */
extern volatile uint16_t g_regs[REG_MAP_SIZE];

/* =========================================================================
 * External peripheral handles (defined by CubeMX in main.c)
 * ========================================================================= */
extern TIM_HandleTypeDef htim2;   /* Encoder interface */
extern TIM_HandleTypeDef htim3;   /* PWM output PB0    */
extern TIM_HandleTypeDef htim6;   /* 1 ms tick         */
extern UART_HandleTypeDef huart1; /* USART1 RS-232     */
extern I2C_HandleTypeDef hi2c3;   /* I2C3              */

/* =========================================================================
 * Public API
 * ========================================================================= */

/* HAL UART RX target — pass to HAL_UART_Receive_IT, read in RxCpltCallback */
extern uint8_t g_mb_rx_byte;

/* Call once from main() after all HAL inits */
void App_Init(void);

/* Call from main() super-loop */
void App_Run(void);

/* UART RX complete callback — call from HAL_UART_RxCpltCallback:
 *   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 *       if (huart->Instance == USART1) ModBus_RxByteCallback(g_mb_rx_byte);
 *   }
 */
void ModBus_RxByteCallback(uint8_t byte);

/* TIM6 period elapsed callback — call from HAL_TIM_PeriodElapsedCallback:
 *   void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 *       if (htim->Instance == TIM6) App_1msTickCallback();
 *   }
 */
void App_1msTickCallback(void);

/* EXTI2 callback for encoder index (Z) — call from HAL_GPIO_EXTI_Callback:
 *   void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
 *       if (GPIO_Pin == GPIO_PIN_2) Encoder_IndexCallback();
 *   }
 */
void Encoder_IndexCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
