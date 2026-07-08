/* -----------------------------------------------------------------------
 * app.c
 * Application layer - top-level init and run loop
 * ----------------------------------------------------------------------- */

#include "app.h"
#include "main.h"
#include "modbus.h"
#include "spi_flash_test.h"
#include "pwm_test.h"
#include "encoder.h"
#include "i2c_test.h"
#include "gps.h"
#include <stdio.h>

static uint32_t lastLedToggle = 0;
static uint32_t ledInterval   = 500;

void App_Init(void)
{
    /* Enable ITM SWO output */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    ITM->LAR  = 0xC5ACCE55;
    ITM->TER  = 1;
    ITM->TCR  = ITM_TCR_ITMENA_Msk | ITM_TCR_SYNCENA_Msk;

    printf("App_Init start\r\n");

    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);

    printf("Modbus_Init...\r\n");
    Modbus_Init();
    /* Reset flash state machine to idle in case previous session left it mid-operation */
    Modbus_SetReg(REG_FLASH_STATUS, FLASH_STATUS_IDLE);
    printf("Modbus_Init done\r\n");

    printf("PWM_Test_Init...\r\n");
    PWM_Test_Init();
    printf("PWM_Test_Init done\r\n");

    printf("Encoder_Init...\r\n");
    Encoder_Init();
    printf("Encoder_Init done\r\n");

    printf("I2C_Test_Init...\r\n");
    I2C_Test_Init();
    printf("I2C_Test_Init done\r\n");

    printf("GPS_Init...\r\n");
    GPS_Init();
    printf("GPS_Init done\r\n");

    printf("SpiFlash_ReadID...\r\n");
    Flash_ID_t id = SpiFlash_ReadID();
    printf("SpiFlash_ReadID done - valid=%d mfr=0x%02X type=0x%02X cap=0x%02X\r\n",
           id.valid, id.manufacturer, id.memType, id.capacity);
    if (!id.valid)
        ledInterval = 100;

    printf("App_Init complete\r\n");
    lastLedToggle = HAL_GetTick();
}

void App_Run(void)
{
    Modbus_Run();
    Flash_Run();
    /* Encoder_Run() before PWM_Test_Run(): the homing/move control loop in
     * PWM_Test_Run() reads REG_LIMIT_SW and REG_*_POS_HI, both written by
     * Encoder_Run(), and must see fresh data from the same tick. */
    Encoder_Run();
    PWM_Test_Run();
    //I2C_Test_Run();
    GPS_Run();

    if (HAL_GetTick() - lastLedToggle >= ledInterval)
    {
        HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
        lastLedToggle = HAL_GetTick();
    }
}
