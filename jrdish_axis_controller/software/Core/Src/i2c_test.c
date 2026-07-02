/* -----------------------------------------------------------------------
 * i2c_test.c
 * I2C sensor test — LM75B, ADXL345, SHT45
 * ----------------------------------------------------------------------- */

#include "i2c_test.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c4;

/* ----------------------------------------------------------------------- */
/* Device addresses (7-bit shifted left for HAL)                           */
/* ----------------------------------------------------------------------- */
#define LM75B_ADDR      (0x49 << 1)  /* A0=VCC, A1=A2=GND */
#define ADXL345_ADDR    (0x53 << 1)  /* SDO=GND; use 0x1D<<1 if SDO=VCC */
#define SHT45_ADDR      (0x44 << 1)

#define I2C_TIMEOUT     100

/* ----------------------------------------------------------------------- */
/* I2C bus scan                                                             */
/* ----------------------------------------------------------------------- */
static void I2C_Scan(void)
{
    printf("I2C scan:\r\n");
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c4, addr << 1, 1, 10) == HAL_OK)
        {
            printf("  Found device at 0x%02X\r\n", addr);
            found++;
        }
    }
    if (found == 0)
        printf("  No devices found\r\n");
    else
        printf("  %u device(s) found\r\n", found);
}

/* ----------------------------------------------------------------------- */
/* LM75B — temperature sensor                                              */
/* Register 0x00 = 2 bytes, MSB first, top 9 bits, 0.5°C resolution       */
/* ----------------------------------------------------------------------- */
static void LM75B_Read(void)
{
    uint8_t buf[2] = {0};
    uint8_t reg    = 0x00;

    /* Write register pointer */
    if (HAL_I2C_Master_Transmit(&hi2c4, LM75B_ADDR, &reg, 1, I2C_TIMEOUT) != HAL_OK)
    {
        printf("LM75B: TX error\r\n");
        return;
    }

    /* Read 2 bytes */
    if (HAL_I2C_Master_Receive(&hi2c4, LM75B_ADDR, buf, 2, I2C_TIMEOUT) != HAL_OK)
    {
        printf("LM75B: RX error\r\n");
        return;
    }

    /* Convert: top 9 bits, 0.5°C per LSB */
    int16_t raw  = (int16_t)((buf[0] << 8) | buf[1]) >> 7;
    float   temp = raw * 0.5f;
    printf("LM75B:   %.1f degC\r\n", temp);
}

/* ----------------------------------------------------------------------- */
/* ADXL345 — accelerometer                                                 */
/* ----------------------------------------------------------------------- */
static uint8_t adxl_ok = 0;

static HAL_StatusTypeDef ADXL345_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return HAL_I2C_Master_Transmit(&hi2c4, ADXL345_ADDR, buf, 2, I2C_TIMEOUT);
}

static HAL_StatusTypeDef ADXL345_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (HAL_I2C_Master_Transmit(&hi2c4, ADXL345_ADDR, &reg, 1, I2C_TIMEOUT) != HAL_OK)
        return HAL_ERROR;
    return HAL_I2C_Master_Receive(&hi2c4, ADXL345_ADDR, buf, len, I2C_TIMEOUT);
}

static void ADXL345_Init(void)
{
    /* Check DEVID register — should return 0xE5 */
    uint8_t devid = 0;
    if (ADXL345_ReadRegs(0x00, &devid, 1) != HAL_OK || devid != 0xE5)
    {
        printf("ADXL345: not found (DEVID=0x%02X)\r\n", devid);
        adxl_ok = 0;
        return;
    }
    printf("ADXL345: DEVID=0x%02X OK\r\n", devid);

    /* Set measurement mode, ±2g, 100Hz output */
    ADXL345_WriteReg(0x2D, 0x08);  /* POWER_CTL: measure=1 */
    ADXL345_WriteReg(0x31, 0x00);  /* DATA_FORMAT: ±2g, right-justify */
    ADXL345_WriteReg(0x2C, 0x0A);  /* BW_RATE: 100 Hz */
    adxl_ok = 1;
}

static void ADXL345_Read(void)
{
    if (!adxl_ok) return;

    uint8_t buf[6];
    if (ADXL345_ReadRegs(0x32, buf, 6) != HAL_OK)
    {
        printf("ADXL345: read error\r\n");
        return;
    }

    int16_t x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t z = (int16_t)((buf[5] << 8) | buf[4]);

    /* ±2g range: 3.9 mg/LSB */
    float fx = x * 0.0039f;
    float fy = y * 0.0039f;
    float fz = z * 0.0039f;

    printf("ADXL345: X=%.3fg  Y=%.3fg  Z=%.3fg\r\n", fx, fy, fz);
}

/* ----------------------------------------------------------------------- */
/* SHT45 — temperature and humidity sensor                                 */
/* Single-shot high-precision measurement command: 0xFD                    */
/* Response: 6 bytes — temp MSB, temp LSB, CRC, hum MSB, hum LSB, CRC    */
/* ----------------------------------------------------------------------- */
static void SHT45_Read(void)
{
    uint8_t cmd = 0xFD;  /* measure T+RH, high precision */
    uint8_t buf[6] = {0};

    if (HAL_I2C_Master_Transmit(&hi2c4, SHT45_ADDR, &cmd, 1, I2C_TIMEOUT) != HAL_OK)
    {
        printf("SHT45: TX error\r\n");
        return;
    }

    HAL_Delay(10);  /* measurement takes ~8.3ms */

    if (HAL_I2C_Master_Receive(&hi2c4, SHT45_ADDR, buf, 6, I2C_TIMEOUT) != HAL_OK)
    {
        printf("SHT45: RX error\r\n");
        return;
    }

    /* Convert temperature */
    uint16_t t_raw = ((uint16_t)buf[0] << 8) | buf[1];
    float temp     = -45.0f + 175.0f * ((float)t_raw / 65535.0f);

    /* Convert humidity */
    uint16_t h_raw = ((uint16_t)buf[3] << 8) | buf[4];
    float hum      = -6.0f + 125.0f * ((float)h_raw / 65535.0f);
    if (hum > 100.0f) hum = 100.0f;
    if (hum < 0.0f)   hum = 0.0f;

    printf("SHT45:   T=%.2f degC  RH=%.2f%%\r\n", temp, hum);
}

/* ----------------------------------------------------------------------- */
/* Public API                                                               */
/* ----------------------------------------------------------------------- */
static uint32_t lastRead = 0;

void I2C_Test_Init(void)
{
    static uint8_t initialized = 0;
    if (initialized) { printf("I2C_Test_Init: already initialized\r\n"); return; }
    initialized = 1;

    printf("I2C_Test_Init...\r\n");
    I2C_Scan();
    ADXL345_Init();
    lastRead = HAL_GetTick();
    printf("I2C_Test_Init done\r\n");
}

void I2C_Test_Run(void)
{
    if (HAL_GetTick() - lastRead < 1000)
        return;
    lastRead = HAL_GetTick();

    LM75B_Read();
    ADXL345_Read();
    SHT45_Read();
}
