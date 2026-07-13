/* -----------------------------------------------------------------------
 * i2c_test.c
 * I2C sensor test — LM75B, ADXL345, SHT45, SHT31
 * ----------------------------------------------------------------------- */

#include "i2c_test.h"
#include "main.h"
#include "modbus.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c4;

/* ----------------------------------------------------------------------- */
/* Device addresses (7-bit shifted left for HAL)                           */
/* ----------------------------------------------------------------------- */
#define LM75B_ADDR      (0x49 << 1)  /* on-board temp; A0=VCC, A1=A2=GND */
#define ADXL345_ADDR    (0x53 << 1)  /* EL axis accel; SDO=GND; use 0x1D<<1 if SDO=VCC */
#define SHT45_ADDR      (0x44 << 1)  /* EL axis temp/RH */
#define SHT31_ADDR      (0x45 << 1)  /* AZ axis temp/RH; ADDR pin=VCC */

#define I2C_TIMEOUT     100

/* Both SHT parts are triggered together; 20ms covers the slower SHT31
 * high-repeatability conversion (15ms max; SHT45 high precision is 8.3ms). */
#define SHT_MEAS_TIME_MS  20

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
static HAL_StatusTypeDef LM75B_ReadTemp(float *temp)
{
    uint8_t buf[2] = {0};
    uint8_t reg    = 0x00;

    /* Write register pointer */
    if (HAL_I2C_Master_Transmit(&hi2c4, LM75B_ADDR, &reg, 1, I2C_TIMEOUT) != HAL_OK)
        return HAL_ERROR;

    /* Read 2 bytes */
    if (HAL_I2C_Master_Receive(&hi2c4, LM75B_ADDR, buf, 2, I2C_TIMEOUT) != HAL_OK)
        return HAL_ERROR;

    /* Convert: top 9 bits, 0.5°C per LSB */
    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]) >> 7;
    *temp = raw * 0.5f;
    return HAL_OK;
}

static void LM75B_Read(void)
{
    float temp;
    if (LM75B_ReadTemp(&temp) != HAL_OK)
    {
        printf("LM75B: read error\r\n");
        return;
    }
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

static HAL_StatusTypeDef ADXL345_ReadXYZ(float *fx, float *fy, float *fz)
{
    /* Retry init on every read so a sensor plugged in after boot works */
    if (!adxl_ok)
        ADXL345_Init();
    if (!adxl_ok)
        return HAL_ERROR;

    uint8_t buf[6];
    if (ADXL345_ReadRegs(0x32, buf, 6) != HAL_OK)
        return HAL_ERROR;

    int16_t x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t z = (int16_t)((buf[5] << 8) | buf[4]);

    /* ±2g range: 3.9 mg/LSB */
    *fx = x * 0.0039f;
    *fy = y * 0.0039f;
    *fz = z * 0.0039f;
    return HAL_OK;
}

static void ADXL345_Read(void)
{
    if (!adxl_ok) return;

    float fx, fy, fz;
    if (ADXL345_ReadXYZ(&fx, &fy, &fz) != HAL_OK)
    {
        printf("ADXL345: read error\r\n");
        return;
    }
    printf("ADXL345: X=%.3fg  Y=%.3fg  Z=%.3fg\r\n", fx, fy, fz);
}

/* ----------------------------------------------------------------------- */
/* SHT45 / SHT31 — temperature and humidity sensors                        */
/* SHT45: single-byte command 0xFD (measure T+RH, high precision)          */
/* SHT31: two-byte command 0x2400 (single shot, high repeatability,        */
/*        clock stretching disabled)                                       */
/* Response (both): temp MSB, temp LSB, CRC, hum MSB, hum LSB, CRC        */
/* Conversion differs: SHT45 RH = -6 + 125*S/65535, SHT31 RH = 100*S/65535 */
/* ----------------------------------------------------------------------- */
static HAL_StatusTypeDef SHT45_StartMeas(void)
{
    uint8_t cmd = 0xFD;
    return HAL_I2C_Master_Transmit(&hi2c4, SHT45_ADDR, &cmd, 1, I2C_TIMEOUT);
}

static HAL_StatusTypeDef SHT31_StartMeas(void)
{
    uint8_t cmd[2] = {0x24, 0x00};
    return HAL_I2C_Master_Transmit(&hi2c4, SHT31_ADDR, cmd, 2, I2C_TIMEOUT);
}

static HAL_StatusTypeDef SHT_ReadResult(uint16_t devAddr, uint8_t is_sht45,
                                         float *temp, float *hum)
{
    uint8_t buf[6] = {0};

    if (HAL_I2C_Master_Receive(&hi2c4, devAddr, buf, 6, I2C_TIMEOUT) != HAL_OK)
        return HAL_ERROR;

    uint16_t t_raw = ((uint16_t)buf[0] << 8) | buf[1];
    *temp = -45.0f + 175.0f * ((float)t_raw / 65535.0f);

    uint16_t h_raw = ((uint16_t)buf[3] << 8) | buf[4];
    float rh = is_sht45 ? -6.0f + 125.0f * ((float)h_raw / 65535.0f)
                        : 100.0f * ((float)h_raw / 65535.0f);
    if (rh > 100.0f) rh = 100.0f;
    if (rh < 0.0f)   rh = 0.0f;
    *hum = rh;
    return HAL_OK;
}

static void SHT45_Read(void)
{
    if (SHT45_StartMeas() != HAL_OK)
    {
        printf("SHT45: TX error\r\n");
        return;
    }

    HAL_Delay(10);  /* measurement takes ~8.3ms */

    float temp, hum;
    if (SHT_ReadResult(SHT45_ADDR, 1, &temp, &hum) != HAL_OK)
    {
        printf("SHT45: RX error\r\n");
        return;
    }
    printf("SHT45:   T=%.2f degC  RH=%.2f%%\r\n", temp, hum);
}

/* ----------------------------------------------------------------------- */
/* Modbus-triggered sensor query                                            */
/*                                                                          */
/* Writing 1 to REG_I2C_CMD sets queryPending; the sweep itself runs from   */
/* I2C_Query_Run() in the main loop, split across ticks so the SHT          */
/* measurement time is a tick-counted wait instead of a HAL_Delay that      */
/* would stall the motion control loop.                                     */
/* ----------------------------------------------------------------------- */
static volatile uint8_t queryPending = 0;

typedef enum {
    QS_IDLE,
    QS_SHT_MEASURING,
} QueryState_t;

static QueryState_t qState = QS_IDLE;
static uint32_t     qMeasStart = 0;
static uint16_t     qPresent = 0;
static uint8_t      qSht45Started = 0;
static uint8_t      qSht31Started = 0;

void I2C_Query_Trigger(void)
{
    queryPending = 1;
}

void I2C_Query_Run(void)
{
    if (qState == QS_IDLE)
    {
        if (!queryPending)
            return;
        queryPending = 0;

        Modbus_SetReg(REG_I2C_STATUS, I2C_QUERY_BUSY);
        qPresent = 0;

        float t;
        if (LM75B_ReadTemp(&t) == HAL_OK)
        {
            qPresent |= I2C_PRESENT_LM75B;
            Modbus_SetRegFloat(REG_I2C_LM75_TEMP_HI, t);
        }

        float x, y, z;
        if (ADXL345_ReadXYZ(&x, &y, &z) == HAL_OK)
        {
            qPresent |= I2C_PRESENT_ADXL345;
            Modbus_SetRegFloat(REG_I2C_ADXL_X_HI, x);
            Modbus_SetRegFloat(REG_I2C_ADXL_Y_HI, y);
            Modbus_SetRegFloat(REG_I2C_ADXL_Z_HI, z);
        }

        /* Kick off both SHT conversions in parallel, collect next tick */
        qSht45Started = (SHT45_StartMeas() == HAL_OK);
        qSht31Started = (SHT31_StartMeas() == HAL_OK);

        qMeasStart = HAL_GetTick();
        qState = QS_SHT_MEASURING;
        return;
    }

    /* QS_SHT_MEASURING */
    if (HAL_GetTick() - qMeasStart < SHT_MEAS_TIME_MS)
        return;

    float temp, hum;
    if (qSht45Started && SHT_ReadResult(SHT45_ADDR, 1, &temp, &hum) == HAL_OK)
    {
        qPresent |= I2C_PRESENT_SHT45;
        Modbus_SetRegFloat(REG_I2C_SHT45_TEMP_HI, temp);
        Modbus_SetRegFloat(REG_I2C_SHT45_RH_HI, hum);
    }
    if (qSht31Started && SHT_ReadResult(SHT31_ADDR, 0, &temp, &hum) == HAL_OK)
    {
        qPresent |= I2C_PRESENT_SHT31;
        Modbus_SetRegFloat(REG_I2C_SHT31_TEMP_HI, temp);
        Modbus_SetRegFloat(REG_I2C_SHT31_RH_HI, hum);
    }

    Modbus_SetReg(REG_I2C_PRESENT, qPresent);
    Modbus_SetReg(REG_I2C_STATUS, I2C_QUERY_DONE);
    qState = QS_IDLE;
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
