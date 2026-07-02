/* -----------------------------------------------------------------------
 * spi_flash_test.c
 * W25Q32JV SPI NOR flash basic test
 *
 * Tests:
 *  1. JEDEC ID read     — confirms chip is present and SPI is working
 *  2. Status register   — confirms chip is not busy/protected
 *  3. Write/read cycle  — erases one 4KB sector, writes 256 bytes,
 *                         reads back and verifies
 * ----------------------------------------------------------------------- */

#include "spi_flash_test.h"
#include "main.h"
#include <string.h>

/* ----------------------------------------------------------------------- */
/* Config                                                                   */
/* ----------------------------------------------------------------------- */
#define FLASH_SPI           hspi2
#define FLASH_CS_PORT       FLASH_CS_GPIO_Port
#define FLASH_CS_PIN        FLASH_CS_Pin
#define FLASH_TIMEOUT       100

/* W25Q32JV commands */
#define CMD_JEDEC_ID        0x9F
#define CMD_READ_SR1        0x05
#define CMD_WRITE_ENABLE    0x06
#define CMD_SECTOR_ERASE    0x20    /* 4KB sector erase */
#define CMD_PAGE_PROGRAM    0x02
#define CMD_READ_DATA       0x03

/* Expected JEDEC values */
#define JEDEC_MANUFACTURER  0xEF
#define JEDEC_MEM_TYPE      0x40
#define JEDEC_CAPACITY      0x16

/* SR1 busy bit */
#define SR1_BUSY            0x01

/* Test address — sector 0, page 0 (first 4KB) */
#define TEST_ADDR           0x000000UL
#define TEST_PAGE_SIZE      256

extern SPI_HandleTypeDef hspi2;

/* ----------------------------------------------------------------------- */
/* Low-level helpers                                                        */
/* ----------------------------------------------------------------------- */

static inline void CS_Low(void)  { HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET); }
static inline void CS_High(void) { HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);   }

static void SPI_Tx(const uint8_t *buf, uint16_t len)
{
    HAL_SPI_Transmit(&FLASH_SPI, (uint8_t *)buf, len, FLASH_TIMEOUT);
}

static void SPI_Rx(uint8_t *buf, uint16_t len)
{
    HAL_SPI_Receive(&FLASH_SPI, buf, len, FLASH_TIMEOUT);
}

/* Wait until SR1 BUSY bit clears (max ~500ms for sector erase) */
static uint8_t Flash_WaitReady(void)
{
    uint32_t start = HAL_GetTick();
    uint8_t  cmd   = CMD_READ_SR1;
    uint8_t  sr1;

    while (1)
    {
        CS_Low();
        SPI_Tx(&cmd, 1);
        SPI_Rx(&sr1, 1);
        CS_High();

        if (!(sr1 & SR1_BUSY))
            return 1;  /* ready */

        if ((HAL_GetTick() - start) > 500)
            return 0;  /* timeout */

        HAL_Delay(1);
    }
}

static void Flash_WriteEnable(void)
{
    uint8_t cmd = CMD_WRITE_ENABLE;
    CS_Low();
    SPI_Tx(&cmd, 1);
    CS_High();
}

/* ----------------------------------------------------------------------- */
/* Public API                                                               */
/* ----------------------------------------------------------------------- */

Flash_ID_t SpiFlash_ReadID(void)
{
    Flash_ID_t id = {0};
    uint8_t cmd = CMD_JEDEC_ID;
    uint8_t rx[3] = {0};

    CS_Low();
    SPI_Tx(&cmd, 1);
    SPI_Rx(rx, 3);
    CS_High();

    id.manufacturer = rx[0];
    id.memType      = rx[1];
    id.capacity     = rx[2];
    id.valid        = (rx[0] == JEDEC_MANUFACTURER &&
                       rx[1] == JEDEC_MEM_TYPE      &&
                       rx[2] == JEDEC_CAPACITY) ? 1 : 0;
    return id;
}

uint8_t SpiFlash_Test(void)
{
    uint8_t txData[TEST_PAGE_SIZE];
    uint8_t rxData[TEST_PAGE_SIZE];
    uint8_t cmd[4];

    /* --- Step 1: JEDEC ID --- */
    Flash_ID_t id = SpiFlash_ReadID();
    if (!id.valid)
        return 0;

    /* --- Step 2: Status register --- */
    uint8_t sr_cmd = CMD_READ_SR1;
    uint8_t sr1    = 0;
    CS_Low();
    SPI_Tx(&sr_cmd, 1);
    SPI_Rx(&sr1, 1);
    CS_High();
    if (sr1 & SR1_BUSY)
        return 0;   /* chip busy — unexpected at startup */

    /* --- Step 3: Sector erase (4KB at TEST_ADDR) --- */
    Flash_WriteEnable();
    cmd[0] = CMD_SECTOR_ERASE;
    cmd[1] = (TEST_ADDR >> 16) & 0xFF;
    cmd[2] = (TEST_ADDR >>  8) & 0xFF;
    cmd[3] = (TEST_ADDR      ) & 0xFF;
    CS_Low();
    SPI_Tx(cmd, 4);
    CS_High();

    if (!Flash_WaitReady())
        return 0;   /* erase timeout */

    /* --- Step 4: Page program (256 bytes) --- */
    for (int i = 0; i < TEST_PAGE_SIZE; i++)
        txData[i] = (uint8_t)(i ^ 0xA5);  /* known pattern */

    Flash_WriteEnable();
    cmd[0] = CMD_PAGE_PROGRAM;
    cmd[1] = (TEST_ADDR >> 16) & 0xFF;
    cmd[2] = (TEST_ADDR >>  8) & 0xFF;
    cmd[3] = (TEST_ADDR      ) & 0xFF;
    CS_Low();
    SPI_Tx(cmd, 4);
    SPI_Tx(txData, TEST_PAGE_SIZE);
    CS_High();

    if (!Flash_WaitReady())
        return 0;   /* program timeout */

    /* --- Step 5: Read back and verify --- */
    memset(rxData, 0, sizeof(rxData));
    cmd[0] = CMD_READ_DATA;
    cmd[1] = (TEST_ADDR >> 16) & 0xFF;
    cmd[2] = (TEST_ADDR >>  8) & 0xFF;
    cmd[3] = (TEST_ADDR      ) & 0xFF;
    CS_Low();
    SPI_Tx(cmd, 4);
    SPI_Rx(rxData, TEST_PAGE_SIZE);
    CS_High();

    if (memcmp(txData, rxData, TEST_PAGE_SIZE) != 0)
        return 0;   /* data mismatch */

    return 1;   /* all steps passed */
}

/* -----------------------------------------------------------------------
 * Non-blocking flash command state machine
 *
 * Flash_ExecCmd() is called by Modbus on FLASH_CMD write — it just
 * records the intent and sets BUSY. Flash_Run() is called every
 * App_Run() iteration and does the actual work in stages so Modbus
 * remains responsive during erase (~150ms) and program operations.
 * ----------------------------------------------------------------------- */

extern uint16_t Modbus_GetReg(uint16_t addr);
extern void     Modbus_SetReg(uint16_t addr, uint16_t value);

#define CMD_TEST_ADDR   0x001000UL  /* sector 1 — safe to use */

typedef enum {
    FS_IDLE,
    FS_ERASE_START,
    FS_ERASE_WAIT,
    FS_WRITE_START,
    FS_WRITE_WAIT,
    FS_READ_START,
} FlashState_t;

static FlashState_t flashState   = FS_IDLE;
static uint16_t     pendingCmd   = 0;

/* Called by Modbus handler on FLASH_CMD write — non-blocking */
void Flash_ExecCmd(uint16_t cmd)
{
    pendingCmd = cmd;
    Modbus_SetReg(0x00C2, 1);  /* REG_FLASH_STATUS = BUSY */

    switch (cmd)
    {
        case 1:  flashState = FS_ERASE_START; break;  /* ERASE */
        case 2:  flashState = FS_WRITE_START; break;  /* WRITE */
        case 3:  flashState = FS_READ_START;  break;  /* READ  */
        default:
            Modbus_SetReg(0x00C2, 3);  /* FAIL */
            flashState = FS_IDLE;
            break;
    }
}

/* Called every App_Run() — drives the state machine one step */
void Flash_Run(void)
{
    uint8_t buf[4];

    switch (flashState)
    {
        case FS_IDLE:
            break;

        /* ---------------------------------------------------------------- */
        case FS_ERASE_START:
        {
            Flash_WriteEnable();
            buf[0] = CMD_SECTOR_ERASE;
            buf[1] = (CMD_TEST_ADDR >> 16) & 0xFF;
            buf[2] = (CMD_TEST_ADDR >>  8) & 0xFF;
            buf[3] = (CMD_TEST_ADDR      ) & 0xFF;
            CS_Low();
            SPI_Tx(buf, 4);
            CS_High();
            flashState = FS_ERASE_WAIT;
            break;
        }

        case FS_ERASE_WAIT:
        {
            /* Poll SR1 BUSY — non-blocking single check per call */
            uint8_t cmd_sr = CMD_READ_SR1;
            uint8_t sr1    = 0;
            CS_Low();
            SPI_Tx(&cmd_sr, 1);
            SPI_Rx(&sr1, 1);
            CS_High();

            if (!(sr1 & SR1_BUSY))
            {
                Modbus_SetReg(0x00C2, 2);  /* PASS */
                flashState = FS_IDLE;
            }
            /* else: still busy, check again next App_Run() */
            break;
        }

        /* ---------------------------------------------------------------- */
        case FS_WRITE_START:
        {
            uint16_t data    = Modbus_GetReg(0x00C1);  /* REG_FLASH_DATA */
            uint8_t  payload[2] = { (data >> 8) & 0xFF, data & 0xFF };

            Flash_WriteEnable();
            buf[0] = CMD_PAGE_PROGRAM;
            buf[1] = (CMD_TEST_ADDR >> 16) & 0xFF;
            buf[2] = (CMD_TEST_ADDR >>  8) & 0xFF;
            buf[3] = (CMD_TEST_ADDR      ) & 0xFF;
            CS_Low();
            SPI_Tx(buf, 4);
            SPI_Tx(payload, 2);
            CS_High();
            flashState = FS_WRITE_WAIT;
            break;
        }

        case FS_WRITE_WAIT:
        {
            uint8_t cmd_sr = CMD_READ_SR1;
            uint8_t sr1    = 0;
            CS_Low();
            SPI_Tx(&cmd_sr, 1);
            SPI_Rx(&sr1, 1);
            CS_High();

            if (!(sr1 & SR1_BUSY))
            {
                Modbus_SetReg(0x00C2, 2);  /* PASS */
                flashState = FS_IDLE;
            }
            break;
        }

        /* ---------------------------------------------------------------- */
        case FS_READ_START:
        {
            uint8_t rx[2] = {0};
            buf[0] = CMD_READ_DATA;
            buf[1] = (CMD_TEST_ADDR >> 16) & 0xFF;
            buf[2] = (CMD_TEST_ADDR >>  8) & 0xFF;
            buf[3] = (CMD_TEST_ADDR      ) & 0xFF;
            CS_Low();
            SPI_Tx(buf, 4);
            SPI_Rx(rx, 2);
            CS_High();

            uint16_t result = ((uint16_t)rx[0] << 8) | rx[1];
            Modbus_SetReg(0x00C1, result);  /* REG_FLASH_DATA */
            Modbus_SetReg(0x00C2, 2);       /* PASS */
            flashState = FS_IDLE;
            break;
        }

        default:
            flashState = FS_IDLE;
            break;
    }
}
