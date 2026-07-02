/* -----------------------------------------------------------------------
 * modbus.c
 * Modbus RTU slave driver
 *
 * Handles FC 0x03 (read), 0x06 (write single), 0x10 (write multiple).
 * Supports node addressing via ADDR0/ADDR1/ADDR2 DIP switches.
 * Broadcast address 0xFF is accepted; no response is sent to broadcasts.
 * Uses USART1 (RS485_1) with manual DE pin control.
 * ----------------------------------------------------------------------- */

#include "modbus.h"
#include "main.h"
#include "spi_flash_test.h"
#include "pwm_test.h"
#include <string.h>
#include <stdint.h>

/* ----------------------------------------------------------------------- */
/* Config                                                                   */
/* ----------------------------------------------------------------------- */
#define MB_UART                 huart1
#define MB_DE_PORT              RS485_1_DE_GPIO_Port
#define MB_DE_PIN               RS485_1_DE_Pin

#define MB_RX_BUF_SIZE          128
#define MB_TX_BUF_SIZE          128

/* Inter-frame timeout: 3.5 char times at 115200 = ~0.32ms, use 5ms for robustness */
#define MB_FRAME_TIMEOUT_MS     5

/* ----------------------------------------------------------------------- */
/* Private types                                                            */
/* ----------------------------------------------------------------------- */
typedef enum {
    MB_STATE_IDLE,
    MB_STATE_RECEIVING,
    MB_STATE_FRAME_READY,
} MB_State_t;

/* ----------------------------------------------------------------------- */
/* Private variables                                                        */
/* ----------------------------------------------------------------------- */
extern UART_HandleTypeDef huart1;

static uint16_t   regs[MODBUS_REG_COUNT];
static uint8_t    nodeAddr = 1;
static uint8_t    isBroadcast = 0;

static uint8_t    rxBuf[MB_RX_BUF_SIZE];
static uint8_t    rxByte;
static uint8_t    rxLen = 0;
static MB_State_t rxState = MB_STATE_IDLE;
static uint32_t   lastByteTime = 0;

static uint8_t    txBuf[MB_TX_BUF_SIZE];

/* ----------------------------------------------------------------------- */
/* CRC-16 (Modbus)                                                         */
/* ----------------------------------------------------------------------- */
static uint16_t CRC16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ----------------------------------------------------------------------- */
/* Read node address from ADDR0/ADDR1/ADDR2 GPIO pins                     */
/* Pins have pull-ups; DIP switch pulls to GND when closed.               */
/* GPIO_PIN_RESET = switch closed = bit set                               */
/* ----------------------------------------------------------------------- */
static uint8_t ReadNodeAddress(void)
{
    uint8_t addr = 0;
    if (HAL_GPIO_ReadPin(ADDR0_GPIO_Port, ADDR0_Pin) == GPIO_PIN_RESET) addr |= (1 << 0);
    if (HAL_GPIO_ReadPin(ADDR1_GPIO_Port, ADDR1_Pin) == GPIO_PIN_RESET) addr |= (1 << 1);
    if (HAL_GPIO_ReadPin(ADDR2_GPIO_Port, ADDR2_Pin) == GPIO_PIN_RESET) addr |= (1 << 2);
    return (addr == 0) ? 1 : addr;  /* default to 1 if all switches open */
}

/* ----------------------------------------------------------------------- */
/* Transmit response                                                        */
/* ----------------------------------------------------------------------- */
static void MB_Transmit(uint8_t *buf, uint16_t len)
{
    HAL_GPIO_WritePin(MB_DE_PORT, MB_DE_PIN, GPIO_PIN_SET);
    HAL_UART_Transmit(&MB_UART, buf, len, 100);
    while (__HAL_UART_GET_FLAG(&MB_UART, UART_FLAG_TC) == RESET);
    HAL_GPIO_WritePin(MB_DE_PORT, MB_DE_PIN, GPIO_PIN_RESET);
}

/* ----------------------------------------------------------------------- */
/* Send exception response                                                  */
/* ----------------------------------------------------------------------- */
static void MB_SendException(uint8_t fc, uint8_t exCode)
{
    uint8_t buf[5];
    buf[0] = nodeAddr;
    buf[1] = fc | 0x80;
    buf[2] = exCode;
    uint16_t crc = CRC16(buf, 3);
    buf[3] = crc & 0xFF;
    buf[4] = (crc >> 8) & 0xFF;
    MB_Transmit(buf, 5);
}

/* ----------------------------------------------------------------------- */
/* Handle FC 0x03 — Read Holding Registers                                 */
/* ----------------------------------------------------------------------- */
static void MB_HandleReadRegs(const uint8_t *frame)
{
    uint16_t startAddr = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t qty       = ((uint16_t)frame[4] << 8) | frame[5];

    if (qty < 1 || qty > 125)
    {
        MB_SendException(FC_READ_REGS, MB_EX_ILLEGAL_VALUE);
        return;
    }
    if ((startAddr + qty) > MODBUS_REG_COUNT)
    {
        MB_SendException(FC_READ_REGS, MB_EX_ILLEGAL_ADDR);
        return;
    }

    uint16_t txLen = 0;
    txBuf[txLen++] = nodeAddr;
    txBuf[txLen++] = FC_READ_REGS;
    txBuf[txLen++] = (uint8_t)(qty * 2);

    for (uint16_t i = 0; i < qty; i++)
    {
        uint16_t val = regs[startAddr + i];
        txBuf[txLen++] = (val >> 8) & 0xFF;
        txBuf[txLen++] = val & 0xFF;
    }

    uint16_t crc = CRC16(txBuf, txLen);
    txBuf[txLen++] = crc & 0xFF;
    txBuf[txLen++] = (crc >> 8) & 0xFF;

    MB_Transmit(txBuf, txLen);
}

/* ----------------------------------------------------------------------- */
/* Handle FC 0x06 — Write Single Register                                  */
/* ----------------------------------------------------------------------- */
static void MB_HandleWriteSingle(const uint8_t *frame)
{
    uint16_t addr  = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t value = ((uint16_t)frame[4] << 8) | frame[5];

    if (addr >= MODBUS_REG_COUNT)
    {
        if (!isBroadcast) MB_SendException(FC_WRITE_SINGLE, MB_EX_ILLEGAL_ADDR);
        return;
    }

    /* Special handling for SYS_RESET */
    if (addr == REG_SYS_RESET && value == MODBUS_RESET_KEY)
    {
        /* Echo first, then reset */
        if (!isBroadcast)
        {
            uint8_t echo[8];
            memcpy(echo, frame, 8);
            MB_Transmit(echo, 8);
        }
        HAL_Delay(10);
        NVIC_SystemReset();
        return;
    }

    regs[addr] = value;

    /* Side effects */
    if (addr == REG_SYS_FAN)
        HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin,
            value ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (addr == REG_FLASH_CMD)
        Flash_ExecCmd(value);

    /* AZ motor control */
    if (addr == REG_AZ_CMD_ENABLE) PWM_Enable_AZ(value);
    if (addr == REG_AZ_CMD_STOP  && value) { PWM_Enable_AZ(0); regs[REG_AZ_CMD_ENABLE] = 0; }

    /* EL motor control */
    if (addr == REG_EL_CMD_ENABLE) PWM_Enable_EL(value);
    if (addr == REG_EL_CMD_STOP  && value) { PWM_Enable_EL(0); regs[REG_EL_CMD_ENABLE] = 0; }

    /* PWM frequency */
    if (addr == REG_AZ_PWM_FREQ && value > 0) PWM_SetFreq_AZ(value);
    if (addr == REG_EL_PWM_FREQ && value > 0) PWM_SetFreq_EL(value);

    /* PWM direction */
    if (addr == REG_AZ_PWM_DIR) PWM_SetDir_AZ(value);
    if (addr == REG_EL_PWM_DIR) PWM_SetDir_EL(value);

    /* Echo frame as response (standard Modbus) */
    if (!isBroadcast)
    {
        uint8_t echo[8];
        memcpy(echo, frame, 8);
        MB_Transmit(echo, 8);
    }
}

/* ----------------------------------------------------------------------- */
/* Handle FC 0x10 — Write Multiple Registers                               */
/* ----------------------------------------------------------------------- */
static void MB_HandleWriteMulti(const uint8_t *frame)
{
    uint16_t startAddr = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t qty       = ((uint16_t)frame[4] << 8) | frame[5];
    uint8_t  byteCount = frame[6];

    if (qty < 1 || qty > 123 || byteCount != qty * 2)
    {
        if (!isBroadcast) MB_SendException(FC_WRITE_MULTI, MB_EX_ILLEGAL_VALUE);
        return;
    }
    if ((startAddr + qty) > MODBUS_REG_COUNT)
    {
        if (!isBroadcast) MB_SendException(FC_WRITE_MULTI, MB_EX_ILLEGAL_ADDR);
        return;
    }

    for (uint16_t i = 0; i < qty; i++)
    {
        uint16_t val = ((uint16_t)frame[7 + i*2] << 8) | frame[8 + i*2];
        regs[startAddr + i] = val;

        /* Side effects per register */
        if ((startAddr + i) == REG_SYS_FAN)
            HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin,
                val ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    if (!isBroadcast)
    {
        uint16_t txLen = 0;
        txBuf[txLen++] = nodeAddr;
        txBuf[txLen++] = FC_WRITE_MULTI;
        txBuf[txLen++] = (startAddr >> 8) & 0xFF;
        txBuf[txLen++] = startAddr & 0xFF;
        txBuf[txLen++] = (qty >> 8) & 0xFF;
        txBuf[txLen++] = qty & 0xFF;
        uint16_t crc = CRC16(txBuf, txLen);
        txBuf[txLen++] = crc & 0xFF;
        txBuf[txLen++] = (crc >> 8) & 0xFF;
        MB_Transmit(txBuf, txLen);
    }
}

/* ----------------------------------------------------------------------- */
/* Process a complete received frame                                        */
/* ----------------------------------------------------------------------- */
static void MB_ProcessFrame(void)
{
    if (rxLen < 4)
        return;

    uint8_t frameAddr = rxBuf[0];

    /* Address filter */
    if (frameAddr != nodeAddr && frameAddr != MODBUS_BROADCAST_ADDR)
        return;

    isBroadcast = (frameAddr == MODBUS_BROADCAST_ADDR);

    /* CRC check */
    uint16_t rxCRC  = ((uint16_t)rxBuf[rxLen-1] << 8) | rxBuf[rxLen-2];
    uint16_t calCRC = CRC16(rxBuf, rxLen - 2);
    if (rxCRC != calCRC)
        return;

    /* Mark RS485 active */
    regs[REG_SYS_STATUS] |= SYS_STATUS_RS485_ACTIVE;

    uint8_t fc = rxBuf[1];
    switch (fc)
    {
        case FC_READ_REGS:
            if (!isBroadcast && rxLen >= 8)
                MB_HandleReadRegs(rxBuf);
            break;

        case FC_WRITE_SINGLE:
            if (rxLen >= 8)
                MB_HandleWriteSingle(rxBuf);
            break;

        case FC_WRITE_MULTI:
            if (rxLen >= 9)
                MB_HandleWriteMulti(rxBuf);
            break;

        default:
            if (!isBroadcast)
                MB_SendException(fc, MB_EX_ILLEGAL_FUNC);
            break;
    }
}

/* HAL_UART_RxCpltCallback is defined in stm32g4xx_it.c
 * which dispatches to GPS_UART_Callback and Modbus internals. */
void Modbus_UART_RxByte(void)
{
    lastByteTime = HAL_GetTick();

    if (rxLen < MB_RX_BUF_SIZE)
        rxBuf[rxLen++] = rxByte;

    rxState = MB_STATE_RECEIVING;

    HAL_UART_Receive_IT(&MB_UART, &rxByte, 1);
}

/* ----------------------------------------------------------------------- */
/* Public API                                                               */
/* ----------------------------------------------------------------------- */

void Modbus_Init(void)
{
    memset(regs, 0, sizeof(regs));

    nodeAddr = ReadNodeAddress();

    /* Pre-populate read-only system registers */
    regs[REG_SYS_ADDR]   = nodeAddr;
    regs[REG_SYS_FW_VER] = MODBUS_FW_VERSION;

    /* Default axis config */
    regs[REG_AZ_ENC_PPR]       = 2000;
    regs[REG_AZ_GEAR_RATIO_LO] = 6000;
    regs[REG_AZ_PULLEY_RATIO]  = 100;
    regs[REG_AZ_PID_MAX]       = 1000;

    regs[REG_EL_ENC_PPR]       = 2000;
    regs[REG_EL_GEAR_RATIO_LO] = 6000;
    regs[REG_EL_PULLEY_RATIO]  = 100;
    regs[REG_EL_PID_MAX]       = 1000;

    /* Fan off */
    HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);

    /* Default PWM frequencies and directions — start at 0 (stopped) */
    regs[REG_AZ_PWM_FREQ] = 0;
    regs[REG_EL_PWM_FREQ] = 0;
    regs[REG_AZ_PWM_DIR]  = 0;
    regs[REG_EL_PWM_DIR]  = 0;

    /* DE low = receive mode */
    HAL_GPIO_WritePin(MB_DE_PORT, MB_DE_PIN, GPIO_PIN_RESET);

    /* Start receive */
    HAL_UART_Receive_IT(&MB_UART, &rxByte, 1);
}

void Modbus_Run(void)
{
    /* Update live read-only registers */
    uint32_t uptime = HAL_GetTick();
    regs[REG_SYS_UPTIME_HI] = (uint16_t)(uptime >> 16);
    regs[REG_SYS_UPTIME_LO] = (uint16_t)(uptime & 0xFFFF);

    /* USER_SW — active low, invert so 1=pressed */
    uint16_t sw = 0;
    if (HAL_GPIO_ReadPin(USER_SW_1_GPIO_Port, USER_SW_1_Pin) == GPIO_PIN_RESET) sw |= (1 << 0);
    if (HAL_GPIO_ReadPin(USER_SW_2_GPIO_Port, USER_SW_2_Pin) == GPIO_PIN_RESET) sw |= (1 << 1);
    if (HAL_GPIO_ReadPin(USER_SW_3_GPIO_Port, USER_SW_3_Pin) == GPIO_PIN_RESET) sw |= (1 << 2);
    if (HAL_GPIO_ReadPin(USER_SW_4_GPIO_Port, USER_SW_4_Pin) == GPIO_PIN_RESET) sw |= (1 << 3);
    regs[REG_SYS_USER_SW] = sw;

    /* Frame timeout detection — process frame after silence gap */
    if (rxState == MB_STATE_RECEIVING &&
        (HAL_GetTick() - lastByteTime) >= MB_FRAME_TIMEOUT_MS)
    {
        rxState = MB_STATE_FRAME_READY;
    }

    if (rxState == MB_STATE_FRAME_READY)
    {
        MB_ProcessFrame();
        rxLen   = 0;
        rxState = MB_STATE_IDLE;
    }
}

/* ----------------------------------------------------------------------- */
/* Register access helpers                                                  */
/* ----------------------------------------------------------------------- */

uint16_t Modbus_GetReg(uint16_t addr)
{
    if (addr >= MODBUS_REG_COUNT) return 0;
    return regs[addr];
}

void Modbus_SetReg(uint16_t addr, uint16_t value)
{
    if (addr >= MODBUS_REG_COUNT) return;
    regs[addr] = value;
}

void Modbus_SetReg32(uint16_t addrHi, uint32_t value)
{
    regs[addrHi]     = (uint16_t)(value >> 16);
    regs[addrHi + 1] = (uint16_t)(value & 0xFFFF);
}

uint32_t Modbus_GetReg32(uint16_t addrHi)
{
    return ((uint32_t)regs[addrHi] << 16) | regs[addrHi + 1];
}

void Modbus_SetRegFloat(uint16_t addrHi, float value)
{
    uint32_t raw;
    memcpy(&raw, &value, sizeof(float));
    Modbus_SetReg32(addrHi, raw);
}

float Modbus_GetRegFloat(uint16_t addrHi)
{
    uint32_t raw = Modbus_GetReg32(addrHi);
    float value;
    memcpy(&value, &raw, sizeof(float));
    return value;
}
