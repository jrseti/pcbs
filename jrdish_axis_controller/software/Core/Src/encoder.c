/* -----------------------------------------------------------------------
 * encoder.c
 * Quadrature encoder reading via STM32 timer encoder interface
 *
 * Connector 1 (ENC1) drives the EL axis — TIM2 (32-bit counter), PA0=CH1, PA1=CH2
 * Connector 2 (ENC2) drives the AZ axis — TIM1 (16-bit counter), PC0=CH1, PC1=PC2
 *
 * TIM2 is 32-bit so EL position is read directly as int32_t.
 * TIM1 is 16-bit so AZ position is tracked with an overflow counter
 * to give a full int32_t range.
 *
 * Encoder mode: both edges on both channels (4x counting).
 * E6B2-CWZ6C: 2000 PPR x4 = 8000 counts/encoder rev.
 *
 * Counts-per-dish-revolution isn't a fixed constant here — position is
 * derived from two-point calibration against the limit switches (see
 * pwm_test.c: LIM1_ANGLE/LIM2_ANGLE/LIM2_POS), not from PPR x gear ratio.
 * ----------------------------------------------------------------------- */

#include "encoder.h"
#include "modbus.h"
#include "modbus_regs.h"
#include "main.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

/* ----------------------------------------------------------------------- */
/* AZ overflow tracking (TIM1 is 16-bit)                                   */
/* ----------------------------------------------------------------------- */
static uint16_t azLastCount   = 0;     /* previous TIM1 CNT value          */
static int32_t  azPosition    = 0;     /* accumulated AZ position          */

/* ----------------------------------------------------------------------- */
/* Z pulse counters — incremented from EXTI IRQ                            */
/* ----------------------------------------------------------------------- */
static volatile uint16_t azZCount = 0;
static volatile uint16_t elZCount = 0;

/* ----------------------------------------------------------------------- */
/* Limit switch debounce — raw mechanical contacts chatter (bench-confirmed
 * via LIMIT_PINS prints: holding a switch steady still toggled the raw
 * reading). A bit only takes effect once the raw pin has held that value
 * continuously for LIMIT_DEBOUNCE_MS.                                     */
/* ----------------------------------------------------------------------- */
#define LIMIT_DEBOUNCE_MS   10UL

typedef struct { uint8_t stable; uint8_t lastRaw; uint32_t lastChangeMs; } Debounce_t;
static Debounce_t azLim1Db, azLim2Db, elLim1Db, elLim2Db;

static uint8_t Debounce(Debounce_t *d, uint8_t raw)
{
    uint32_t now = HAL_GetTick();
    if (raw != d->lastRaw)
    {
        d->lastRaw = raw;
        d->lastChangeMs = now;
    }
    else if (now - d->lastChangeMs >= LIMIT_DEBOUNCE_MS)
    {
        d->stable = raw;
    }
    return d->stable;
}

/* Called from HAL_GPIO_EXTI_Callback in stm32g4xx_it.c for ENC2_Z (PC3) */
void Encoder_AZ_Z_Callback(void)
{
    azZCount++;
    Modbus_SetReg(REG_AZ_Z_COUNT, azZCount);
    printf("AZ Z pulse #%u\r\n", azZCount);
}

/* Called from HAL_GPIO_EXTI_Callback for ENC1_Z (PA2) */
void Encoder_EL_Z_Callback(void)
{
    elZCount++;
    Modbus_SetReg(REG_EL_Z_COUNT, elZCount);
    printf("EL Z pulse #%u\r\n", elZCount);
}

/* ----------------------------------------------------------------------- */
/* Public API                                                               */
/* ----------------------------------------------------------------------- */

void Encoder_Init(void)
{
    static uint8_t initialized = 0;
    if (initialized) { printf("Encoder_Init: already initialized\r\n"); return; }
    initialized = 1;

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(&htim2, 0x80000000UL);
    __HAL_TIM_SET_COUNTER(&htim1, 0x8000U);

    azLastCount = 0x8000U;
    azPosition  = 0;

    printf("Encoder_Init done\r\n");
}

int32_t Encoder_GetAZ(void)
{
    /* Connector 2 / TIM1 is 16-bit — track overflow/underflow manually */
    uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
    int16_t  delta   = (int16_t)(current - azLastCount);
    azLastCount      = current;

    /* Accumulate into 32-bit position */
    azPosition += delta;
    return azPosition;
}

int32_t Encoder_GetEL(void)
{
    /* Connector 1 / TIM2 is 32-bit — direct signed read with midpoint offset.
     * Negated: on the bench, CW motor rotation counts down here despite
     * identical encoder wiring/timer config to AZ (TIM1/TIM2 IC polarity and
     * mode are identical) — a mechanical asymmetry between the two axes.
     * Flipped so CW increases position on both axes, matching AZ. */
    uint32_t raw = __HAL_TIM_GET_COUNTER(&htim2);
    return -(int32_t)(raw - 0x80000000UL);
}

void Encoder_ResetAZ(void)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0x8000U);
    azLastCount = 0x8000U;
    azPosition  = 0;
}

void Encoder_ResetEL(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0x80000000UL);
}

void Encoder_Run(void)
{
    int32_t az = Encoder_GetAZ();
    int32_t el = Encoder_GetEL();

    /* Update Modbus position registers */
    Modbus_SetReg32(REG_AZ_POS_HI, (uint32_t)az);
    Modbus_SetReg32(REG_EL_POS_HI, (uint32_t)el);

    /* Read limit switches — active LOW, invert so 1=triggered.
     * Connector 2 (LIMIT2_*) is the AZ axis, connector 1 (LIMIT1_*) is EL. */
    uint8_t azLim1Raw = (HAL_GPIO_ReadPin(LIMIT2_SW1_GPIO_Port, LIMIT2_SW1_Pin) == GPIO_PIN_RESET);
    uint8_t azLim2Raw = (HAL_GPIO_ReadPin(LIMIT2_SW2_GPIO_Port, LIMIT2_SW2_Pin) == GPIO_PIN_RESET);
    uint8_t elLim1Raw = (HAL_GPIO_ReadPin(LIMIT1_SW1_GPIO_Port, LIMIT1_SW1_Pin) == GPIO_PIN_RESET);
    uint8_t elLim2Raw = (HAL_GPIO_ReadPin(LIMIT1_SW2_GPIO_Port, LIMIT1_SW2_Pin) == GPIO_PIN_RESET);

    uint8_t azLim1 = Debounce(&azLim1Db, azLim1Raw);
    uint8_t azLim2 = Debounce(&azLim2Db, azLim2Raw);
    uint8_t elLim1 = Debounce(&elLim1Db, elLim1Raw);
    uint8_t elLim2 = Debounce(&elLim2Db, elLim2Raw);

    uint16_t lim = 0;
    if (azLim1) lim |= (1 << 0);
    if (azLim2) lim |= (1 << 1);
    if (elLim1) lim |= (1 << 2);
    if (elLim2) lim |= (1 << 3);
    Modbus_SetReg(REG_LIMIT_SW, lim);

    /* Print only on change — Encoder_Run() ticks every main-loop iteration,
     * so printing unconditionally would flood the SWO console. A print per
     * transition still catches a single-tick glitch (one line in, one line
     * back out), which is exactly what a per-tick print would show anyway. */
    static uint16_t lastPrinted = 0xFFFF;  /* force a print on the first tick */
    if (lim != lastPrinted)
    {
        printf("LIMIT_PINS: AZ_LIM1=%u AZ_LIM2=%u EL_LIM1=%u EL_LIM2=%u\r\n",
               azLim1, azLim2, elLim1, elLim2);
        lastPrinted = lim;
    }
}
