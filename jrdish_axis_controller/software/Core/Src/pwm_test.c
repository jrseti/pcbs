/* -----------------------------------------------------------------------
 * pwm_test.c
 * Stepper pulse output and direction/enable control
 *
 * AZ axis:
 *   PUL1   = TIM16 CH1, PA6
 *   DIR1   = PC4  (0=CW, 1=CCW)
 *   ENABLE1= PC5  (active LOW — LOW=enabled, HIGH=disabled)
 *
 * EL axis:
 *   PUL2   = TIM3  CH3, PB0
 *   DIR2   = PB1  (0=CW, 1=CCW)
 *   ENABLE2= PB2  (active LOW — LOW=enabled, HIGH=disabled)
 *
 * Timer clock = 170 MHz
 * PSC = 16 → tick = 10 MHz
 * ARR = 999 → 10 kHz at init
 * CCR = 500 → 50% duty cycle
 *
 * Frequency formula: ARR = (170,000,000 / (PSC+1) / hz) - 1
 *                    CCR = (ARR + 1) / 2
 * ----------------------------------------------------------------------- */

#include "pwm_test.h"
#include "main.h"

#define TIMER_CLOCK_HZ  170000000UL
#define PWM_PRESCALER   16

extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim3;

/* ----------------------------------------------------------------------- */
/* Private helpers                                                          */
/* ----------------------------------------------------------------------- */

static void SetTimerFreq(TIM_HandleTypeDef *htim, uint32_t channel,
                         uint32_t hz)
{
    if (hz == 0) return;

    /* Find smallest prescaler that keeps ARR within 16-bit range (<=65535).
     * At fixed PSC=16: min freq = 10MHz/65536 = 152.6 Hz
     * For lower frequencies we increase PSC dynamically. */
    uint32_t psc  = TIMER_CLOCK_HZ / (hz * 65536UL);
    if (psc < PWM_PRESCALER) psc = PWM_PRESCALER;

    uint32_t tick = TIMER_CLOCK_HZ / (psc + 1);
    uint32_t arr  = (tick / hz) - 1;
    uint32_t ccr  = (arr + 1) / 2;  /* 50% duty cycle */

    __HAL_TIM_SET_PRESCALER(htim, psc);
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}

/* ----------------------------------------------------------------------- */
/* Public API                                                               */
/* ----------------------------------------------------------------------- */

void PWM_Test_Init(void)
{
    /* Both drivers disabled at startup (active HIGH — LOW=disabled) */
    HAL_GPIO_WritePin(ENABLE1_GPIO_Port, ENABLE1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ENABLE2_GPIO_Port, ENABLE2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR1_GPIO_Port,    DIR1_Pin,    GPIO_PIN_RESET); /* CW */
    HAL_GPIO_WritePin(DIR2_GPIO_Port,    DIR2_Pin,    GPIO_PIN_RESET);

    /* Start at 100 Hz — user sets desired frequency via Modbus before enabling */
    SetTimerFreq(&htim16, TIM_CHANNEL_1, 100);
    SetTimerFreq(&htim3,  TIM_CHANNEL_3, 100);

    /* Start PWM — pulses will only reach driver when ENABLE is asserted */
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,  TIM_CHANNEL_3);
}

void PWM_Test_Run(void)
{
    /* Placeholder for future ramp/sweep logic */
}

/* --- AZ --- */

void PWM_SetFreq_AZ(uint32_t hz)
{
    SetTimerFreq(&htim16, TIM_CHANNEL_1, hz);
}

void PWM_Enable_AZ(uint8_t en)
{
    /* ENABLE is active HIGH on this board (ENA+ to MCU, ENA- to GND) */
    HAL_GPIO_WritePin(ENABLE1_GPIO_Port, ENABLE1_Pin,
        en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void PWM_SetDir_AZ(uint8_t dir)
{
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin,
        dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* --- EL --- */

void PWM_SetFreq_EL(uint32_t hz)
{
    SetTimerFreq(&htim3, TIM_CHANNEL_3, hz);
}

void PWM_Enable_EL(uint8_t en)
{
    /* ENABLE is active HIGH on this board */
    HAL_GPIO_WritePin(ENABLE2_GPIO_Port, ENABLE2_Pin,
        en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void PWM_SetDir_EL(uint8_t dir)
{
    HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin,
        dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
