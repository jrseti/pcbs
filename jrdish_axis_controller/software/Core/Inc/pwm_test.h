#ifndef PWM_TEST_H
#define PWM_TEST_H

/* -----------------------------------------------------------------------
 * pwm_test.h
 * Stepper pulse output test — PUL1 (TIM16 CH1, PA6) and PUL2 (TIM3 CH3, PB0)
 * DIR1=PC4, ENABLE1=PC5 (AZ)
 * DIR2=PB1, ENABLE2=PB2 (EL)
 * ENABLE is active LOW on DM556Y.
 * ----------------------------------------------------------------------- */

#include <stdint.h>

void PWM_Test_Init(void);
void PWM_Test_Run(void);

/* Pulse frequency */
void PWM_SetFreq_AZ(uint32_t hz);
void PWM_SetFreq_EL(uint32_t hz);

/* Enable/disable pulse outputs — also controls ENABLE pin to driver */
void PWM_Enable_AZ(uint8_t en);
void PWM_Enable_EL(uint8_t en);

/* Direction: 0=CW, 1=CCW */
void PWM_SetDir_AZ(uint8_t dir);
void PWM_SetDir_EL(uint8_t dir);

#endif /* PWM_TEST_H */
