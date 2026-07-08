#ifndef PWM_TEST_H
#define PWM_TEST_H

/* -----------------------------------------------------------------------
 * pwm_test.h
 * Stepper pulse output test — PUL1 (TIM16 CH1, PA6) and PUL2 (TIM3 CH3, PB0)
 * DIR1=PC4, ENABLE1=PC5 (EL — connector 1)
 * DIR2=PB1, ENABLE2=PB2 (AZ — connector 2)
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

/* Homing — drives CW to LIMIT1, zeroes the encoder there, sets AXIS_STATUS_HOMED */
void AZ_StartHoming(void);
void EL_StartHoming(void);

/* Position moves — target angle (degrees) is read from REG_AZ_CMD_POS_HI/LO
 * (float32); converted to encoder counts via the LIM1/LIM2 calibration */
void AZ_StartMove(void);
void EL_StartMove(void);

/* Abort any in-progress homing or position move (called on REG_AZ_CMD_STOP) */
void PWM_AbortMotion_AZ(void);
void PWM_AbortMotion_EL(void);

#endif /* PWM_TEST_H */
