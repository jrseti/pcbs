#ifndef ENCODER_H
#define ENCODER_H

/* -----------------------------------------------------------------------
 * encoder.h
 * Quadrature encoder reading
 *
 * AZ = TIM2 (32-bit), CH1=PA0, CH2=PA1
 * EL = TIM1 (16-bit), CH1=PC0, CH2=PC1
 * ----------------------------------------------------------------------- */

#include <stdint.h>

void     Encoder_Init(void);
void     Encoder_Run(void);

int32_t  Encoder_GetAZ(void);
int32_t  Encoder_GetEL(void);
void     Encoder_ResetAZ(void);
void     Encoder_ResetEL(void);

/* Called from EXTI IRQ handlers */
void     Encoder_AZ_Z_Callback(void);
void     Encoder_EL_Z_Callback(void);

#endif /* ENCODER_H */
