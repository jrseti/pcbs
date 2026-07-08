/* -----------------------------------------------------------------------
 * pwm_test.c
 * Stepper pulse output and direction/enable control, plus homing and
 * calibration-based position-move control loops.
 *
 * EL axis (connector 1):
 *   PUL1   = TIM16 CH1, PA6
 *   DIR1   = PC4  (0=CW, 1=CCW)
 *   ENABLE1= PC5  (active LOW — LOW=enabled, HIGH=disabled)
 *
 * AZ axis (connector 2):
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
 *
 * Position is tracked via two-point calibration (LIM1_ANGLE/LIM2_ANGLE/
 * LIM2_POS, set by the operator via an external angle instrument), not
 * from gear ratio — this is a stepper system, so velocity is open-loop
 * (commanded pulse rate) and the encoder is used purely for position
 * feedback/stopping. Gear ratio + driver PPR are only used to convert a
 * commanded output RPM into a pulse frequency (see AZ_RPM_to_Hz).
 * ----------------------------------------------------------------------- */

#include "pwm_test.h"
#include "main.h"
#include "modbus.h"
#include "modbus_regs.h"
#include "encoder.h"
#include <math.h>
#include <stdio.h>

#define TIMER_CLOCK_HZ  170000000UL
#define PWM_PRESCALER   16

#define AXIS_HOME_TIMEOUT_MS    300000UL  /* 5 min safety abort while seeking LIMIT1 */
#define AXIS_POS_TOLERANCE_DEG  0.01f     /* stopping deadband */

/* A stepper can't jump straight from a stop to its cruise pulse rate — a
 * high enough GEAR_RATIO x DRIVER_PPR product turns even a modest output
 * RPM into thousands of Hz at the motor, and commanding that instantly
 * just stalls it (whines, doesn't turn). AXIS_START_HZ is the rate it can
 * reliably pull in from a stop; AXIS_ACCEL_HZ_PER_SEC ramps up from there.
 * Both are first-pass values — tune them (or ask to make them registers) if
 * the motor still stalls, or if it ramps too slowly/quickly for the load. */
#define AXIS_START_HZ           200
#define AXIS_ACCEL_HZ_PER_SEC   2000.0f

/* PWM_SetDir_AZ/EL()'s dir argument and azCurrentDir/elCurrentDir's meaning. */
#define AXIS_DIR_CW             0
#define AXIS_DIR_CCW            1

/* REG_LIMIT_SW bit positions (see modbus_regs.h) */
#define AZ_LIM1_BIT             (1 << 0)
#define AZ_LIM2_BIT             (1 << 1)
#define EL_LIM1_BIT             (1 << 2)
#define EL_LIM2_BIT             (1 << 3)

extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim3;

/* Homing is a 3-phase approach for a repeatable trigger point: a fast switch
 * hit is momentum/overtravel-dependent and not very repeatable run to run.
 * SEEKING drives CW into LIMIT1 at normal homing speed; once triggered,
 * BACKING_OFF reverses just far enough to release the switch; APPROACHING
 * then creeps back in at a much slower speed, and *that* trigger point is
 * what actually gets zeroed. */
#define HOME_IDLE           0
#define HOME_SEEKING        1
#define HOME_BACKING_OFF    2
#define HOME_APPROACHING    3
/* A move is a 2-phase approach for the same reason homing is: stopping
 * directly on whichever side the move started from leaves the last bit of
 * position dependent on drivetrain backlash, which showed up as tens of
 * degrees of real-world error even though the encoder count matched the
 * target exactly. SEEKING drives at full speed to a point MOVE_APPROACH_
 * MARGIN_DEG short of the target on the CCW side (overshooting past it if
 * the move started from below); APPROACHING then always creeps in CW, the
 * same final direction LIM1/home uses, so backlash is taken up identically
 * on every move regardless of starting position. */
#define MOVE_IDLE         0
#define MOVE_SEEKING      1
#define MOVE_APPROACHING  2

#define HOME_SEEK_RPM_FRACTION   0.2f   /* seek + back-off speed, fraction of MAX_RPM */
#define HOME_SEEK_RPM_MIN        0.05f
#define HOME_CREEP_RPM_FRACTION  0.05f  /* final slow creep-in speed, fraction of MAX_RPM */
#define HOME_CREEP_RPM_MIN       0.02f

/* Keep backing off for this long *after* the switch releases, rather than
 * stopping the instant it releases — a bare release point is a small, hard
 * to perceive amount of travel; this guarantees an audible/visible margin. */
#define HOME_BACKOFF_EXTRA_MS    300UL

/* How far short of the target (on the CCW/overshoot side) a move's final
 * approach starts from — must be comfortably larger than the drivetrain's
 * actual backlash to fully cancel it out. */
#define MOVE_APPROACH_MARGIN_DEG  1.0f

static uint8_t     azHomeState = HOME_IDLE;
static uint8_t     elHomeState = HOME_IDLE;
static uint8_t     azMoveState = MOVE_IDLE;
static uint8_t     elMoveState = MOVE_IDLE;
static int32_t     azTargetCounts = 0;
static int32_t     elTargetCounts = 0;
static int32_t     azApproachCounts = 0;  /* MOVE_APPROACH_MARGIN_DEG short of azTargetCounts */
static int32_t     elApproachCounts = 0;
static uint32_t    azHomeStartMs  = 0;
static uint32_t    elHomeStartMs  = 0;
static uint8_t     azBackoffReleased  = 0;  /* HOME_BACKING_OFF: has the switch released yet? */
static uint8_t     elBackoffReleased  = 0;
static uint32_t    azBackoffReleaseMs = 0;  /* tick when it first released */
static uint32_t    elBackoffReleaseMs = 0;
static uint8_t     azCurrentDir   = AXIS_DIR_CW;  /* mirrors the last PWM_SetDir_AZ() call */
static uint8_t     elCurrentDir   = AXIS_DIR_CW;
static uint32_t    azCurrentHz    = 0;  /* actual applied pulse rate (0 = stopped) */
static uint32_t    elCurrentHz    = 0;
static uint32_t    azTargetHz     = 0;  /* ramp target while homing/moving */
static uint32_t    elTargetHz     = 0;
static uint32_t    azRampLastMs   = 0;
static uint32_t    elRampLastMs   = 0;

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

static void SetStatusBits(uint16_t statusReg, uint16_t setMask, uint16_t clearMask)
{
    uint16_t s = Modbus_GetReg(statusReg);
    s |= setMask;
    s &= (uint16_t)~clearMask;
    Modbus_SetReg(statusReg, s);
}

/* Human-readable formatters for the AZ_INTERLOCK/EL_INTERLOCK debug prints —
 * spell out names instead of raw hex/decimal so a switch/direction mix-up is
 * obvious on sight rather than requiring a bit-mask lookup. */
static const char *DirStr(uint8_t dir)       { return (dir == AXIS_DIR_CW) ? "CW" : "CCW"; }
static const char *HomeStateStr(uint8_t s)
{
    switch (s)
    {
        case HOME_SEEKING:     return "HOME_SEEKING";
        case HOME_BACKING_OFF: return "HOME_BACKING_OFF";
        case HOME_APPROACHING: return "HOME_APPROACHING";
        default:               return "HOME_IDLE";
    }
}
static const char *MoveStateStr(uint8_t s)
{
    switch (s)
    {
        case MOVE_SEEKING:     return "MOVE_SEEKING";
        case MOVE_APPROACHING: return "MOVE_APPROACHING";
        default:               return "MOVE_IDLE";
    }
}

static const char *AZ_LimStr(uint16_t lim)
{
    uint8_t l1 = (lim & AZ_LIM1_BIT) != 0;
    uint8_t l2 = (lim & AZ_LIM2_BIT) != 0;
    if (l1 && l2) return "LIM1+LIM2";
    if (l1)       return "LIM1";
    if (l2)       return "LIM2";
    return "NONE";
}

static const char *EL_LimStr(uint16_t lim)
{
    uint8_t l1 = (lim & EL_LIM1_BIT) != 0;
    uint8_t l2 = (lim & EL_LIM2_BIT) != 0;
    if (l1 && l2) return "LIM1+LIM2";
    if (l1)       return "LIM1";
    if (l2)       return "LIM2";
    return "NONE";
}

/* On this hardware, sitting at the true CW/LIM1 end reads only the LIM1 bit,
 * but sitting at the CCW/LIM2 end has been observed to also spuriously read
 * LIM1 (bench-confirmed, cause unconfirmed — not a wiring fault). Homing's
 * "already home" checks must not trust LIM1 in isolation, or commanding a
 * home while sitting at the far end would falsely declare success and zero
 * the encoder at the wrong physical position. */
static uint8_t AZ_AtLim1(uint16_t lim) { return (lim & AZ_LIM1_BIT) && !(lim & AZ_LIM2_BIT); }
static uint8_t EL_AtLim1(uint16_t lim) { return (lim & EL_LIM1_BIT) && !(lim & EL_LIM2_BIT); }

/* rpm (output shaft) -> stepper pulse frequency (Hz), via gear ratio and the
 * DM556Y's own pulses/rev DIP-switch setting. Encoder PPR plays no part —
 * velocity here is open-loop/commanded, not derived from encoder feedback. */
static uint32_t AZ_RPM_to_Hz(float rpm)
{
    uint32_t gearRatio = Modbus_GetReg32(REG_AZ_GEAR_RATIO_HI);
    uint16_t ppr       = Modbus_GetReg(REG_AZ_DRIVER_PPR);
    float hz = (rpm / 60.0f) * (float)gearRatio * (float)ppr;
    if (hz < 1.0f)     hz = 1.0f;
    if (hz > 10000.0f) hz = 10000.0f;
    return (uint32_t)(hz + 0.5f);
}

static uint32_t EL_RPM_to_Hz(float rpm)
{
    uint32_t gearRatio = Modbus_GetReg32(REG_EL_GEAR_RATIO_HI);
    uint16_t ppr       = Modbus_GetReg(REG_EL_DRIVER_PPR);
    float hz = (rpm / 60.0f) * (float)gearRatio * (float)ppr;
    if (hz < 1.0f)     hz = 1.0f;
    if (hz > 10000.0f) hz = 10000.0f;
    return (uint32_t)(hz + 0.5f);
}

/* Speed for the initial seek and the back-off leg of homing. */
static float HomeSeekRpm(float maxRpm)
{
    float rpm = maxRpm * HOME_SEEK_RPM_FRACTION;
    return (rpm < HOME_SEEK_RPM_MIN) ? HOME_SEEK_RPM_MIN : rpm;
}

/* Much slower speed for the final creep-in leg, so the switch's actual
 * trigger point is repeatable run to run instead of overtravel-dependent. */
static float HomeCreepRpm(float maxRpm)
{
    float rpm = maxRpm * HOME_CREEP_RPM_FRACTION;
    return (rpm < HOME_CREEP_RPM_MIN) ? HOME_CREEP_RPM_MIN : rpm;
}

/* Ramp azCurrentHz/elCurrentHz toward azTargetHz/elTargetHz and apply it.
 * Call every tick while homing/moving. From a stop, jumps straight to
 * AXIS_START_HZ (ramping from 0 makes no physical sense), then ramps the
 * rest of the way at AXIS_ACCEL_HZ_PER_SEC. */
static void AZ_ApplyRamp(void)
{
    uint32_t now = HAL_GetTick();
    if (azCurrentHz == 0)
    {
        azCurrentHz = (AXIS_START_HZ < azTargetHz) ? AXIS_START_HZ : azTargetHz;
    }
    else
    {
        uint32_t elapsedMs = now - azRampLastMs;
        uint32_t step = (uint32_t)(AXIS_ACCEL_HZ_PER_SEC * elapsedMs / 1000.0f);
        if (step < 1) step = 1;
        if (azCurrentHz < azTargetHz)
        {
            azCurrentHz += step;
            if (azCurrentHz > azTargetHz) azCurrentHz = azTargetHz;
        }
        else if (azCurrentHz > azTargetHz)
        {
            azCurrentHz = (azCurrentHz > azTargetHz + step) ? (azCurrentHz - step) : azTargetHz;
        }
    }
    azRampLastMs = now;
    PWM_SetFreq_AZ(azCurrentHz);
    Modbus_SetReg(REG_AZ_PWM_FREQ, azCurrentHz);
}

static void EL_ApplyRamp(void)
{
    uint32_t now = HAL_GetTick();
    if (elCurrentHz == 0)
    {
        elCurrentHz = (AXIS_START_HZ < elTargetHz) ? AXIS_START_HZ : elTargetHz;
    }
    else
    {
        uint32_t elapsedMs = now - elRampLastMs;
        uint32_t step = (uint32_t)(AXIS_ACCEL_HZ_PER_SEC * elapsedMs / 1000.0f);
        if (step < 1) step = 1;
        if (elCurrentHz < elTargetHz)
        {
            elCurrentHz += step;
            if (elCurrentHz > elTargetHz) elCurrentHz = elTargetHz;
        }
        else if (elCurrentHz > elTargetHz)
        {
            elCurrentHz = (elCurrentHz > elTargetHz + step) ? (elCurrentHz - step) : elTargetHz;
        }
    }
    elRampLastMs = now;
    PWM_SetFreq_EL(elCurrentHz);
    Modbus_SetReg(REG_EL_PWM_FREQ, elCurrentHz);
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

    /* Deliberately do NOT start PWM output here. PWM_SetFreq_AZ/EL() now own
     * starting/stopping the timer (hz>0 starts it, hz==0 stops it) so that
     * REG_AZ_PWM_FREQ/REG_EL_PWM_FREQ always reflect whether the hardware is
     * actually pulsing — the homing/move state machines and the limit-switch
     * interlock all depend on that being true. Starting PWM here regardless
     * of any commanded frequency left the motor pulsing from boot at this
     * stray default rate, invisible to the software (register stayed 0),
     * and unprotected by the limit interlock. */
}

/* --- AZ (connector 2) --- */

void PWM_SetFreq_AZ(uint32_t hz)
{
    /* SetTimerFreq() no-ops on hz==0 (it can't compute a period for 0 Hz) —
     * PWM_Start() runs continuously once started in PWM_Test_Init(), so
     * hz==0 must explicitly stop output here, or "stop" requests from
     * homing/move/abort logic would silently do nothing and pulses would
     * keep going at the last commanded frequency forever. */
    if (hz == 0)
    {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
        return;
    }
    SetTimerFreq(&htim3, TIM_CHANNEL_3, hz);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void PWM_Enable_AZ(uint8_t en)
{
    /* ENABLE is active HIGH on this board (ENA+ to MCU, ENA- to GND) */
    HAL_GPIO_WritePin(ENABLE2_GPIO_Port, ENABLE2_Pin,
        en ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (en) SetStatusBits(REG_AZ_STATUS, AXIS_STATUS_ENABLED, 0);
    else    SetStatusBits(REG_AZ_STATUS, 0, AXIS_STATUS_ENABLED);
}

void PWM_SetDir_AZ(uint8_t dir)
{
    azCurrentDir = dir ? AXIS_DIR_CCW : AXIS_DIR_CW;
    HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin,
        dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* --- EL (connector 1) --- */

void PWM_SetFreq_EL(uint32_t hz)
{
    if (hz == 0)
    {
        HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
        return;
    }
    SetTimerFreq(&htim16, TIM_CHANNEL_1, hz);
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
}

void PWM_Enable_EL(uint8_t en)
{
    /* ENABLE is active HIGH on this board */
    HAL_GPIO_WritePin(ENABLE1_GPIO_Port, ENABLE1_Pin,
        en ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (en) SetStatusBits(REG_EL_STATUS, AXIS_STATUS_ENABLED, 0);
    else    SetStatusBits(REG_EL_STATUS, 0, AXIS_STATUS_ENABLED);
}

void PWM_SetDir_EL(uint8_t dir)
{
    elCurrentDir = dir ? AXIS_DIR_CCW : AXIS_DIR_CW;
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin,
        dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ----------------------------------------------------------------------- */
/* Homing                                                                   */
/* ----------------------------------------------------------------------- */

void AZ_StartHoming(void)
{
    if (!(Modbus_GetReg(REG_AZ_STATUS) & AXIS_STATUS_ENABLED))
    {
        Modbus_SetReg(REG_AZ_ERROR, AXIS_ERR_HOMING_FAILED);
        SetStatusBits(REG_AZ_STATUS, AXIS_STATUS_ERROR, 0);
        return;
    }

    azMoveState = MOVE_IDLE;   /* homing always takes priority over a move */

    /* Always run the full sequence so the final trigger point is repeatable
     * rather than whatever overtravel the previous approach left it at. If
     * already sitting at/into LIMIT1 (e.g. re-homing without moving away
     * first), skip straight to backing off instead of seeking further into
     * an already-triggered switch. */
    float seekRpm = HomeSeekRpm(Modbus_GetRegFloat(REG_AZ_MAX_RPM_HI));
    uint16_t lim  = Modbus_GetReg(REG_LIMIT_SW);
    if (AZ_AtLim1(lim))
    {
        azHomeState = HOME_BACKING_OFF;
        azBackoffReleased = 0;
        PWM_SetDir_AZ(AXIS_DIR_CCW);
    }
    else
    {
        azHomeState = HOME_SEEKING;
        PWM_SetDir_AZ(AXIS_DIR_CW);  /* LIMIT1 is the CW-side switch */
    }
    printf("AZ_HOME_START: initial=%s lim=%s\r\n", HomeStateStr(azHomeState), AZ_LimStr(lim));
    azHomeStartMs = HAL_GetTick();
    azTargetHz    = AZ_RPM_to_Hz(seekRpm);
    azCurrentHz   = 0;             /* ramp from a stop — see AZ_ApplyRamp() */
    azRampLastMs  = HAL_GetTick();

    Modbus_SetReg(REG_AZ_ERROR, AXIS_ERR_NONE);
    SetStatusBits(REG_AZ_STATUS, AXIS_STATUS_HOMING | AXIS_STATUS_MOVING,
                                  AXIS_STATUS_HOMED | AXIS_STATUS_AT_TARGET | AXIS_STATUS_ERROR);
}

void EL_StartHoming(void)
{
    if (!(Modbus_GetReg(REG_EL_STATUS) & AXIS_STATUS_ENABLED))
    {
        Modbus_SetReg(REG_EL_ERROR, AXIS_ERR_HOMING_FAILED);
        SetStatusBits(REG_EL_STATUS, AXIS_STATUS_ERROR, 0);
        return;
    }

    elMoveState = MOVE_IDLE;

    float seekRpm = HomeSeekRpm(Modbus_GetRegFloat(REG_EL_MAX_RPM_HI));
    uint16_t lim  = Modbus_GetReg(REG_LIMIT_SW);
    if (EL_AtLim1(lim))
    {
        elHomeState = HOME_BACKING_OFF;
        elBackoffReleased = 0;
        PWM_SetDir_EL(AXIS_DIR_CCW);
    }
    else
    {
        elHomeState = HOME_SEEKING;
        PWM_SetDir_EL(AXIS_DIR_CW);  /* LIMIT1 is the CW-side switch */
    }
    printf("EL_HOME_START: initial=%s lim=%s\r\n", HomeStateStr(elHomeState), EL_LimStr(lim));
    elHomeStartMs = HAL_GetTick();
    elTargetHz    = EL_RPM_to_Hz(seekRpm);
    elCurrentHz   = 0;             /* ramp from a stop — see EL_ApplyRamp() */
    elRampLastMs  = HAL_GetTick();

    Modbus_SetReg(REG_EL_ERROR, AXIS_ERR_NONE);
    SetStatusBits(REG_EL_STATUS, AXIS_STATUS_HOMING | AXIS_STATUS_MOVING,
                                  AXIS_STATUS_HOMED | AXIS_STATUS_AT_TARGET | AXIS_STATUS_ERROR);
}

static void AZ_HomingTick(void)
{
    if (azHomeState == HOME_IDLE) return;

    if (HAL_GetTick() - azHomeStartMs > AXIS_HOME_TIMEOUT_MS)
    {
        PWM_SetFreq_AZ(0);
        Modbus_SetReg(REG_AZ_PWM_FREQ, 0);
        azCurrentHz = 0;
        PWM_Enable_AZ(0);
        azHomeState = HOME_IDLE;
        Modbus_SetReg(REG_AZ_ERROR, AXIS_ERR_HOMING_FAILED);
        SetStatusBits(REG_AZ_STATUS, AXIS_STATUS_ERROR,
                                      AXIS_STATUS_HOMING | AXIS_STATUS_MOVING | AXIS_STATUS_ENABLED);
        return;
    }

    uint16_t lim  = Modbus_GetReg(REG_LIMIT_SW);
    uint8_t atLim = AZ_AtLim1(lim);

    if (azHomeState == HOME_SEEKING)
    {
        if (atLim)
        {
            azHomeState  = HOME_BACKING_OFF;
            azBackoffReleased = 0;
            PWM_SetDir_AZ(AXIS_DIR_CCW);
            azTargetHz   = AZ_RPM_to_Hz(HomeSeekRpm(Modbus_GetRegFloat(REG_AZ_MAX_RPM_HI)));
            azCurrentHz  = 0;  /* direction changed — restart the ramp */
            azRampLastMs = HAL_GetTick();
            printf("AZ_HOME: SEEKING -> BACKING_OFF lim=%s\r\n", AZ_LimStr(lim));
        }
        else
        {
            AZ_ApplyRamp();
        }
    }
    else if (azHomeState == HOME_BACKING_OFF)
    {
        if (atLim)
        {
            azBackoffReleased = 0;  /* re-triggered (bounce/backlash) — restart the margin */
            AZ_ApplyRamp();
        }
        else if (!azBackoffReleased)
        {
            azBackoffReleased  = 1;
            azBackoffReleaseMs = HAL_GetTick();
            AZ_ApplyRamp();
        }
        else if (HAL_GetTick() - azBackoffReleaseMs < HOME_BACKOFF_EXTRA_MS)
        {
            AZ_ApplyRamp();  /* keep backing off a little past the release point */
        }
        else
        {
            azHomeState  = HOME_APPROACHING;
            PWM_SetDir_AZ(AXIS_DIR_CW);
            azTargetHz   = AZ_RPM_to_Hz(HomeCreepRpm(Modbus_GetRegFloat(REG_AZ_MAX_RPM_HI)));
            azCurrentHz  = 0;  /* direction changed — restart the ramp */
            azRampLastMs = HAL_GetTick();
            printf("AZ_HOME: BACKING_OFF -> APPROACHING lim=%s\r\n", AZ_LimStr(lim));
        }
    }
    else /* HOME_APPROACHING */
    {
        if (atLim)
        {
            PWM_SetFreq_AZ(0);
            Modbus_SetReg(REG_AZ_PWM_FREQ, 0);
            azCurrentHz = 0;
            Encoder_ResetAZ();
            azHomeState = HOME_IDLE;
            Modbus_SetReg(REG_AZ_ERROR, AXIS_ERR_NONE);
            SetStatusBits(REG_AZ_STATUS, AXIS_STATUS_HOMED,
                                          AXIS_STATUS_HOMING | AXIS_STATUS_MOVING);
            printf("AZ_HOME: APPROACHING -> HOMED lim=%s\r\n", AZ_LimStr(lim));
        }
        else
        {
            AZ_ApplyRamp();
        }
    }
}

static void EL_HomingTick(void)
{
    if (elHomeState == HOME_IDLE) return;

    if (HAL_GetTick() - elHomeStartMs > AXIS_HOME_TIMEOUT_MS)
    {
        PWM_SetFreq_EL(0);
        Modbus_SetReg(REG_EL_PWM_FREQ, 0);
        elCurrentHz = 0;
        PWM_Enable_EL(0);
        elHomeState = HOME_IDLE;
        Modbus_SetReg(REG_EL_ERROR, AXIS_ERR_HOMING_FAILED);
        SetStatusBits(REG_EL_STATUS, AXIS_STATUS_ERROR,
                                      AXIS_STATUS_HOMING | AXIS_STATUS_MOVING | AXIS_STATUS_ENABLED);
        return;
    }

    uint16_t lim  = Modbus_GetReg(REG_LIMIT_SW);
    uint8_t atLim = EL_AtLim1(lim);

    if (elHomeState == HOME_SEEKING)
    {
        if (atLim)
        {
            elHomeState  = HOME_BACKING_OFF;
            elBackoffReleased = 0;
            PWM_SetDir_EL(AXIS_DIR_CCW);
            elTargetHz   = EL_RPM_to_Hz(HomeSeekRpm(Modbus_GetRegFloat(REG_EL_MAX_RPM_HI)));
            elCurrentHz  = 0;  /* direction changed — restart the ramp */
            elRampLastMs = HAL_GetTick();
            printf("EL_HOME: SEEKING -> BACKING_OFF lim=%s\r\n", EL_LimStr(lim));
        }
        else
        {
            EL_ApplyRamp();
        }
    }
    else if (elHomeState == HOME_BACKING_OFF)
    {
        if (atLim)
        {
            elBackoffReleased = 0;  /* re-triggered (bounce/backlash) — restart the margin */
            EL_ApplyRamp();
        }
        else if (!elBackoffReleased)
        {
            elBackoffReleased  = 1;
            elBackoffReleaseMs = HAL_GetTick();
            EL_ApplyRamp();
        }
        else if (HAL_GetTick() - elBackoffReleaseMs < HOME_BACKOFF_EXTRA_MS)
        {
            EL_ApplyRamp();  /* keep backing off a little past the release point */
        }
        else
        {
            elHomeState  = HOME_APPROACHING;
            PWM_SetDir_EL(AXIS_DIR_CW);
            elTargetHz   = EL_RPM_to_Hz(HomeCreepRpm(Modbus_GetRegFloat(REG_EL_MAX_RPM_HI)));
            elCurrentHz  = 0;  /* direction changed — restart the ramp */
            elRampLastMs = HAL_GetTick();
            printf("EL_HOME: BACKING_OFF -> APPROACHING lim=%s\r\n", EL_LimStr(lim));
        }
    }
    else /* HOME_APPROACHING */
    {
        if (atLim)
        {
            PWM_SetFreq_EL(0);
            Modbus_SetReg(REG_EL_PWM_FREQ, 0);
            elCurrentHz = 0;
            Encoder_ResetEL();
            elHomeState = HOME_IDLE;
            Modbus_SetReg(REG_EL_ERROR, AXIS_ERR_NONE);
            SetStatusBits(REG_EL_STATUS, AXIS_STATUS_HOMED,
                                          AXIS_STATUS_HOMING | AXIS_STATUS_MOVING);
            printf("EL_HOME: APPROACHING -> HOMED lim=%s\r\n", EL_LimStr(lim));
        }
        else
        {
            EL_ApplyRamp();
        }
    }
}

/* ----------------------------------------------------------------------- */
/* Position moves                                                           */
/* ----------------------------------------------------------------------- */

void AZ_StartMove(void)
{
    if (azHomeState != HOME_IDLE) return;                 /* homing has priority */
    if (Modbus_GetReg(REG_AZ_CMD_MODE) != 0) return;      /* position mode only */
    if (!(Modbus_GetReg(REG_AZ_STATUS) & AXIS_STATUS_ENABLED)) return;

    float   lim1A = Modbus_GetRegFloat(REG_AZ_LIM1_ANGLE_HI);
    float   lim2A = Modbus_GetRegFloat(REG_AZ_LIM2_ANGLE_HI);
    int32_t lim2C = (int32_t)Modbus_GetReg32(REG_AZ_LIM2_POS_HI);
    float   target = Modbus_GetRegFloat(REG_AZ_CMD_POS_HI);

    if (lim2A == lim1A || lim2C == 0)
    {
        Modbus_SetReg(REG_AZ_ERROR, AXIS_ERR_NOT_CALIBRATED);
        SetStatusBits(REG_AZ_STATUS, AXIS_STATUS_ERROR, 0);
        return;
    }

    float countsF = (target - lim1A) * (float)lim2C / (lim2A - lim1A);
    float loF = (lim2C < 0) ? (float)lim2C : 0.0f;
    float hiF = (lim2C < 0) ? 0.0f : (float)lim2C;
    if (countsF < loF || countsF > hiF)
    {
        Modbus_SetReg(REG_AZ_ERROR, AXIS_ERR_POS_RANGE);
        SetStatusBits(REG_AZ_STATUS, AXIS_STATUS_ERROR, 0);
        return;
    }

    azTargetCounts = (int32_t)countsF;

    /* Unidirectional approach: land on every target via the same final CW
     * creep homing uses at LIM1, so drivetrain backlash is taken up
     * identically every time instead of depending on which side the move
     * started from (see MOVE_SEEKING/MOVE_APPROACHING comment above). */
    float   countsPerDeg = (float)lim2C / (lim2A - lim1A);
    int32_t marginCounts = (int32_t)(MOVE_APPROACH_MARGIN_DEG * fabsf(countsPerDeg) + 0.5f);
    if (marginCounts < 1) marginCounts = 1;
    float approachF = countsF - (float)marginCounts;
    if (approachF < loF) approachF = loF;
    azApproachCounts = (int32_t)approachF;

    azMoveState  = MOVE_SEEKING;
    azCurrentHz  = 0;             /* ramp from a stop — see AZ_ApplyRamp() */
    azRampLastMs = HAL_GetTick();
    Modbus_SetReg(REG_AZ_ERROR, AXIS_ERR_NONE);
    SetStatusBits(REG_AZ_STATUS, AXIS_STATUS_MOVING,
                                  AXIS_STATUS_AT_TARGET | AXIS_STATUS_ERROR);
}

void EL_StartMove(void)
{
    if (elHomeState != HOME_IDLE) return;
    if (Modbus_GetReg(REG_EL_CMD_MODE) != 0) return;
    if (!(Modbus_GetReg(REG_EL_STATUS) & AXIS_STATUS_ENABLED)) return;

    float   lim1A = Modbus_GetRegFloat(REG_EL_LIM1_ANGLE_HI);
    float   lim2A = Modbus_GetRegFloat(REG_EL_LIM2_ANGLE_HI);
    int32_t lim2C = (int32_t)Modbus_GetReg32(REG_EL_LIM2_POS_HI);
    float   target = Modbus_GetRegFloat(REG_EL_CMD_POS_HI);

    if (lim2A == lim1A || lim2C == 0)
    {
        Modbus_SetReg(REG_EL_ERROR, AXIS_ERR_NOT_CALIBRATED);
        SetStatusBits(REG_EL_STATUS, AXIS_STATUS_ERROR, 0);
        return;
    }

    float countsF = (target - lim1A) * (float)lim2C / (lim2A - lim1A);
    float loF = (lim2C < 0) ? (float)lim2C : 0.0f;
    float hiF = (lim2C < 0) ? 0.0f : (float)lim2C;
    if (countsF < loF || countsF > hiF)
    {
        Modbus_SetReg(REG_EL_ERROR, AXIS_ERR_POS_RANGE);
        SetStatusBits(REG_EL_STATUS, AXIS_STATUS_ERROR, 0);
        return;
    }

    elTargetCounts = (int32_t)countsF;

    /* Unidirectional approach — see AZ_StartMove for the full rationale. */
    float   countsPerDeg = (float)lim2C / (lim2A - lim1A);
    int32_t marginCounts = (int32_t)(MOVE_APPROACH_MARGIN_DEG * fabsf(countsPerDeg) + 0.5f);
    if (marginCounts < 1) marginCounts = 1;
    float approachF = countsF - (float)marginCounts;
    if (approachF < loF) approachF = loF;
    elApproachCounts = (int32_t)approachF;

    elMoveState  = MOVE_SEEKING;
    elCurrentHz  = 0;             /* ramp from a stop — see EL_ApplyRamp() */
    elRampLastMs = HAL_GetTick();
    Modbus_SetReg(REG_EL_ERROR, AXIS_ERR_NONE);
    SetStatusBits(REG_EL_STATUS, AXIS_STATUS_MOVING,
                                  AXIS_STATUS_AT_TARGET | AXIS_STATUS_ERROR);
}

static void AZ_MoveTick(void)
{
    if (azMoveState == MOVE_IDLE) return;

    float   lim1A = Modbus_GetRegFloat(REG_AZ_LIM1_ANGLE_HI);
    float   lim2A = Modbus_GetRegFloat(REG_AZ_LIM2_ANGLE_HI);
    int32_t lim2C = (int32_t)Modbus_GetReg32(REG_AZ_LIM2_POS_HI);
    float countsPerDeg = (lim2C != 0 && lim2A != lim1A) ?
        (float)lim2C / (lim2A - lim1A) : 0.0f;
    int32_t deadband = (int32_t)(AXIS_POS_TOLERANCE_DEG * fabsf(countsPerDeg) + 0.5f);
    if (deadband < 1) deadband = 1;

    int32_t cur = (int32_t)Modbus_GetReg32(REG_AZ_POS_HI);

    if (azMoveState == MOVE_SEEKING)
    {
        int32_t err = azApproachCounts - cur;
        if ((err < 0 ? -err : err) <= deadband)
        {
            azMoveState  = MOVE_APPROACHING;
            azCurrentHz  = 0;  /* final leg is always CW — restart the ramp */
            azRampLastMs = HAL_GetTick();
        }
        else
        {
            uint8_t newDir = (err > 0) ? AXIS_DIR_CW : AXIS_DIR_CCW;
            if (newDir != azCurrentDir) azCurrentHz = 0;
            PWM_SetDir_AZ(newDir);
            azTargetHz = AZ_RPM_to_Hz(Modbus_GetRegFloat(REG_AZ_MAX_RPM_HI));
            AZ_ApplyRamp();
            SetStatusBits(REG_AZ_STATUS, AXIS_STATUS_MOVING, AXIS_STATUS_AT_TARGET);
        }
    }
    else /* MOVE_APPROACHING */
    {
        int32_t err = azTargetCounts - cur;  /* always positive by design — approach is always from below */
        if (err <= deadband)
        {
            PWM_SetFreq_AZ(0);
            Modbus_SetReg(REG_AZ_PWM_FREQ, 0);
            azCurrentHz = 0;
            azMoveState = MOVE_IDLE;
            SetStatusBits(REG_AZ_STATUS, AXIS_STATUS_AT_TARGET, AXIS_STATUS_MOVING);
        }
        else
        {
            if (azCurrentDir != AXIS_DIR_CW) azCurrentHz = 0;
            PWM_SetDir_AZ(AXIS_DIR_CW);
            azTargetHz = AZ_RPM_to_Hz(HomeCreepRpm(Modbus_GetRegFloat(REG_AZ_MAX_RPM_HI)));
            AZ_ApplyRamp();
            SetStatusBits(REG_AZ_STATUS, AXIS_STATUS_MOVING, AXIS_STATUS_AT_TARGET);
        }
    }
}

static void EL_MoveTick(void)
{
    if (elMoveState == MOVE_IDLE) return;

    float   lim1A = Modbus_GetRegFloat(REG_EL_LIM1_ANGLE_HI);
    float   lim2A = Modbus_GetRegFloat(REG_EL_LIM2_ANGLE_HI);
    int32_t lim2C = (int32_t)Modbus_GetReg32(REG_EL_LIM2_POS_HI);
    float countsPerDeg = (lim2C != 0 && lim2A != lim1A) ?
        (float)lim2C / (lim2A - lim1A) : 0.0f;
    int32_t deadband = (int32_t)(AXIS_POS_TOLERANCE_DEG * fabsf(countsPerDeg) + 0.5f);
    if (deadband < 1) deadband = 1;

    int32_t cur = (int32_t)Modbus_GetReg32(REG_EL_POS_HI);

    if (elMoveState == MOVE_SEEKING)
    {
        int32_t err = elApproachCounts - cur;
        if ((err < 0 ? -err : err) <= deadband)
        {
            elMoveState  = MOVE_APPROACHING;
            elCurrentHz  = 0;  /* final leg is always CW — restart the ramp */
            elRampLastMs = HAL_GetTick();
        }
        else
        {
            uint8_t newDir = (err > 0) ? AXIS_DIR_CW : AXIS_DIR_CCW;
            if (newDir != elCurrentDir) elCurrentHz = 0;
            PWM_SetDir_EL(newDir);
            elTargetHz = EL_RPM_to_Hz(Modbus_GetRegFloat(REG_EL_MAX_RPM_HI));
            EL_ApplyRamp();
            SetStatusBits(REG_EL_STATUS, AXIS_STATUS_MOVING, AXIS_STATUS_AT_TARGET);
        }
    }
    else /* MOVE_APPROACHING */
    {
        int32_t err = elTargetCounts - cur;  /* always positive by design — approach is always from below */
        if (err <= deadband)
        {
            PWM_SetFreq_EL(0);
            Modbus_SetReg(REG_EL_PWM_FREQ, 0);
            elCurrentHz = 0;
            elMoveState = MOVE_IDLE;
            SetStatusBits(REG_EL_STATUS, AXIS_STATUS_AT_TARGET, AXIS_STATUS_MOVING);
        }
        else
        {
            if (elCurrentDir != AXIS_DIR_CW) elCurrentHz = 0;
            PWM_SetDir_EL(AXIS_DIR_CW);
            elTargetHz = EL_RPM_to_Hz(HomeCreepRpm(Modbus_GetRegFloat(REG_EL_MAX_RPM_HI)));
            EL_ApplyRamp();
            SetStatusBits(REG_EL_STATUS, AXIS_STATUS_MOVING, AXIS_STATUS_AT_TARGET);
        }
    }
}

/* ----------------------------------------------------------------------- */
/* Telemetry — fills REG_*_POS_DEG and REG_*_RPM every tick                */
/* ----------------------------------------------------------------------- */

static void AZ_UpdateTelemetry(void)
{
    float   lim1A = Modbus_GetRegFloat(REG_AZ_LIM1_ANGLE_HI);
    float   lim2A = Modbus_GetRegFloat(REG_AZ_LIM2_ANGLE_HI);
    int32_t lim2C = (int32_t)Modbus_GetReg32(REG_AZ_LIM2_POS_HI);

    if (lim2C != 0 && lim2A != lim1A)
    {
        int32_t cur = (int32_t)Modbus_GetReg32(REG_AZ_POS_HI);
        float angle = lim1A + (float)cur * (lim2A - lim1A) / (float)lim2C;
        Modbus_SetRegFloat(REG_AZ_POS_DEG_HI, angle);
    }

    /* Reflects the actual ramped pulse rate (azCurrentHz), not just the
     * eventual target — otherwise this would misleadingly show full speed
     * immediately even while still ramping up. */
    float rpm = 0.0f;
    if (azCurrentHz > 0)
    {
        uint32_t gearRatio = Modbus_GetReg32(REG_AZ_GEAR_RATIO_HI);
        uint16_t ppr       = Modbus_GetReg(REG_AZ_DRIVER_PPR);
        if (gearRatio > 0 && ppr > 0)
            rpm = (float)azCurrentHz * 60.0f / ((float)gearRatio * (float)ppr);
    }
    Modbus_SetRegFloat(REG_AZ_RPM_HI, rpm);
}

static void EL_UpdateTelemetry(void)
{
    float   lim1A = Modbus_GetRegFloat(REG_EL_LIM1_ANGLE_HI);
    float   lim2A = Modbus_GetRegFloat(REG_EL_LIM2_ANGLE_HI);
    int32_t lim2C = (int32_t)Modbus_GetReg32(REG_EL_LIM2_POS_HI);

    if (lim2C != 0 && lim2A != lim1A)
    {
        int32_t cur = (int32_t)Modbus_GetReg32(REG_EL_POS_HI);
        float angle = lim1A + (float)cur * (lim2A - lim1A) / (float)lim2C;
        Modbus_SetRegFloat(REG_EL_POS_DEG_HI, angle);
    }

    /* Reflects the actual ramped pulse rate (elCurrentHz), not just the
     * eventual target — otherwise this would misleadingly show full speed
     * immediately even while still ramping up. */
    float rpm = 0.0f;
    if (elCurrentHz > 0)
    {
        uint32_t gearRatio = Modbus_GetReg32(REG_EL_GEAR_RATIO_HI);
        uint16_t ppr       = Modbus_GetReg(REG_EL_DRIVER_PPR);
        if (gearRatio > 0 && ppr > 0)
            rpm = (float)elCurrentHz * 60.0f / ((float)gearRatio * (float)ppr);
    }
    Modbus_SetRegFloat(REG_EL_RPM_HI, rpm);
}

/* ----------------------------------------------------------------------- */
/* Abort — called from REG_AZ_CMD_STOP / REG_EL_CMD_STOP                   */
/* ----------------------------------------------------------------------- */

void PWM_AbortMotion_AZ(void)
{
    azMoveState = MOVE_IDLE;
    azHomeState = HOME_IDLE;
    PWM_SetFreq_AZ(0);
    Modbus_SetReg(REG_AZ_PWM_FREQ, 0);
    azCurrentHz = 0;
    /* AXIS_STATUS_HOMED is deliberately left untouched — an E-stop doesn't
     * invalidate a prior successful home. */
    SetStatusBits(REG_AZ_STATUS, 0,
                                  AXIS_STATUS_MOVING | AXIS_STATUS_HOMING | AXIS_STATUS_AT_TARGET);
}

void PWM_AbortMotion_EL(void)
{
    elMoveState = MOVE_IDLE;
    elHomeState = HOME_IDLE;
    PWM_SetFreq_EL(0);
    Modbus_SetReg(REG_EL_PWM_FREQ, 0);
    elCurrentHz = 0;
    SetStatusBits(REG_EL_STATUS, 0,
                                  AXIS_STATUS_MOVING | AXIS_STATUS_HOMING | AXIS_STATUS_AT_TARGET);
}

/* ----------------------------------------------------------------------- */
/* Limit-switch direction interlock                                        */
/*                                                                          */
/* Homing and position moves already stop themselves correctly on arrival  */
/* at a limit switch (AZ_HomingTick/AZ_MoveTick above), so this only needs */
/* to guard the case neither of those state machines is managing motion —  */
/* i.e. a raw manual jog (Motor control menu, or the calibration wizard's  */
/* manual jog-to-LIMIT2 step) left running unattended. Without this, a jog */
/* commanded into a limit switch would keep pulsing forever, since the    */
/* driver has no idea the switch tripped.                                 */
/* ----------------------------------------------------------------------- */

static void AZ_LimitInterlockTick(void)
{
    if (azHomeState != HOME_IDLE || azMoveState != MOVE_IDLE) return;
    if (Modbus_GetReg(REG_AZ_PWM_FREQ) == 0) return;

    uint16_t lim = Modbus_GetReg(REG_LIMIT_SW);
    uint8_t blocked = (azCurrentDir == AXIS_DIR_CW  && (lim & AZ_LIM1_BIT)) ||
                      (azCurrentDir == AXIS_DIR_CCW && (lim & AZ_LIM2_BIT));
    if (!blocked) return;

    printf("AZ_INTERLOCK: blocked dir=%s lim=%s homeState=%s moveState=%s\r\n",
           DirStr(azCurrentDir), AZ_LimStr(lim), HomeStateStr(azHomeState), MoveStateStr(azMoveState));

    PWM_SetFreq_AZ(0);
    Modbus_SetReg(REG_AZ_PWM_FREQ, 0);
    azCurrentHz = 0;
    Modbus_SetReg(REG_AZ_ERROR, AXIS_ERR_LIMIT_FAULT);
    SetStatusBits(REG_AZ_STATUS, AXIS_STATUS_ERROR, 0);
}

static void EL_LimitInterlockTick(void)
{
    if (elHomeState != HOME_IDLE || elMoveState != MOVE_IDLE) return;
    if (Modbus_GetReg(REG_EL_PWM_FREQ) == 0) return;

    uint16_t lim = Modbus_GetReg(REG_LIMIT_SW);
    uint8_t blocked = (elCurrentDir == AXIS_DIR_CW  && (lim & EL_LIM1_BIT)) ||
                      (elCurrentDir == AXIS_DIR_CCW && (lim & EL_LIM2_BIT));
    if (!blocked) return;

    printf("EL_INTERLOCK: blocked dir=%s lim=%s homeState=%s moveState=%s\r\n",
           DirStr(elCurrentDir), EL_LimStr(lim), HomeStateStr(elHomeState), MoveStateStr(elMoveState));

    PWM_SetFreq_EL(0);
    Modbus_SetReg(REG_EL_PWM_FREQ, 0);
    elCurrentHz = 0;
    Modbus_SetReg(REG_EL_ERROR, AXIS_ERR_LIMIT_FAULT);
    SetStatusBits(REG_EL_STATUS, AXIS_STATUS_ERROR, 0);
}

void PWM_Test_Run(void)
{
    AZ_LimitInterlockTick();
    AZ_HomingTick();
    AZ_MoveTick();
    AZ_UpdateTelemetry();

    EL_LimitInterlockTick();
    EL_HomingTick();
    EL_MoveTick();
    EL_UpdateTelemetry();
}
