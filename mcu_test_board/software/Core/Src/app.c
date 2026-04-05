/* app.c — MCU Test Board application
 * STM32G431KBT6TR  |  ModBus-RTU over USART1  |  Rev 1.7
 *
 * Architecture:
 *   • TIM2 configured as quadrature encoder interface (PA0=A, PA1=B)
 *   • PA2 EXTI2 rising edge captures encoder index (Z pulse)
 *   • TIM3 CH3 generates PWM on PB0 for motor pulse (PUL)
 *   • TIM6 provides 1 ms system tick for timekeeping
 *   • USART1 interrupt for ModBus-RTU framing
 *   • I2C3 polls SHT45 and ADXL345 every SENSOR_POLL_INTERVAL_MS
 *   • g_regs[] is the single source of truth for all register values
 *
 * Flow:
 *   App_Init()
 *     → read SW1 address pins → store in REG_SYS_ADDR
 *     → initialise g_regs defaults
 *     → start TIM2 encoder, TIM3 PWM (disabled), TIM6
 *     → start USART1 RX interrupt (byte-by-byte)
 *
 *   App_Run()  (called from while(1))
 *     → ModBus_Process()   — build/send response if frame ready
 *     → Encoder_Update()   — sync TIM2 counter into register map
 *     → Sensor_Poll()      — timed I2C reads
 *     All communication is host-initiated polling, no unsolicited TX.
 *
 * Velocity ramp (App_1msTickCallback):
 *   Each 1ms tick Ramp_Tick() steps the PWM frequency (in milli-Hz) toward
 *   the target frequency at MOT_ACCEL_HZ_S or MOT_DECEL_HZ_S Hz/sec.
 *   step_per_tick = rate_hz_s / 1000  (accumulated in milli-Hz to preserve
 *   sub-Hz increments at low rates). Setting rate to 0 = instant.
 *   Ramp always starts from current PWM frequency — no separate start Hz.
 *
 * PWM frequency generation (Motor_ApplyPWM):
 *   TIM3 source = APB1 timer clock = 170 MHz.
 *   PSC and ARR are calculated dynamically so ARR always fits in 16 bits
 *   across the full 1–50000 Hz range, fixing the previous 154 Hz floor
 *   caused by the fixed 10 MHz clock and 16-bit ARR clamp.
 */

#include "app.h"
#include <string.h>

/* =========================================================================
 * Register map
 * ========================================================================= */
volatile uint16_t g_regs[REG_MAP_SIZE];

/* Read-only protection mask: 1 = register rejects writes */
static const uint8_t s_readonly[REG_MAP_SIZE];  /* populated in Regs_Init() */

/* =========================================================================
 * Internal state
 * ========================================================================= */

/* --- ModBus ---------------------------------------------------------------- */
typedef enum {
    MB_STATE_IDLE,
    MB_STATE_RECEIVING,
    MB_STATE_FRAME_READY,
} MbState_t;

/* HAL UART RX target — file-scope so main.c can extern it via app.h */
uint8_t g_mb_rx_byte;

static struct {
    uint8_t   rx_buf[MODBUS_RX_BUF_SIZE];
    uint16_t  rx_len;
    uint8_t   tx_buf[MODBUS_TX_BUF_SIZE];
    uint16_t  tx_len;
    MbState_t state;
    uint32_t  last_rx_tick;   /* ms timestamp of last received byte */
    bool      tx_busy;
} s_mb;

/* --- Encoder --------------------------------------------------------------- */
static struct {
    int32_t  position;       /* Current signed count                    */
    int32_t  last_vel_pos;   /* Position snapshot for velocity calc (ISR) */
    bool     index_seen;
} s_enc;

/* --- Sensors --------------------------------------------------------------- */
static struct {
    uint32_t last_poll_tick;
} s_sensors;

/* --- Timing ---------------------------------------------------------------- */
static volatile uint32_t s_tick_ms = 0;   /* incremented by TIM6 ISR */

/* =========================================================================
 * Forward declarations
 * ========================================================================= */
static void     Regs_Init(void);
static uint8_t  Addr_Read(void);
static uint16_t CRC16(const uint8_t *data, uint16_t len);
static void     ModBus_Process(void);
static void     ModBus_HandleFC03(const uint8_t *frame, uint16_t len);
static void     ModBus_HandleFC06(const uint8_t *frame, uint16_t len);
static void     ModBus_HandleFC10(const uint8_t *frame, uint16_t len);
static void     ModBus_SendException(uint8_t fc, uint8_t ex_code);
static void     ModBus_SendFrame(uint8_t *buf, uint16_t len);
static bool     Reg_IsReadOnly(uint16_t addr);
static void     Reg_Write(uint16_t addr, uint16_t value);
static void     Encoder_Update(void);
static void     Sensor_Poll(void);
static bool     SHT45_Read(void);
static bool     ADXL345_Read(void);
static void     Motor_ApplyEnable(void);
static void     Motor_ApplyTargetRPM(void);
static void     Ramp_Tick(void);
static void     Motor_ApplyDir(void);
static void     Motor_ApplyPWM(void);
static uint32_t Tick(void);

/* =========================================================================
 * App_Init
 * ========================================================================= */
void App_Init(void)
{
    Regs_Init();

    /* Read 3-bit node address from SW1 (active LOW with pull-ups) */
    uint8_t addr = Addr_Read();
    g_regs[REG_SYS_ADDR]   = (addr == 0u) ? 1u : addr;  /* 0 = broadcast, clamp to 1 */
    g_regs[REG_SYS_FW_VER] = FW_VERSION;

    /* TIM2 — quadrature encoder, centre counter so negative travel works */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2, 0x80000000u);  /* centre 32-bit counter */

    /* TIM3 — PWM output, start with 0% duty (motor off) */
    TIM3->CCR3 = 0;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

    /* TIM6 — 1 ms tick interrupt */
    HAL_TIM_Base_Start_IT(&htim6);

    /* Motor outputs — safe state */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);    /* ENABLE high = opto OFF = motor disabled (safe state) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);  /* DIR low = CW     */

    /* LED on during init */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

    /* USART1 RX — single-byte interrupt mode */
    HAL_UART_Receive_IT(&huart1, &g_mb_rx_byte, 1);

    /* Encoder velocity baseline */
    s_enc.last_vel_pos = 0;

    /* LED off — init complete */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
}

/* =========================================================================
 * App_Run — called from main() while(1)
 * ========================================================================= */
void App_Run(void)
{
    ModBus_Process();
    Encoder_Update();
    Sensor_Poll();

    uint32_t t = Tick();
    g_regs[REG_SYS_UPTIME_HI] = (uint16_t)(t >> 16);
    g_regs[REG_SYS_UPTIME_LO] = (uint16_t)(t & 0xFFFFu);
}

/* =========================================================================
 * App_1msTickCallback — call from HAL_TIM_PeriodElapsedCallback (TIM6)
 * ========================================================================= */
void App_1msTickCallback(void)
{
    s_tick_ms++;

    /* Heartbeat: toggle LED every 500 ms */
    if ((s_tick_ms % 500u) == 0u) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
    }

    /* Velocity ramp — step current RPM toward target */
    Ramp_Tick();

    /* Velocity measurement — update every VELOCITY_INTERVAL_MS
     * Done here in the 1ms ISR so it is immune to I2C blocking   */
    static uint32_t s_vel_tick_count = 0u;
    s_vel_tick_count++;
    if (s_vel_tick_count >= VELOCITY_INTERVAL_MS) {
        s_vel_tick_count = 0u;
        /* Read TIM2 counter — 32-bit unsigned, centred at 0x80000000
         * Use unsigned subtraction to handle wrap-around naturally  */
        uint32_t cnt  = TIM2->CNT;
        int32_t  pos  = (int32_t)(cnt - 0x80000000u);
        int32_t  vel  = pos - s_enc.last_vel_pos;
        if (vel >  32767) vel =  32767;
        if (vel < -32768) vel = -32768;
        g_regs[REG_ENC_VELOCITY] = (uint16_t)(int16_t)vel;
        s_enc.last_vel_pos = pos;
    }

    /* ModBus inter-frame gap detection */
    if (s_mb.state == MB_STATE_RECEIVING) {
        if ((s_tick_ms - s_mb.last_rx_tick) >= MODBUS_FRAME_TIMEOUT_MS) {
            if (s_mb.rx_len >= 4u) {
                s_mb.state = MB_STATE_FRAME_READY;
            } else {
                s_mb.rx_len = 0;
                s_mb.state  = MB_STATE_IDLE;
            }
        }
    }
}

/* =========================================================================
 * ModBus_RxByteCallback — call from HAL_UART_RxCpltCallback
 * ========================================================================= */
void ModBus_RxByteCallback(uint8_t byte)
{
    uint32_t now = Tick();

    if (s_mb.state == MB_STATE_FRAME_READY) {
        /* Previous frame not yet consumed — drop byte, re-arm */
        goto restart_rx;
    }

    /* Detect gap mid-frame — start fresh */
    if (s_mb.state == MB_STATE_RECEIVING &&
        (now - s_mb.last_rx_tick) >= MODBUS_FRAME_TIMEOUT_MS) {
        s_mb.rx_len = 0;
    }

    if (s_mb.rx_len < MODBUS_RX_BUF_SIZE) {
        s_mb.rx_buf[s_mb.rx_len++] = byte;
    }
    s_mb.state        = MB_STATE_RECEIVING;
    s_mb.last_rx_tick = now;

restart_rx:
    HAL_UART_Receive_IT(&huart1, &g_mb_rx_byte, 1);
}

/* =========================================================================
 * Encoder_IndexCallback — call from HAL_GPIO_EXTI_Callback (PA2)
 * ========================================================================= */
void Encoder_IndexCallback(void)
{
    s_enc.index_seen = true;
    g_regs[REG_ENC_STATUS] |= ENC_STATUS_INDEX_SEEN;
}

/* =========================================================================
 * Internal: Tick
 * ========================================================================= */
static uint32_t Tick(void)
{
    return s_tick_ms;
}

/* =========================================================================
 * Internal: CRC-16 ModBus (poly 0xA001, init 0xFFFF)
 * ========================================================================= */
static uint16_t CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint8_t)data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x0001u)
                crc = (crc >> 1) ^ 0xA001u;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* =========================================================================
 * Internal: Regs_Init — defaults and read-only protection
 * ========================================================================= */
static void Regs_Init(void)
{
    memset((void *)g_regs, 0, sizeof(g_regs));

    g_regs[REG_ENC_PPR]        = 2000u;
    g_regs[REG_MOT_PWM_FREQ]   = 1000u;
    g_regs[REG_MOT_PWM_DUTY]   = 500u;
    g_regs[REG_MOT_DRIVER_PPR] = MOT_DRIVER_PPR_DEFAULT;
    g_regs[REG_MOT_GEAR_NUM]   = MOT_GEAR_NUM_DEFAULT;
    g_regs[REG_MOT_GEAR_DEN]   = MOT_GEAR_DEN_DEFAULT;
    g_regs[REG_MOT_TARGET_RPM]  = 0u;
    g_regs[REG_MOT_ACCEL_HZ_S]  = MOT_ACCEL_HZ_S_DEFAULT;
    g_regs[REG_MOT_DECEL_HZ_S]  = MOT_DECEL_HZ_S_DEFAULT;
    g_regs[REG_MOT_CURRENT_HZ]  = 0u;

    /* Mark read-only registers */
    uint8_t *ro = (uint8_t *)s_readonly;
    memset(ro, 0, sizeof(s_readonly));
    ro[REG_ENC_POS_HI]    = 1;
    ro[REG_ENC_POS_LO]    = 1;
    ro[REG_ENC_VELOCITY]  = 1;
    ro[REG_ENC_STATUS]    = 1;
    ro[REG_MOT_STATUS]      = 1;
    ro[REG_MOT_ACTUAL_RPM]  = 1;
    ro[REG_MOT_CURRENT_HZ]  = 1;
    ro[REG_SHT45_TEMP]    = 1;
    ro[REG_SHT45_RH]      = 1;
    ro[REG_SHT45_STATUS]  = 1;
    ro[REG_ADXL_X]        = 1;
    ro[REG_ADXL_Y]        = 1;
    ro[REG_ADXL_Z]        = 1;
    ro[REG_ADXL_STATUS]   = 1;
    ro[REG_SYS_ADDR]      = 1;
    ro[REG_SYS_UPTIME_HI] = 1;
    ro[REG_SYS_UPTIME_LO] = 1;
    ro[REG_SYS_FW_VER]    = 1;
}

static bool Reg_IsReadOnly(uint16_t addr)
{
    if (addr >= REG_MAP_SIZE) return false;
    return ((const uint8_t *)s_readonly)[addr] != 0u;
}

/* =========================================================================
 * Internal: Addr_Read — SW1 3-bit address, active LOW
 * ========================================================================= */
static uint8_t Addr_Read(void)
{
    uint8_t a0 = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)  == GPIO_PIN_RESET) ? 1u : 0u;
    uint8_t a1 = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET) ? 1u : 0u;
    uint8_t a2 = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET) ? 1u : 0u;
    return (uint8_t)((a2 << 2) | (a1 << 1) | a0);
}

/* =========================================================================
 * ModBus_Process — handle one complete frame per call
 * ========================================================================= */
static void ModBus_Process(void)
{
    if (s_mb.state != MB_STATE_FRAME_READY) return;
    if (s_mb.tx_busy)                        return;

    uint8_t  *frame = s_mb.rx_buf;
    uint16_t  len   = s_mb.rx_len;

    s_mb.state  = MB_STATE_IDLE;
    s_mb.rx_len = 0;

    if (len < 5u) return;

    /* Address filter */
    uint8_t our_addr = (uint8_t)g_regs[REG_SYS_ADDR];
    if (frame[0] != our_addr && frame[0] != 0u) return;

    /* CRC check */
    uint16_t rx_crc   = ((uint16_t)frame[len-1] << 8) | frame[len-2];
    uint16_t calc_crc = CRC16(frame, len - 2u);
    if (rx_crc != calc_crc) return;

    uint8_t fc = frame[1];
    switch (fc) {
        case FC_READ_HOLDING:  ModBus_HandleFC03(frame, len); break;
        case FC_WRITE_SINGLE:  ModBus_HandleFC06(frame, len); break;
        case FC_WRITE_MULTIPLE: ModBus_HandleFC10(frame, len); break;
        default:
            ModBus_SendException(fc, MB_EX_ILLEGAL_FUNC);
            break;
    }
}

/* -------------------------------------------------------------------------
 * FC 0x03 — Read Holding Registers
 * ------------------------------------------------------------------------- */
static void ModBus_HandleFC03(const uint8_t *frame, uint16_t len)
{
    if (len < 8u) { ModBus_SendException(FC_READ_HOLDING, MB_EX_ILLEGAL_FUNC); return; }

    uint16_t reg = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t qty = ((uint16_t)frame[4] << 8) | frame[5];

    if (qty == 0u || qty > 64u) {
        ModBus_SendException(FC_READ_HOLDING, MB_EX_ILLEGAL_DATA); return;
    }
    if ((uint32_t)(reg + qty) > REG_MAP_SIZE) {
        ModBus_SendException(FC_READ_HOLDING, MB_EX_ILLEGAL_ADDR); return;
    }

    uint8_t  *tx     = s_mb.tx_buf;
    uint16_t  tx_len = 0;
    tx[tx_len++] = (uint8_t)g_regs[REG_SYS_ADDR];
    tx[tx_len++] = FC_READ_HOLDING;
    tx[tx_len++] = (uint8_t)(qty * 2u);

    for (uint16_t i = 0; i < qty; i++) {
        uint16_t val = g_regs[reg + i];
        tx[tx_len++] = (uint8_t)(val >> 8);
        tx[tx_len++] = (uint8_t)(val & 0xFFu);
    }
    ModBus_SendFrame(tx, tx_len);
}

/* -------------------------------------------------------------------------
 * FC 0x06 — Write Single Register
 * ------------------------------------------------------------------------- */
static void ModBus_HandleFC06(const uint8_t *frame, uint16_t len)
{
    if (len < 8u) { ModBus_SendException(FC_WRITE_SINGLE, MB_EX_ILLEGAL_FUNC); return; }

    uint16_t reg = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t val = ((uint16_t)frame[4] << 8) | frame[5];

    if (reg >= REG_MAP_SIZE) {
        ModBus_SendException(FC_WRITE_SINGLE, MB_EX_ILLEGAL_ADDR); return;
    }
    if (Reg_IsReadOnly(reg)) {
        ModBus_SendException(FC_WRITE_SINGLE, MB_EX_ILLEGAL_ADDR); return;
    }

    Reg_Write(reg, val);

    /* Echo: build response from parsed values, not from the raw buffer
     * (the buffer may be overwritten by new RX bytes before we get here) */
    uint8_t *tx     = s_mb.tx_buf;
    uint16_t tx_len = 0;
    tx[tx_len++] = (uint8_t)g_regs[REG_SYS_ADDR];
    tx[tx_len++] = FC_WRITE_SINGLE;
    tx[tx_len++] = (uint8_t)(reg >> 8);
    tx[tx_len++] = (uint8_t)(reg & 0xFFu);
    tx[tx_len++] = (uint8_t)(val >> 8);
    tx[tx_len++] = (uint8_t)(val & 0xFFu);
    ModBus_SendFrame(tx, tx_len);
}

/* -------------------------------------------------------------------------
 * FC 0x10 — Write Multiple Registers
 * ------------------------------------------------------------------------- */
static void ModBus_HandleFC10(const uint8_t *frame, uint16_t len)
{
    if (len < 9u) { ModBus_SendException(FC_WRITE_MULTIPLE, MB_EX_ILLEGAL_FUNC); return; }

    uint16_t reg      = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t qty      = ((uint16_t)frame[4] << 8) | frame[5];
    uint8_t  byte_cnt = frame[6];

    if (qty == 0u || qty > 64u || byte_cnt != qty * 2u) {
        ModBus_SendException(FC_WRITE_MULTIPLE, MB_EX_ILLEGAL_DATA); return;
    }
    if ((uint32_t)(reg + qty) > REG_MAP_SIZE) {
        ModBus_SendException(FC_WRITE_MULTIPLE, MB_EX_ILLEGAL_ADDR); return;
    }
    for (uint16_t i = 0; i < qty; i++) {
        if (Reg_IsReadOnly(reg + i)) {
            ModBus_SendException(FC_WRITE_MULTIPLE, MB_EX_ILLEGAL_ADDR); return;
        }
    }

    for (uint16_t i = 0; i < qty; i++) {
        uint16_t val = ((uint16_t)frame[7u + i*2u] << 8) | frame[8u + i*2u];
        Reg_Write(reg + i, val);
    }

    uint8_t  *tx     = s_mb.tx_buf;
    uint16_t  tx_len = 0;
    tx[tx_len++] = (uint8_t)g_regs[REG_SYS_ADDR];
    tx[tx_len++] = FC_WRITE_MULTIPLE;
    tx[tx_len++] = frame[2];
    tx[tx_len++] = frame[3];
    tx[tx_len++] = frame[4];
    tx[tx_len++] = frame[5];
    ModBus_SendFrame(tx, tx_len);
}

/* =========================================================================
 * Reg_Write — write register and apply hardware side-effects
 * ========================================================================= */
static void Reg_Write(uint16_t addr, uint16_t value)
{
    if (addr >= REG_MAP_SIZE) return;

    if (addr == REG_SYS_RESET && value == SOFT_RESET_MAGIC) {
        NVIC_SystemReset();
        return;
    }

    g_regs[addr] = value;

    switch (addr) {
        case REG_MOT_ENABLE:     Motor_ApplyEnable();    break;
        case REG_MOT_DIR:        Motor_ApplyDir();       break;
        case REG_MOT_PWM_FREQ:
        case REG_MOT_PWM_DUTY:   Motor_ApplyPWM();       break;
        case REG_MOT_TARGET_RPM:
        case REG_MOT_DRIVER_PPR:
        case REG_MOT_GEAR_NUM:
        case REG_MOT_GEAR_DEN:   Motor_ApplyTargetRPM(); break;
        /* Ramp params — picked up by Ramp_Tick each ms tick */
        case REG_MOT_ACCEL_HZ_S:
        case REG_MOT_DECEL_HZ_S:  break;
        default: break;
    }
}

/* =========================================================================
 * ModBus_SendException
 * ========================================================================= */
static void ModBus_SendException(uint8_t fc, uint8_t ex_code)
{
    uint8_t  *tx     = s_mb.tx_buf;
    uint16_t  tx_len = 0;
    tx[tx_len++] = (uint8_t)g_regs[REG_SYS_ADDR];
    tx[tx_len++] = fc | FC_EXCEPTION_MASK;
    tx[tx_len++] = ex_code;
    ModBus_SendFrame(tx, tx_len);
}

/* =========================================================================
 * ModBus_SendFrame — append CRC and transmit via USART1 interrupt
 * ========================================================================= */
static void ModBus_SendFrame(uint8_t *buf, uint16_t len)
{
    uint16_t crc = CRC16(buf, len);
    buf[len++] = (uint8_t)(crc & 0xFFu);
    buf[len++] = (uint8_t)(crc >> 8);
    s_mb.tx_len  = len;
    s_mb.tx_busy = true;
    HAL_UART_Transmit_IT(&huart1, buf, len);
}

/* =========================================================================
 * HAL_UART_TxCpltCallback — clear tx_busy flag
 * Place this in main.c USER CODE BEGIN 4, or call from your existing callback.
 * ========================================================================= */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        s_mb.tx_busy = false;
    }
}

/* =========================================================================
 * Encoder_Update — read TIM2 counter into register map, update velocity
 * ========================================================================= */
static void Encoder_Update(void)
{
    /* TIM2 is 32-bit, centred at 0x80000000 on init */
    uint32_t raw     = __HAL_TIM_GET_COUNTER(&htim2);
    int32_t position = (int32_t)(raw - 0x80000000u);

    s_enc.position = position;

    g_regs[REG_ENC_POS_HI] = (uint16_t)((uint32_t)position >> 16);
    g_regs[REG_ENC_POS_LO] = (uint16_t)((uint32_t)position & 0xFFFFu);

    /* Velocity is updated in App_1msTickCallback() for accuracy —
     * immune to I2C blocking in the main loop.                    */

    /* ENC_STATUS — preserve sticky INDEX_SEEN bit, refresh the rest */
    uint16_t status = g_regs[REG_ENC_STATUS] & ENC_STATUS_INDEX_SEEN;

    /* Limit switches: opto pull-up, active LOW → GPIO reads RESET when triggered */
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET)
        status |= ENC_STATUS_AT_LIMIT1;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET)
        status |= ENC_STATUS_AT_LIMIT2;

    /* Overflow flag: position beyond ±2^30 (very unlikely on this application) */
    if (position > (int32_t)0x3FFFFFFF || position < (int32_t)(-0x40000000))
        status |= ENC_STATUS_OVERFLOW;

    g_regs[REG_ENC_STATUS] = status;

    /* MOT_STATUS */
    uint16_t mot = 0;
    if (g_regs[REG_MOT_ENABLE])                              mot |= MOT_STATUS_ENABLED;
    if (g_regs[REG_MOT_ENABLE] && g_regs[REG_MOT_PWM_DUTY]) mot |= MOT_STATUS_RUNNING;
    if (g_regs[REG_MOT_DIR] == 0u)                          mot |= MOT_STATUS_DIR_CW;
    g_regs[REG_MOT_STATUS] = mot;

    /* MOT_ACTUAL_RPM — derived from encoder velocity
     * velocity is in encoder counts per 100 ms.
     * encoder_ppr (REG_ENC_PPR) counts per motor revolution (×4 for quadrature).
     * motor_rpm = (velocity * 10 * 60) / (REG_ENC_PPR * 4)
     * output_rpm = motor_rpm * gear_den / gear_num               */
    uint16_t enc_ppr  = g_regs[REG_ENC_PPR]  ? g_regs[REG_ENC_PPR]  : 1u;
    uint16_t gear_num = g_regs[REG_MOT_GEAR_NUM] ? g_regs[REG_MOT_GEAR_NUM] : 1u;
    uint16_t gear_den = g_regs[REG_MOT_GEAR_DEN] ? g_regs[REG_MOT_GEAR_DEN] : 1u;
    int16_t  vel      = (int16_t)g_regs[REG_ENC_VELOCITY];
    /* Use absolute velocity; direction comes from MOT_DIR */
    int32_t  vel_abs  = (vel < 0) ? -vel : vel;
    /* motor_rpm = vel_abs [counts/100ms] * 10 [100ms/s] * 60 [s/min]
     *             / (enc_ppr * 4)  [counts/rev, quadrature]        */
    uint32_t motor_rpm_x100 = (uint32_t)vel_abs * 600u / ((uint32_t)enc_ppr * 4u);
    /* output_rpm = motor_rpm * gear_den / gear_num */
    uint32_t output_rpm = motor_rpm_x100 * (uint32_t)gear_den
                          / ((uint32_t)gear_num * 100u);
    if (output_rpm > 0xFFFFu) output_rpm = 0xFFFFu;
    g_regs[REG_MOT_ACTUAL_RPM] = (uint16_t)output_rpm;
}

/* =========================================================================
 * Motor_ApplyEnable
 * ========================================================================= */
static void Motor_ApplyEnable(void)
{
    /* Opto cathode on PA6: GPIO LOW = opto ON = ENA- pulled low = motor enabled.
     * Register 1 = enabled  → GPIO LOW (GPIO_PIN_RESET)
     * Register 0 = disabled → GPIO HIGH (GPIO_PIN_SET)                          */
    GPIO_PinState s = g_regs[REG_MOT_ENABLE] ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, s);
}

/* =========================================================================
 * Motor_ApplyDir
 * ========================================================================= */
static void Motor_ApplyDir(void)
{
    /* DIR=0 → CW (low), DIR=1 → CCW (high) */
    GPIO_PinState s = g_regs[REG_MOT_DIR] ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, s);
}

/* =========================================================================
 * Motor_ApplyPWM — dynamic PSC+ARR calculation
 *
 * TIM3 source clock = APB1 timer clock = 170 MHz (PWM_TIM_SRC_CLK_HZ).
 *
 * The previous implementation used a fixed prescaler (PSC=16, giving
 * 10 MHz TIM clock) and a hard 0xFFFF cap on ARR.  At 100 Hz this
 * requires ARR = 99999, which exceeds 65535, so ARR was clamped to 65535
 * → actual frequency 152 Hz.
 *
 * Fix: calculate PSC so that ARR = (src_clk / ((PSC+1) * freq)) - 1
 * always fits in 16 bits.  Minimum PSC that satisfies this:
 *
 *   PSC_min = ceil(src_clk / (65536 * freq)) - 1
 *           = (src_clk - 1) / (65536 * freq)   [integer ceiling]
 *
 * Then ARR = src_clk / ((PSC+1) * freq) - 1.
 *
 * Frequency range 1–50000 Hz is fully covered with no clamping.
 * ========================================================================= */
static void Motor_ApplyPWM(void)
{
    uint16_t freq = g_regs[REG_MOT_PWM_FREQ];
    uint16_t duty = g_regs[REG_MOT_PWM_DUTY];   /* 0–1000, tenths of % */

    if (freq < PWM_FREQ_MIN_HZ) freq = PWM_FREQ_MIN_HZ;
    if (freq > PWM_FREQ_MAX_HZ) freq = PWM_FREQ_MAX_HZ;
    if (duty > 1000u)           duty = 1000u;

    const uint32_t src = PWM_TIM_SRC_CLK_HZ;

    /* Minimum prescaler so ARR fits in 16 bits */
    uint32_t psc = (src - 1u) / (65536u * (uint32_t)freq);

    /* ARR for the chosen prescaler */
    uint32_t arr = (src / ((psc + 1u) * (uint32_t)freq)) - 1u;

    /* Clamp ARR defensively (should never be needed) */
    if (arr > 0xFFFFu) arr = 0xFFFFu;
    if (arr == 0u)     arr = 1u;

    /* CCR: duty is in tenths of %, so full scale = 1000 */
    uint32_t ccr = ((arr + 1u) * (uint32_t)duty) / 1000u;

    TIM3->PSC  = (uint16_t)psc;
    __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
    TIM3->CCR3 = ccr;

    /* Apply new PSC+ARR without forcing EGR_UG (which resets the counter
     * mid-pulse and causes stuttering during ramps). The new values take
     * effect at the next timer overflow naturally. For an immediate apply
     * only when the timer is idle (CCR3 was 0), use EGR_UG.           */
    if (ccr == 0u) {
        TIM3->EGR = TIM_EGR_UG;
    }
}

/* =========================================================================
 * Ramp_Tick — called every 1 ms from App_1msTickCallback
 *
 * Ramps the PWM frequency directly in milli-Hz toward the target
 * frequency derived from MOT_TARGET_RPM.
 *
 * Accumulator unit: milli-Hz (Hz × 1000)
 * Step per tick   : accel_hz_s  (Hz/sec ÷ 1000ms/sec = Hz/ms,
 *                   but we keep the full accel_hz_s value and accumulate
 *                   into the milli-Hz counter — so 1 tick advances by
 *                   accel_hz_s milli-Hz, reaching accel_hz_s Hz after
 *                   1000 ticks = 1 second. Correct by definition.)
 * ========================================================================= */
static void Ramp_Tick(void)
{
    /* S-curve velocity profile.
     *
     * Accumulators in milli-units to preserve sub-unit precision:
     *   s_current_mhz    — current PWM frequency × 1000  (milli-Hz)
     *   s_current_maccel — current accel rate × 1000      (milli-Hz/sec)
     *
     * Each 1ms tick:
     *   1. Accel rate ramps toward its target by jerk milli-Hz/sec per tick
     *      (jerk is Hz/sec², so adding jerk each tick gives Hz/sec per second)
     *   2. Frequency steps by current_maccel milli-Hz
     *
     * When jerk=0 the accel jumps instantly to max → linear ramp.
     */
    static uint32_t s_current_mhz    = 0u;
    static uint32_t s_current_maccel = 0u;
    static bool     s_initialised    = false;
    static uint32_t s_last_hz        = 0xFFFFFFFFu;

    uint16_t target_rpm = g_regs[REG_MOT_TARGET_RPM];
    uint16_t max_accel  = g_regs[REG_MOT_ACCEL_HZ_S];
    uint16_t max_decel  = g_regs[REG_MOT_DECEL_HZ_S];
    uint16_t jerk       = g_regs[REG_MOT_JERK_HZ_S2];

    /* Seed from current PWM on first call */
    if (!s_initialised) {
        s_current_mhz    = (uint32_t)g_regs[REG_MOT_PWM_FREQ] * 1000u;
        s_current_maccel = 0u;
        s_initialised    = true;
    }

    /* Calculate target PWM in milli-Hz */
    uint16_t driver_ppr = g_regs[REG_MOT_DRIVER_PPR];
    uint16_t gear_num   = g_regs[REG_MOT_GEAR_NUM];
    uint16_t gear_den   = g_regs[REG_MOT_GEAR_DEN];
    if (driver_ppr == 0u) driver_ppr = MOT_DRIVER_PPR_DEFAULT;
    if (gear_num   == 0u) gear_num   = 1u;
    if (gear_den   == 0u) gear_den   = 1u;

    uint32_t motor_rpm  = (uint32_t)target_rpm * gear_num / gear_den;
    uint32_t target_hz  = motor_rpm * driver_ppr / 60u;
    if (target_hz > PWM_FREQ_MAX_HZ) target_hz = PWM_FREQ_MAX_HZ;
    uint32_t target_mhz = target_hz * 1000u;

    if (s_current_mhz == target_mhz && s_current_maccel == 0u) return;

    if (s_current_mhz < target_mhz) {
        /* ---- Accelerating -------------------------------------------- */
        uint32_t max_maccel = (uint32_t)max_accel * 1000u;
        if (max_maccel == 0u) {
            /* Instant */
            s_current_mhz    = target_mhz;
            s_current_maccel = 0u;
        } else {
            /* S-curve: taper accel down when close to target.
             * Distance to stop building accel = accel² / (2 × jerk)     */
            uint32_t remaining   = target_mhz - s_current_mhz;
            uint32_t taper_dist  = (jerk > 0u && s_current_maccel > 0u)
                ? (s_current_maccel / 1000u) * (s_current_maccel / 1000u)
                  / (2u * (uint32_t)jerk)
                  * 1000u
                : 0u;

            if (remaining <= taper_dist || s_current_maccel >= max_maccel) {
                /* Taper down */
                uint32_t step = (jerk == 0u) ? s_current_maccel : (uint32_t)jerk * 1000u;
                s_current_maccel = (step >= s_current_maccel) ? 0u
                                                              : s_current_maccel - step;
            } else {
                /* Build up */
                uint32_t step = (jerk == 0u) ? max_maccel : (uint32_t)jerk * 1000u;
                s_current_maccel += step;
                if (s_current_maccel > max_maccel) s_current_maccel = max_maccel;
            }
            s_current_mhz += s_current_maccel;
            if (s_current_mhz >= target_mhz) {
                s_current_mhz    = target_mhz;
                s_current_maccel = 0u;
            }
        }
    } else if (s_current_mhz > target_mhz) {
        /* ---- Decelerating -------------------------------------------- */
        uint32_t max_mdecel = (uint32_t)max_decel * 1000u;
        if (max_mdecel == 0u) {
            s_current_mhz    = target_mhz;
            s_current_maccel = 0u;
        } else {
            uint32_t remaining  = s_current_mhz - target_mhz;
            uint32_t taper_dist = (jerk > 0u && s_current_maccel > 0u)
                ? (s_current_maccel / 1000u) * (s_current_maccel / 1000u)
                  / (2u * (uint32_t)jerk)
                  * 1000u
                : 0u;

            if (remaining <= taper_dist || s_current_maccel >= max_mdecel) {
                /* Taper down */
                uint32_t step = (jerk == 0u) ? s_current_maccel : (uint32_t)jerk * 1000u;
                s_current_maccel = (step >= s_current_maccel) ? 0u
                                                              : s_current_maccel - step;
            } else {
                /* Build up */
                uint32_t step = (jerk == 0u) ? max_mdecel : (uint32_t)jerk * 1000u;
                s_current_maccel += step;
                if (s_current_maccel > max_mdecel) s_current_maccel = max_mdecel;
            }
            if (s_current_maccel >= s_current_mhz - target_mhz) {
                s_current_mhz    = target_mhz;
                s_current_maccel = 0u;
            } else {
                s_current_mhz -= s_current_maccel;
            }
        }
    } else {
        s_current_maccel = 0u;
    }

    /* Convert to whole Hz */
    uint32_t current_hz = s_current_mhz / 1000u;
    if (current_hz > PWM_FREQ_MAX_HZ) current_hz = PWM_FREQ_MAX_HZ;
    g_regs[REG_MOT_CURRENT_HZ] = (uint16_t)current_hz;

    /* Only update hardware when Hz changes */
    if (current_hz == s_last_hz) return;
    s_last_hz = current_hz;

    if (s_current_mhz == 0u) {
        TIM3->CCR3 = 0u;
        g_regs[REG_MOT_PWM_FREQ] = 0u;
    } else {
        g_regs[REG_MOT_PWM_FREQ] = (uint16_t)current_hz;
        if (g_regs[REG_MOT_PWM_DUTY] == 0u) g_regs[REG_MOT_PWM_DUTY] = 500u;
        Motor_ApplyPWM();
    }
}
/* =========================================================================
 * Motor_ApplyTargetRPM — convert desired output shaft RPM to PWM frequency
 *
 * Called when MOT_TARGET_RPM, MOT_DRIVER_PPR, MOT_GEAR_NUM or MOT_GEAR_DEN
 * is written. Also writes the result into REG_MOT_PWM_FREQ so the host can
 * read back the actual frequency that was applied.
 *
 * Formula:
 *   motor_rpm  = target_output_rpm * gear_num / gear_den
 *   pwm_hz     = motor_rpm / 60 * driver_ppr
 *
 * If target_rpm == 0, PWM is set to 0 Hz (duty also zeroed — motor stops).
 * ========================================================================= */
static void Motor_ApplyTargetRPM(void)
{
    uint16_t target_rpm = g_regs[REG_MOT_TARGET_RPM];
    uint16_t driver_ppr = g_regs[REG_MOT_DRIVER_PPR];
    uint16_t gear_num   = g_regs[REG_MOT_GEAR_NUM];
    uint16_t gear_den   = g_regs[REG_MOT_GEAR_DEN];

    /* Guard against divide-by-zero */
    if (driver_ppr == 0u) driver_ppr = MOT_DRIVER_PPR_DEFAULT;
    if (gear_num   == 0u) gear_num   = 1u;
    if (gear_den   == 0u) gear_den   = 1u;

    if (target_rpm == 0u) {
        /* Stop: zero duty, frequency unchanged */
        g_regs[REG_MOT_PWM_DUTY] = 0u;
        TIM3->CCR3 = 0u;
        return;
    }

    /* motor_rpm = target_output_rpm * gear_num / gear_den
     * Use 32-bit arithmetic to avoid overflow on large values.   */
    uint32_t motor_rpm = (uint32_t)target_rpm
                         * (uint32_t)gear_num
                         / (uint32_t)gear_den;

    /* pwm_hz = motor_rpm * driver_ppr / 60 */
    uint32_t freq_hz = motor_rpm * (uint32_t)driver_ppr / 60u;

    /* Clamp to supported range */
    if (freq_hz < PWM_FREQ_MIN_HZ) freq_hz = PWM_FREQ_MIN_HZ;
    if (freq_hz > PWM_FREQ_MAX_HZ) freq_hz = PWM_FREQ_MAX_HZ;

    /* Store back so host can read the actual frequency applied */
    g_regs[REG_MOT_PWM_FREQ] = (uint16_t)freq_hz;

    /* In instant mode (both rates 0) sync CURRENT_HZ to actual freq immediately */
    if (g_regs[REG_MOT_ACCEL_HZ_S] == 0u && g_regs[REG_MOT_DECEL_HZ_S] == 0u) {
        g_regs[REG_MOT_CURRENT_HZ] = (uint16_t)freq_hz;
    }

    /* Ensure duty is at 50% if it was zeroed */
    if (g_regs[REG_MOT_PWM_DUTY] == 0u) {
        g_regs[REG_MOT_PWM_DUTY] = 500u;
    }

    Motor_ApplyPWM();
}

/* =========================================================================
 * Sensor_Poll — timed I2C reads
 * ========================================================================= */
static void Sensor_Poll(void)
{
    uint32_t now = Tick();
    if ((now - s_sensors.last_poll_tick) < SENSOR_POLL_INTERVAL_MS) return;
    s_sensors.last_poll_tick = now;

    SHT45_Read();
    ADXL345_Read();
}

/* =========================================================================
 * SHT45 — temperature + humidity
 * Command 0xFD: high-precision measurement (~8 ms)
 * Response: [T_MSB][T_LSB][CRC][RH_MSB][RH_LSB][CRC]
 * ========================================================================= */
static uint8_t sht45_crc(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFFu;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x31u) : (uint8_t)(crc << 1);
    }
    return crc;
}

static bool SHT45_Read(void)
{
    uint8_t cmd = 0xFDu;
    HAL_StatusTypeDef st;

    st = HAL_I2C_Master_Transmit(&hi2c3, SHT45_I2C_ADDR, &cmd, 1, 10);
    if (st != HAL_OK) { g_regs[REG_SHT45_STATUS] = SENSOR_STATUS_TIMEOUT; return false; }

    /* Non-blocking delay: return now and re-enter after 10ms     */
    static uint32_t s_sht45_cmd_tick = 0u;
    static bool     s_sht45_cmd_sent = false;
    if (!s_sht45_cmd_sent) {
        s_sht45_cmd_sent = true;
        s_sht45_cmd_tick = Tick();
        return false;  /* come back next poll cycle */
    }
    if ((Tick() - s_sht45_cmd_tick) < 12u) {
        return false;  /* measurement not ready yet */
    }
    s_sht45_cmd_sent = false;  /* reset for next reading */

    uint8_t buf[6];
    st = HAL_I2C_Master_Receive(&hi2c3, SHT45_I2C_ADDR, buf, 6, 10);
    if (st != HAL_OK) { g_regs[REG_SHT45_STATUS] = SENSOR_STATUS_TIMEOUT; return false; }

    if (sht45_crc(buf, 2u) != buf[2] || sht45_crc(buf + 3u, 2u) != buf[5]) {
        g_regs[REG_SHT45_STATUS] = SENSOR_STATUS_CRC_ERR; return false;
    }

    uint16_t t_raw  = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t rh_raw = ((uint16_t)buf[3] << 8) | buf[4];

    /* T[°C]  = -45 + 175 * raw/65535  → stored as 0.01 °C, signed 16-bit */
    /* RH[%]  = -6  + 125 * raw/65535  → stored as 0.01 %, unsigned 16-bit */
    int32_t temp_cdeg = -4500  + (int32_t)(17500u * (uint32_t)t_raw  / 65535u);
    int32_t rh_cpct   = -600   + (int32_t)(12500u * (uint32_t)rh_raw / 65535u);

    if (temp_cdeg < -4000) temp_cdeg = -4000;
    if (temp_cdeg >  8500) temp_cdeg =  8500;
    if (rh_cpct   <     0) rh_cpct   = 0;
    if (rh_cpct   > 10000) rh_cpct   = 10000;

    g_regs[REG_SHT45_TEMP]   = (uint16_t)(int16_t)temp_cdeg;
    g_regs[REG_SHT45_RH]     = (uint16_t)rh_cpct;
    g_regs[REG_SHT45_STATUS] = SENSOR_STATUS_OK;
    return true;
}

/* =========================================================================
 * ADXL345 — triple-axis accelerometer
 * ========================================================================= */
#define ADXL345_REG_POWER_CTL   0x2Du
#define ADXL345_REG_DATA_FORMAT 0x31u
#define ADXL345_REG_DATAX0      0x32u

static bool s_adxl_init_done = false;

static bool ADXL345_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return HAL_I2C_Master_Transmit(&hi2c3, ADXL345_I2C_ADDR, buf, 2, 10) == HAL_OK;
}

static bool ADXL345_Read(void)
{
    /* One-time init: full resolution, ±2g, measure mode */
    if (!s_adxl_init_done) {
        if (!ADXL345_WriteReg(ADXL345_REG_DATA_FORMAT, 0x08u)) {
            g_regs[REG_ADXL_STATUS] = SENSOR_STATUS_TIMEOUT; return false;
        }
        if (!ADXL345_WriteReg(ADXL345_REG_POWER_CTL, 0x08u)) {
            g_regs[REG_ADXL_STATUS] = SENSOR_STATUS_TIMEOUT; return false;
        }
        s_adxl_init_done = true;
    }

    /* Burst read 6 bytes from DATAX0 (0x80 bit = auto-increment) */
    uint8_t reg = ADXL345_REG_DATAX0 | 0x80u;
    HAL_StatusTypeDef st;

    st = HAL_I2C_Master_Transmit(&hi2c3, ADXL345_I2C_ADDR, &reg, 1, 10);
    if (st != HAL_OK) { g_regs[REG_ADXL_STATUS] = SENSOR_STATUS_TIMEOUT; return false; }

    uint8_t buf[6];
    st = HAL_I2C_Master_Receive(&hi2c3, ADXL345_I2C_ADDR, buf, 6, 10);
    if (st != HAL_OK) { g_regs[REG_ADXL_STATUS] = SENSOR_STATUS_TIMEOUT; return false; }

    /* Little-endian signed 16-bit output */
    int16_t x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    int16_t y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    int16_t z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));

    g_regs[REG_ADXL_X]      = (uint16_t)x;
    g_regs[REG_ADXL_Y]      = (uint16_t)y;
    g_regs[REG_ADXL_Z]      = (uint16_t)z;
    g_regs[REG_ADXL_STATUS] = SENSOR_STATUS_OK;
    return true;
}
