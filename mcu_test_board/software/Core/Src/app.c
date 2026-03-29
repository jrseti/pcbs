/* app.c — MCU Test Board application
 * STM32G431KBT6TR  |  ModBus-RTU over USART1  |  Rev 1.1
 *
 * Architecture:
 *   • TIM2 configured as quadrature encoder interface (PA0=A, PA1=B)
 *   • PA2 EXTI2 rising edge captures encoder index (Z pulse)
 *   • TIM3 CH3 generates PWM on PB0 for motor pulse (PUL)
 *   • TIM6 provides 1 ms system tick for timekeeping
 *   • USART1 DMA/interrupt for ModBus-RTU framing
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
 *     → ModBus_Process()      — build/send response if frame ready
 *     → Encoder_Update()      — sync TIM2 counter into register map
 *     → Sensor_Poll()         — timed I2C reads
 *     → PushNotify_Check()    — send FC=0x41 if position changed enough
 */

#include "app.h"
#include <string.h>

/* =========================================================================
 * Register map
 * ========================================================================= */
volatile uint16_t g_regs[REG_MAP_SIZE];

/* Read-only register protection mask —
 * 1 = register is read-only (write attempts return exception 0x02)
 */
static const uint8_t s_readonly[REG_MAP_SIZE];  /* Filled in App_Init */

/* =========================================================================
 * Internal state
 * ========================================================================= */

/* --- ModBus ---------------------------------------------------------------- */
typedef enum {
    MB_STATE_IDLE,
    MB_STATE_RECEIVING,
    MB_STATE_FRAME_READY,
} MbState_t;

/* HAL UART RX target — one byte at a time, extern'd via app.h for main.c callback */
uint8_t g_mb_rx_byte;

static struct {
    uint8_t    rx_buf[MODBUS_RX_BUF_SIZE];
    uint16_t   rx_len;
    uint8_t    tx_buf[MODBUS_TX_BUF_SIZE];
    uint16_t   tx_len;
    MbState_t  state;
    uint32_t   last_rx_tick;   /* ms timestamp of last received byte */
    bool       tx_busy;
} s_mb;

/* --- Encoder --------------------------------------------------------------- */
static struct {
    int32_t  position;         /* Current signed count */
    int32_t  last_push_pos;    /* Position at last push notification */
    int32_t  last_vel_pos;     /* Position snapshot for velocity calc */
    uint32_t last_vel_tick;    /* ms timestamp of last velocity calc */
    bool     index_seen;
} s_enc;

/* --- Sensors --------------------------------------------------------------- */
static struct {
    uint32_t last_poll_tick;
} s_sensors;

/* --- Timing ---------------------------------------------------------------- */
static volatile uint32_t s_tick_ms = 0;   /* Incremented by TIM6 ISR */

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
static void     PushNotify_Check(void);
static void     Sensor_Poll(void);
static bool     SHT45_Read(void);
static bool     ADXL345_Read(void);
static void     Motor_ApplyEnable(void);
static void     Motor_ApplyDir(void);
static void     Motor_ApplyPWM(void);
static uint32_t Tick(void);

/* =========================================================================
 * App_Init
 * ========================================================================= */
void App_Init(void)
{
    /* Initialise register defaults */
    Regs_Init();

    /* Read 3-bit node address from SW1 (active LOW with pull-ups) */
    uint8_t addr = Addr_Read();
    g_regs[REG_SYS_ADDR]    = (addr == 0) ? 1 : addr;  /* Clamp: 0 is broadcast */
    g_regs[REG_SYS_FW_VER]  = FW_VERSION;

    /* Encoder: start TIM2 in encoder mode */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2, 0x8000);  /* Centre counter to allow ± travel */

    /* PWM: TIM3 CH3 — start with 0% duty (motor off) */
    TIM3->CCR3 = 0;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

    /* System tick: TIM6 generates interrupt every 1 ms */
    HAL_TIM_Base_Start_IT(&htim6);

    /* Motor outputs — safe state */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);  /* ENABLE = low = off */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);  /* DIR = low = CW     */

    /* LED */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);   /* LED on during init */

    /* USART1: enable RX interrupt, byte by byte */
    HAL_UART_Receive_IT(&huart1, &g_mb_rx_byte, 1);

    /* Initialise encoder snapshot */
    s_enc.last_push_pos  = 0;
    s_enc.last_vel_pos   = 0;
    s_enc.last_vel_tick  = 0;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);  /* LED off — init done */
}

/* =========================================================================
 * App_Run  — called from main() while(1)
 * ========================================================================= */
void App_Run(void)
{
    ModBus_Process();
    Encoder_Update();
    Sensor_Poll();
    PushNotify_Check();

    /* Update uptime */
    uint32_t t = Tick();
    g_regs[REG_SYS_UPTIME_HI] = (uint16_t)(t >> 16);
    g_regs[REG_SYS_UPTIME_LO] = (uint16_t)(t & 0xFFFF);
}

/* =========================================================================
 * App_1msTickCallback — called from HAL_TIM_PeriodElapsedCallback for TIM6
 * ========================================================================= */
void App_1msTickCallback(void)
{
    s_tick_ms++;
    /* Detect inter-frame gap: if bytes were coming and stopped, mark ready */
    if (s_mb.state == MB_STATE_RECEIVING) {
        if ((s_tick_ms - s_mb.last_rx_tick) >= MODBUS_FRAME_TIMEOUT_MS) {
            if (s_mb.rx_len >= 4) {
                s_mb.state = MB_STATE_FRAME_READY;
            } else {
                /* Too short — discard */
                s_mb.rx_len = 0;
                s_mb.state  = MB_STATE_IDLE;
            }
        }
    }
}

/* =========================================================================
 * ModBus_RxByteCallback — called from HAL_UART_RxCpltCallback
 * ========================================================================= */
void ModBus_RxByteCallback(uint8_t byte)
{
    uint32_t now = Tick();

    if (s_mb.state == MB_STATE_FRAME_READY) {
        /* Still processing previous frame — drop incoming byte */
        goto restart_rx;
    }

    /* Inter-frame gap detection */
    if (s_mb.state == MB_STATE_RECEIVING &&
        (now - s_mb.last_rx_tick) >= MODBUS_FRAME_TIMEOUT_MS) {
        /* Gap detected mid-accumulation — start fresh */
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
 * Encoder_IndexCallback — called from HAL_GPIO_EXTI_Callback for PA2
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
 * Internal: CRC-16 ModBus
 * ========================================================================= */
static uint16_t CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint8_t)data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* =========================================================================
 * Internal: Regs_Init — set defaults, mark read-only registers
 * ========================================================================= */
static void Regs_Init(void)
{
    memset((void *)g_regs, 0, sizeof(g_regs));

    /* Defaults */
    g_regs[REG_ENC_PPR]        = 2000;
    g_regs[REG_ENC_PUSH_EN]    = 1;
    g_regs[REG_ENC_PUSH_THRESH]= 1;
    g_regs[REG_MOT_PWM_FREQ]   = 1000;
    g_regs[REG_MOT_PWM_DUTY]   = 500;

    /* Mark read-only (cast away volatile for init only) */
    uint8_t *ro = (uint8_t *)s_readonly;
    memset(ro, 0, sizeof(s_readonly));
    ro[REG_ENC_POS_HI]   = 1;
    ro[REG_ENC_POS_LO]   = 1;
    ro[REG_ENC_VELOCITY] = 1;
    ro[REG_ENC_STATUS]   = 1;
    ro[REG_MOT_STATUS]   = 1;
    ro[REG_SHT45_TEMP]   = 1;
    ro[REG_SHT45_RH]     = 1;
    ro[REG_SHT45_STATUS] = 1;
    ro[REG_ADXL_X]       = 1;
    ro[REG_ADXL_Y]       = 1;
    ro[REG_ADXL_Z]       = 1;
    ro[REG_ADXL_STATUS]  = 1;
    ro[REG_SYS_ADDR]     = 1;
    ro[REG_SYS_UPTIME_HI]= 1;
    ro[REG_SYS_UPTIME_LO]= 1;
    ro[REG_SYS_FW_VER]   = 1;
}

static bool Reg_IsReadOnly(uint16_t addr)
{
    if (addr >= REG_MAP_SIZE) return false;
    return ((uint8_t *)s_readonly)[addr] != 0;
}

/* =========================================================================
 * Internal: Addr_Read — read SW1 3-bit address (active LOW)
 * ========================================================================= */
static uint8_t Addr_Read(void)
{
    uint8_t a0 = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)  == GPIO_PIN_RESET) ? 1u : 0u;
    uint8_t a1 = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET) ? 1u : 0u;
    uint8_t a2 = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET) ? 1u : 0u;
    return (uint8_t)((a2 << 2) | (a1 << 1) | a0);
}

/* =========================================================================
 * ModBus_Process — called from App_Run, handles one complete frame per call
 * ========================================================================= */
static void ModBus_Process(void)
{
    if (s_mb.state != MB_STATE_FRAME_READY) return;
    if (s_mb.tx_busy)                        return;  /* TX still ongoing */

    uint8_t  *frame = s_mb.rx_buf;
    uint16_t  len   = s_mb.rx_len;

    /* Reset for next frame */
    s_mb.state  = MB_STATE_IDLE;
    s_mb.rx_len = 0;

    /* Minimum frame: addr(1) + fc(1) + data(≥1) + crc(2) = 5 bytes */
    if (len < 5) return;

    /* Address filter: accept our address or broadcast (0) */
    uint8_t our_addr = (uint8_t)g_regs[REG_SYS_ADDR];
    if (frame[0] != our_addr && frame[0] != 0) return;

    /* CRC check */
    uint16_t rx_crc = (uint16_t)(frame[len-1] << 8) | frame[len-2];
    uint16_t calc_crc = CRC16(frame, len - 2);
    if (rx_crc != calc_crc) return;   /* Silently discard bad CRC */

    /* Dispatch by function code */
    uint8_t fc = frame[1];
    switch (fc) {
        case FC_READ_HOLDING: ModBus_HandleFC03(frame, len); break;
        case FC_WRITE_SINGLE: ModBus_HandleFC06(frame, len); break;
        case FC_WRITE_MULTIPLE: ModBus_HandleFC10(frame, len); break;
        default:
            ModBus_SendException(fc, MB_EX_ILLEGAL_FUNC);
            break;
    }
}

/* -------------------------------------------------------------------------
 * FC 0x03 — Read Holding Registers
 * Request:  [addr][03][reg_hi][reg_lo][qty_hi][qty_lo][crc×2]  = 8 bytes
 * Response: [addr][03][byte_cnt][data…][crc×2]
 * ------------------------------------------------------------------------- */
static void ModBus_HandleFC03(const uint8_t *frame, uint16_t len)
{
    if (len < 8) { ModBus_SendException(FC_READ_HOLDING, MB_EX_ILLEGAL_FUNC); return; }

    uint16_t reg = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t qty = ((uint16_t)frame[4] << 8) | frame[5];

    if (qty == 0 || qty > 64) {
        ModBus_SendException(FC_READ_HOLDING, MB_EX_ILLEGAL_DATA); return;
    }
    if ((uint32_t)(reg + qty) > REG_MAP_SIZE) {
        ModBus_SendException(FC_READ_HOLDING, MB_EX_ILLEGAL_ADDR); return;
    }

    uint8_t *tx = s_mb.tx_buf;
    uint16_t tx_len = 0;
    tx[tx_len++] = (uint8_t)g_regs[REG_SYS_ADDR];
    tx[tx_len++] = FC_READ_HOLDING;
    tx[tx_len++] = (uint8_t)(qty * 2);

    for (uint16_t i = 0; i < qty; i++) {
        uint16_t val = g_regs[reg + i];
        tx[tx_len++] = (uint8_t)(val >> 8);
        tx[tx_len++] = (uint8_t)(val & 0xFF);
    }

    ModBus_SendFrame(tx, tx_len);
}

/* -------------------------------------------------------------------------
 * FC 0x06 — Write Single Register
 * Request:  [addr][06][reg_hi][reg_lo][val_hi][val_lo][crc×2]  = 8 bytes
 * Response: Echo of request
 * ------------------------------------------------------------------------- */
static void ModBus_HandleFC06(const uint8_t *frame, uint16_t len)
{
    if (len < 8) { ModBus_SendException(FC_WRITE_SINGLE, MB_EX_ILLEGAL_FUNC); return; }

    uint16_t reg = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t val = ((uint16_t)frame[4] << 8) | frame[5];

    if (reg >= REG_MAP_SIZE) {
        ModBus_SendException(FC_WRITE_SINGLE, MB_EX_ILLEGAL_ADDR); return;
    }
    if (Reg_IsReadOnly(reg)) {
        ModBus_SendException(FC_WRITE_SINGLE, MB_EX_ILLEGAL_ADDR); return;
    }

    Reg_Write(reg, val);

    /* Echo */
    uint8_t *tx = s_mb.tx_buf;
    memcpy(tx, frame, len);
    ModBus_SendFrame(tx, len - 2);  /* Re-append CRC via SendFrame */
}

/* -------------------------------------------------------------------------
 * FC 0x10 — Write Multiple Registers
 * Request:  [addr][10][reg_hi][reg_lo][qty_hi][qty_lo][byte_cnt][data…][crc×2]
 * Response: [addr][10][reg_hi][reg_lo][qty_hi][qty_lo][crc×2]
 * ------------------------------------------------------------------------- */
static void ModBus_HandleFC10(const uint8_t *frame, uint16_t len)
{
    if (len < 9) { ModBus_SendException(FC_WRITE_MULTIPLE, MB_EX_ILLEGAL_FUNC); return; }

    uint16_t reg      = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t qty      = ((uint16_t)frame[4] << 8) | frame[5];
    uint8_t  byte_cnt = frame[6];

    if (qty == 0 || qty > 64 || byte_cnt != qty * 2) {
        ModBus_SendException(FC_WRITE_MULTIPLE, MB_EX_ILLEGAL_DATA); return;
    }
    if ((uint32_t)(reg + qty) > REG_MAP_SIZE) {
        ModBus_SendException(FC_WRITE_MULTIPLE, MB_EX_ILLEGAL_ADDR); return;
    }
    /* Check for any read-only registers in the range */
    for (uint16_t i = 0; i < qty; i++) {
        if (Reg_IsReadOnly(reg + i)) {
            ModBus_SendException(FC_WRITE_MULTIPLE, MB_EX_ILLEGAL_ADDR); return;
        }
    }

    for (uint16_t i = 0; i < qty; i++) {
        uint16_t val = ((uint16_t)frame[7 + i*2] << 8) | frame[8 + i*2];
        Reg_Write(reg + i, val);
    }

    /* Response */
    uint8_t *tx = s_mb.tx_buf;
    uint16_t tx_len = 0;
    tx[tx_len++] = (uint8_t)g_regs[REG_SYS_ADDR];
    tx[tx_len++] = FC_WRITE_MULTIPLE;
    tx[tx_len++] = frame[2];
    tx[tx_len++] = frame[3];
    tx[tx_len++] = frame[4];
    tx[tx_len++] = frame[5];
    ModBus_SendFrame(tx, tx_len);
}

/* =========================================================================
 * Reg_Write — write a register and apply side effects
 * ========================================================================= */
static void Reg_Write(uint16_t addr, uint16_t value)
{
    if (addr >= REG_MAP_SIZE) return;

    /* Special: soft reset */
    if (addr == REG_SYS_RESET && value == SOFT_RESET_MAGIC) {
        NVIC_SystemReset();
        return;
    }

    g_regs[addr] = value;

    /* Apply side effects */
    switch (addr) {
        case REG_MOT_ENABLE:  Motor_ApplyEnable(); break;
        case REG_MOT_DIR:     Motor_ApplyDir();    break;
        case REG_MOT_PWM_FREQ:
        case REG_MOT_PWM_DUTY: Motor_ApplyPWM();  break;
        default: break;
    }
}

/* =========================================================================
 * ModBus_SendException
 * ========================================================================= */
static void ModBus_SendException(uint8_t fc, uint8_t ex_code)
{
    uint8_t *tx = s_mb.tx_buf;
    uint16_t tx_len = 0;
    tx[tx_len++] = (uint8_t)g_regs[REG_SYS_ADDR];
    tx[tx_len++] = fc | FC_EXCEPTION_MASK;
    tx[tx_len++] = ex_code;
    ModBus_SendFrame(tx, tx_len);
}

/* =========================================================================
 * ModBus_SendFrame — append CRC and transmit
 * ========================================================================= */
static void ModBus_SendFrame(uint8_t *buf, uint16_t len)
{
    uint16_t crc = CRC16(buf, len);
    buf[len++] = (uint8_t)(crc & 0xFF);
    buf[len++] = (uint8_t)(crc >> 8);
    s_mb.tx_len  = len;
    s_mb.tx_busy = true;
    HAL_UART_Transmit_IT(&huart1, buf, len);
}

/* =========================================================================
 * HAL_UART_TxCpltCallback — must be called from the actual callback in main.c
 * ========================================================================= */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        s_mb.tx_busy = false;
    }
}

/* =========================================================================
 * Encoder_Update — sync TIM2 counter into register map, compute velocity
 * ========================================================================= */
static void Encoder_Update(void)
{
    /* TIM2 is 32-bit; its counter is the raw quadrature count.
     * We offset by 0x8000 at init so negative travel is possible. */
    int32_t raw_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t position  = raw_count - (int32_t)0x8000;

    s_enc.position = position;

    /* Store in registers */
    g_regs[REG_ENC_POS_HI] = (uint16_t)((uint32_t)position >> 16);
    g_regs[REG_ENC_POS_LO] = (uint16_t)((uint32_t)position & 0xFFFF);

    /* Velocity — counts per 100 ms */
    uint32_t now = Tick();
    if ((now - s_enc.last_vel_tick) >= VELOCITY_INTERVAL_MS) {
        int32_t delta = position - s_enc.last_vel_pos;
        /* Scale to counts/100ms */
        int32_t vel = delta;  /* Already 100 ms window */
        if (vel > 32767)  vel = 32767;
        if (vel < -32768) vel = -32768;
        g_regs[REG_ENC_VELOCITY]  = (uint16_t)(int16_t)vel;
        s_enc.last_vel_pos        = position;
        s_enc.last_vel_tick       = now;
    }

    /* Status bits */
    uint16_t status = g_regs[REG_ENC_STATUS] & ENC_STATUS_INDEX_SEEN;  /* Preserve sticky bit */

    /* Limit switches: opto-isolated, pull-up, active LOW → HAL reads RESET when active */
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET)
        status |= ENC_STATUS_AT_LIMIT1;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET)
        status |= ENC_STATUS_AT_LIMIT2;

    /* Overflow: TIM2 wrap (rare — 32-bit counter) */
    /* For simplicity, flag if count exceeds ±2^30 */
    if (position > (int32_t)0x3FFFFFFF || position < (int32_t)(-0x40000000))
        status |= ENC_STATUS_OVERFLOW;

    g_regs[REG_ENC_STATUS] = status;

    /* Motor status */
    uint16_t mot_status = 0;
    if (g_regs[REG_MOT_ENABLE])  mot_status |= MOT_STATUS_ENABLED;
    if (g_regs[REG_MOT_ENABLE] && g_regs[REG_MOT_PWM_DUTY] > 0)
        mot_status |= MOT_STATUS_RUNNING;
    if (g_regs[REG_MOT_DIR] == 0) mot_status |= MOT_STATUS_DIR_CW;
    g_regs[REG_MOT_STATUS] = mot_status;
}

/* =========================================================================
 * PushNotify_Check — send FC=0x41 if position changed enough
 * ========================================================================= */
static void PushNotify_Check(void)
{
    if (!g_regs[REG_ENC_PUSH_EN]) return;
    if (s_mb.tx_busy)             return;  /* Don't interrupt ongoing TX */

    int32_t pos    = s_enc.position;
    int32_t delta  = pos - s_enc.last_push_pos;
    if (delta < 0) delta = -delta;

    if (delta < (int32_t)g_regs[REG_ENC_PUSH_THRESH]) return;

    s_enc.last_push_pos = pos;

    uint32_t upos = (uint32_t)pos;
    uint8_t  flags = 0;
    if (g_regs[REG_ENC_STATUS] & ENC_STATUS_AT_LIMIT1) flags |= 0x01u;
    if (g_regs[REG_ENC_STATUS] & ENC_STATUS_AT_LIMIT2) flags |= 0x02u;
    if (g_regs[REG_ENC_STATUS] & ENC_STATUS_INDEX_SEEN) flags |= 0x04u;

    uint8_t *tx = s_mb.tx_buf;
    uint16_t tx_len = 0;
    tx[tx_len++] = (uint8_t)g_regs[REG_SYS_ADDR];
    tx[tx_len++] = FC_PUSH_NOTIFY;
    tx[tx_len++] = (uint8_t)(upos >> 24);
    tx[tx_len++] = (uint8_t)(upos >> 16);
    tx[tx_len++] = (uint8_t)(upos >> 8);
    tx[tx_len++] = (uint8_t)(upos & 0xFF);
    tx[tx_len++] = flags;
    ModBus_SendFrame(tx, tx_len);
}

/* =========================================================================
 * Motor control helpers
 * ========================================================================= */
static void Motor_ApplyEnable(void)
{
    GPIO_PinState state = g_regs[REG_MOT_ENABLE] ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, state);
}

static void Motor_ApplyDir(void)
{
    /* DIR=0 → CW (GPIO low), DIR=1 → CCW (GPIO high) */
    GPIO_PinState state = g_regs[REG_MOT_DIR] ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, state);
}

static void Motor_ApplyPWM(void)
{
    uint16_t freq = g_regs[REG_MOT_PWM_FREQ];
    uint16_t duty = g_regs[REG_MOT_PWM_DUTY];  /* 0–1000 */

    if (freq < PWM_FREQ_MIN_HZ) freq = PWM_FREQ_MIN_HZ;
    if (freq > PWM_FREQ_MAX_HZ) freq = PWM_FREQ_MAX_HZ;
    if (duty > 1000) duty = 1000;

    /* TIM3 clock = APB1 timer clock = 170 MHz
     * Prescaler in IOC = 16  → TIM3_CLK = 170MHz / (16+1) ≈ 10 MHz
     * Period (ARR) = TIM3_CLK / freq - 1
     * CCR3 = ARR * duty / 1000
     */
    uint32_t arr = (PWM_TIMER_CLK_HZ / (uint32_t)freq) - 1u;
    if (arr > 0xFFFF) arr = 0xFFFF;

    uint32_t ccr = (arr * (uint32_t)duty) / 1000u;

    __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
    TIM3->CCR3 = ccr;

    /* Force update to apply immediately */
    TIM3->EGR = TIM_EGR_UG;
}

/* =========================================================================
 * Sensor_Poll — I2C reads on a timed schedule
 * ========================================================================= */
static void Sensor_Poll(void)
{
    uint32_t now = Tick();
    if ((now - s_sensors.last_poll_tick) < SENSOR_POLL_INTERVAL_MS) return;
    s_sensors.last_poll_tick = now;

    SHT45_Read();
    ADXL345_Read();

    /* Toggle LED to show sensor activity */
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
}

/* =========================================================================
 * SHT45 — High-precision temperature + humidity
 * Command: 0xFD (measure high precision, ~8 ms)
 * Response: [T_MSB][T_LSB][CRC_T][RH_MSB][RH_LSB][CRC_RH] = 6 bytes
 * ========================================================================= */
static uint8_t sht45_crc(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
    return crc;
}

static bool SHT45_Read(void)
{
    uint8_t cmd = 0xFD;
    HAL_StatusTypeDef st;

    st = HAL_I2C_Master_Transmit(&hi2c3, SHT45_I2C_ADDR, &cmd, 1, 10);
    if (st != HAL_OK) {
        g_regs[REG_SHT45_STATUS] = SENSOR_STATUS_TIMEOUT;
        return false;
    }

    HAL_Delay(10);  /* Measurement time */

    uint8_t buf[6];
    st = HAL_I2C_Master_Receive(&hi2c3, SHT45_I2C_ADDR, buf, 6, 10);
    if (st != HAL_OK) {
        g_regs[REG_SHT45_STATUS] = SENSOR_STATUS_TIMEOUT;
        return false;
    }

    /* CRC checks */
    if (sht45_crc(buf,   2) != buf[2] || sht45_crc(buf+3, 2) != buf[5]) {
        g_regs[REG_SHT45_STATUS] = SENSOR_STATUS_CRC_ERR;
        return false;
    }

    /* Convert: T[°C] = -45 + 175 * (raw / 65535)
     *          RH[%] = -6  + 125 * (raw / 65535)     */
    uint16_t t_raw  = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t rh_raw = ((uint16_t)buf[3] << 8) | buf[4];

    /* Scale to 0.01 °C and 0.01 % LSB for 16-bit register storage */
    int32_t temp_cdeg = (int32_t)(-4500) + (int32_t)(17500 * (uint32_t)t_raw / 65535);
    int32_t rh_cpct   = (int32_t)(-600)  + (int32_t)(12500 * (uint32_t)rh_raw / 65535);

    /* Clamp */
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
 * ADXL345 — Triple-axis accelerometer
 * Registers: 0x32–0x37 (DATAX0/1, DATAY0/1, DATAZ0/1)
 * ========================================================================= */

#define ADXL345_REG_POWER_CTL  0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_DATAX0     0x32

static bool s_adxl_init_done = false;

static bool ADXL345_Write_Reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return HAL_I2C_Master_Transmit(&hi2c3, ADXL345_I2C_ADDR, buf, 2, 10) == HAL_OK;
}

static bool ADXL345_Read(void)
{
    /* One-time init: enable measurement, ±2g range, full resolution */
    if (!s_adxl_init_done) {
        if (!ADXL345_Write_Reg(ADXL345_REG_DATA_FORMAT, 0x08)) {
            g_regs[REG_ADXL_STATUS] = SENSOR_STATUS_TIMEOUT;
            return false;
        }
        if (!ADXL345_Write_Reg(ADXL345_REG_POWER_CTL, 0x08)) {
            g_regs[REG_ADXL_STATUS] = SENSOR_STATUS_TIMEOUT;
            return false;
        }
        s_adxl_init_done = true;
    }

    uint8_t reg = ADXL345_REG_DATAX0 | 0x80;  /* Multi-byte read bit */
    HAL_StatusTypeDef st;
    st = HAL_I2C_Master_Transmit(&hi2c3, ADXL345_I2C_ADDR, &reg, 1, 10);
    if (st != HAL_OK) { g_regs[REG_ADXL_STATUS] = SENSOR_STATUS_TIMEOUT; return false; }

    uint8_t buf[6];
    st = HAL_I2C_Master_Receive(&hi2c3, ADXL345_I2C_ADDR, buf, 6, 10);
    if (st != HAL_OK) { g_regs[REG_ADXL_STATUS] = SENSOR_STATUS_TIMEOUT; return false; }

    /* ADXL345 output is little-endian signed 16-bit */
    int16_t x = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    int16_t y = (int16_t)(buf[2] | ((uint16_t)buf[3] << 8));
    int16_t z = (int16_t)(buf[4] | ((uint16_t)buf[5] << 8));

    g_regs[REG_ADXL_X]      = (uint16_t)x;
    g_regs[REG_ADXL_Y]      = (uint16_t)y;
    g_regs[REG_ADXL_Z]      = (uint16_t)z;
    g_regs[REG_ADXL_STATUS] = SENSOR_STATUS_OK;
    return true;
}
