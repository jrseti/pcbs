# Axis Controller - Modbus RTU Protocol Definition
## Rev 1.1 — Dual Axis, STM32G474RET6

---

## 1. Physical Layer

- **Port:** RS485_1 (USART1, isolated ISOW1432B transceiver)
- **Baud rate:** 115200, 8N1, no parity
- **Topology:** Multi-drop RS485 bus, up to 32 nodes
- **Node address:** Set by ADDR0/ADDR1/ADDR2 DIP switches at boot (1–7)
- **RS485_2** is reserved for servo motor controllers — not used for Modbus

---

## 2. Addressing

| Address | Meaning |
|---------|---------|
| 0x01–0x07 | Board node address (set by ADDR0–ADDR2 DIP switches) |
| 0xFF | Broadcast — all boards on bus ingest, none reply |

The board reads ADDR0/ADDR1/ADDR2 GPIO pins at boot and stores the
3-bit value as its node address (1–7). Address 0x00 is not used
(reserved by Modbus standard for broadcast; we use 0xFF instead to
avoid conflicts with standard Modbus tools).

Incoming frames with an address that does not match the board's node
address AND is not 0xFF are silently discarded.

Broadcast frames (0xFF) are processed but the board does NOT transmit
a response — this is required to prevent bus collisions.

---

## 3. Frame Format

Standard Modbus RTU framing:

```
[ADDR 1B][FC 1B][DATA nB][CRC-16 2B]
```

- CRC-16 is standard Modbus CRC (polynomial 0x8005), LSB first
- Inter-frame silence gap: 3.5 character times (~0.32ms at 115200 baud)

---

## 4. Function Codes

| Code | Name                    | Direction          |
|------|-------------------------|--------------------|
| 0x03 | Read Holding Registers  | Host → Board       |
| 0x06 | Write Single Register   | Host → Board       |
| 0x10 | Write Multiple Regs     | Host → Board       |

The protocol is strictly master/slave polling. The board never transmits
unsolicited frames. The host polls each board in round-robin to read
status and position. This avoids RS485 bus collisions when multiple
boards are present.

---

## 5. Register Map

All registers are 16-bit. 32-bit values occupy two consecutive
registers (HI word at lower address, LO word at next address).
Float32 values are IEEE 754, stored as two 16-bit registers.

Axis 1 = Azimuth (AZ)
Axis 2 = Elevation (EL)

---

### 5.1 System Configuration (Read/Write)

| Address | Name              | Type    | Description                          | Default |
|---------|-------------------|---------|--------------------------------------|---------|
| 0x0000  | SYS_ADDR          | uint16  | Node address (read-only, from DIP)   | —       |
| 0x0001  | SYS_FW_VER        | uint16  | Firmware version (e.g. 0x0100=v1.0) | —       |
| 0x0002  | SYS_UPTIME_HI     | uint16  | System uptime HIGH word (ms)         | —       |
| 0x0003  | SYS_UPTIME_LO     | uint16  | System uptime LOW word (ms)          | —       |
| 0x0004  | SYS_RESET         | uint16  | Write 0xDEAD to soft-reset MCU       | —       |
| 0x0005  | SYS_STATUS        | uint16  | Global status flags (see §7)         | —       |
| 0x0006  | SYS_USER_SW       | uint16  | User switch states (read-only, see §7) | —     |
| 0x0007  | SYS_FAN           | uint16  | Fan control: 0=off, 1=on             | 0       |

---

### 5.2 Axis 1 (AZ) — Encoder Configuration (Read/Write)

| Address | Name                | Type    | Description                          | Default |
|---------|---------------------|---------|--------------------------------------|---------|
| 0x0010  | AZ_ENC_PPR          | uint16  | Encoder pulses per revolution        | 2000    |
| 0x0011  | AZ_GEAR_RATIO_HI    | uint16  | Motor→shaft gear ratio HIGH word     | 0       |
| 0x0012  | AZ_GEAR_RATIO_LO    | uint16  | Motor→shaft gear ratio LOW word      | 6000    |
| 0x0013  | AZ_PULLEY_RATIO     | uint16  | Shaft→encoder pulley ratio (×100)    | 100     |
| 0x0014  | AZ_LIM1_POS_HI      | uint16  | Limit 1 encoder count HIGH word      | 0       |
| 0x0015  | AZ_LIM1_POS_LO      | uint16  | Limit 1 encoder count LOW word       | 0       |
| 0x0016  | AZ_LIM2_POS_HI      | uint16  | Limit 2 encoder count HIGH word      | 0       |
| 0x0017  | AZ_LIM2_POS_LO      | uint16  | Limit 2 encoder count LOW word       | 0       |

---

### 5.3 Axis 1 (AZ) — PID Configuration (Read/Write)

| Address | Name         | Type    | Description              | Default |
|---------|--------------|---------|--------------------------|---------|
| 0x0020  | AZ_PID_P_HI  | uint16  | Proportional gain HIGH   | 0       |
| 0x0021  | AZ_PID_P_LO  | uint16  | Proportional gain LOW    | 0       |
| 0x0022  | AZ_PID_I_HI  | uint16  | Integral gain HIGH       | 0       |
| 0x0023  | AZ_PID_I_LO  | uint16  | Integral gain LOW        | 0       |
| 0x0024  | AZ_PID_D_HI  | uint16  | Derivative gain HIGH     | 0       |
| 0x0025  | AZ_PID_D_LO  | uint16  | Derivative gain LOW      | 0       |
| 0x0026  | AZ_PID_MAX   | uint16  | PID output clamp (0–1000)| 1000    |

PID gains are float32 stored as HI/LO uint16 pairs (IEEE 754).

---

### 5.4 Axis 1 (AZ) — Motion Command (Read/Write)

| Address | Name              | Type    | Description                                      |
|---------|-------------------|---------|--------------------------------------------------|
| 0x0030  | AZ_CMD_ENABLE     | uint16  | Motor enable: 0=off, 1=on                        |
| 0x0031  | AZ_CMD_MODE       | uint16  | 0=position, 1=constant RPM                       |
| 0x0032  | AZ_CMD_POS_HI     | uint16  | Target position HIGH word (encoder counts)       |
| 0x0033  | AZ_CMD_POS_LO     | uint16  | Target position LOW word                         |
| 0x0034  | AZ_CMD_RPM_HI     | uint16  | Target RPM HIGH word (float32, shaft RPM)        |
| 0x0035  | AZ_CMD_RPM_LO     | uint16  | Target RPM LOW word                              |
| 0x0036  | AZ_CMD_HOME       | uint16  | Write 1 to start homing sequence                 |
| 0x0037  | AZ_CMD_STOP       | uint16  | Write 1 to emergency stop                        |

---

### 5.5 Axis 1 (AZ) — Status (Read-Only)

| Address | Name              | Type    | Description                          |
|---------|-------------------|---------|--------------------------------------|
| 0x0040  | AZ_STATUS         | uint16  | Axis status flags (see §7)           |
| 0x0041  | AZ_POS_HI         | uint16  | Current encoder position HIGH word   |
| 0x0042  | AZ_POS_LO         | uint16  | Current encoder position LOW word    |
| 0x0043  | AZ_POS_DEG_HI     | uint16  | Current position degrees HIGH (f32)  |
| 0x0044  | AZ_POS_DEG_LO     | uint16  | Current position degrees LOW (f32)   |
| 0x0045  | AZ_RPM_HI         | uint16  | Current shaft RPM HIGH (float32)     |
| 0x0046  | AZ_RPM_LO         | uint16  | Current shaft RPM LOW (float32)      |
| 0x0047  | AZ_PID_OUT_HI     | uint16  | PID output value HIGH (float32)      |
| 0x0048  | AZ_PID_OUT_LO     | uint16  | PID output value LOW (float32)       |
| 0x0049  | AZ_ERROR          | uint16  | Axis error code (see §8)             |

---

### 5.6 Axis 2 (EL) — Encoder Configuration (Read/Write)

Same structure as Axis 1, offset by 0x0060:

| Address | Name                | Type    | Description                          | Default |
|---------|---------------------|---------|--------------------------------------|---------|
| 0x0070  | EL_ENC_PPR          | uint16  | Encoder pulses per revolution        | 2000    |
| 0x0071  | EL_GEAR_RATIO_HI    | uint16  | Motor→shaft gear ratio HIGH word     | 0       |
| 0x0072  | EL_GEAR_RATIO_LO    | uint16  | Motor→shaft gear ratio LOW word      | 6000    |
| 0x0073  | EL_PULLEY_RATIO     | uint16  | Shaft→encoder pulley ratio (×100)    | 100     |
| 0x0074  | EL_LIM1_POS_HI      | uint16  | Limit 1 encoder count HIGH word      | 0       |
| 0x0075  | EL_LIM1_POS_LO      | uint16  | Limit 1 encoder count LOW word       | 0       |
| 0x0076  | EL_LIM2_POS_HI      | uint16  | Limit 2 encoder count HIGH word      | 0       |
| 0x0077  | EL_LIM2_POS_LO      | uint16  | Limit 2 encoder count LOW word       | 0       |

---

### 5.7 Axis 2 (EL) — PID Configuration (Read/Write)

| Address | Name         | Type    | Description              | Default |
|---------|--------------|---------|--------------------------|---------|
| 0x0080  | EL_PID_P_HI  | uint16  | Proportional gain HIGH   | 0       |
| 0x0081  | EL_PID_P_LO  | uint16  | Proportional gain LOW    | 0       |
| 0x0082  | EL_PID_I_HI  | uint16  | Integral gain HIGH       | 0       |
| 0x0083  | EL_PID_I_LO  | uint16  | Integral gain LOW        | 0       |
| 0x0084  | EL_PID_D_HI  | uint16  | Derivative gain HIGH     | 0       |
| 0x0085  | EL_PID_D_LO  | uint16  | Derivative gain LOW      | 0       |
| 0x0086  | EL_PID_MAX   | uint16  | PID output clamp (0–1000)| 1000    |

---

### 5.8 Axis 2 (EL) — Motion Command (Read/Write)

| Address | Name              | Type    | Description                                      |
|---------|-------------------|---------|--------------------------------------------------|
| 0x0090  | EL_CMD_ENABLE     | uint16  | Motor enable: 0=off, 1=on                        |
| 0x0091  | EL_CMD_MODE       | uint16  | 0=position, 1=constant RPM                       |
| 0x0092  | EL_CMD_POS_HI     | uint16  | Target position HIGH word (encoder counts)       |
| 0x0093  | EL_CMD_POS_LO     | uint16  | Target position LOW word                         |
| 0x0094  | EL_CMD_RPM_HI     | uint16  | Target RPM HIGH word (float32, shaft RPM)        |
| 0x0095  | EL_CMD_RPM_LO     | uint16  | Target RPM LOW word                              |
| 0x0096  | EL_CMD_HOME       | uint16  | Write 1 to start homing sequence                 |
| 0x0097  | EL_CMD_STOP       | uint16  | Write 1 to emergency stop                        |

---

### 5.9 Axis 2 (EL) — Status (Read-Only)

| Address | Name              | Type    | Description                          |
|---------|-------------------|---------|--------------------------------------|
| 0x00A0  | EL_STATUS         | uint16  | Axis status flags (see §7)           |
| 0x00A1  | EL_POS_HI         | uint16  | Current encoder position HIGH word   |
| 0x00A2  | EL_POS_LO         | uint16  | Current encoder position LOW word    |
| 0x00A3  | EL_POS_DEG_HI     | uint16  | Current position degrees HIGH (f32)  |
| 0x00A4  | EL_POS_DEG_LO     | uint16  | Current position degrees LOW (f32)   |
| 0x00A5  | EL_RPM_HI         | uint16  | Current shaft RPM HIGH (float32)     |
| 0x00A6  | EL_RPM_LO         | uint16  | Current shaft RPM LOW (float32)      |
| 0x00A7  | EL_PID_OUT_HI     | uint16  | PID output value HIGH (float32)      |
| 0x00A8  | EL_PID_OUT_LO     | uint16  | PID output value LOW (float32)       |
| 0x00A9  | EL_ERROR          | uint16  | Axis error code (see §8)             |

---

### 5.10 GPS / Timing (Read-Only)

| Address | Name              | Type    | Description                          |
|---------|-------------------|---------|--------------------------------------|
| 0x00B0  | GPS_STATUS        | uint16  | 0=no fix, 1=fix, 2=disciplined       |
| 0x00B1  | GPS_UTC_HH_MM     | uint16  | UTC time: HH×100 + MM                |
| 0x00B2  | GPS_UTC_SS        | uint16  | UTC seconds                          |
| 0x00B3  | GPS_1PPS_COUNT_HI | uint16  | 1PPS timer capture count HIGH        |
| 0x00B4  | GPS_1PPS_COUNT_LO | uint16  | 1PPS timer capture count LOW         |

---

## 6. Polling Strategy

The host polls each board in a round-robin loop. A typical cycle for
N boards on the bus:

```
for each board (addr 1..N):
    FC 0x03 — read SYS registers (0x0000–0x0007): uptime, status, switches, fan
    FC 0x03 — read AZ status (0x0040–0x0049): position, RPM, PID out, error
    FC 0x03 — read EL status (0x00A0–0x00A9): position, RPM, PID out, error
```

At 115200 baud each FC 0x03 read of 10 registers takes ~3ms round trip.
Three reads per board × 7 boards = ~63ms worst case, giving ~15Hz
position update rate across a full 7-board bus. With fewer boards the
rate is proportionally higher.

Limit switch and error events are detected by inspecting AZ_STATUS,
EL_STATUS, and AZ_ERROR/EL_ERROR bits on every poll cycle — no
unsolicited frames are needed.

---

## 7. Status Flag Bits

### SYS_STATUS (0x0005)
```
Bit 0 — AZ axis OK
Bit 1 — EL axis OK
Bit 2 — GPS fix
Bit 3 — RS485_1 active
Bit 4–15 — reserved
```

### SYS_USER_SW (0x0006) — Read-Only
```
Bit 0 — USER_SW_1 (0=open, 1=pressed)
Bit 1 — USER_SW_2 (0=open, 1=pressed)
Bit 2 — USER_SW_3 (0=open, 1=pressed)
Bit 3 — USER_SW_4 (0=open, 1=pressed)
Bit 4–15 — reserved
```

Switches are active-low inputs with internal pull-ups. The register
reflects the logical state: 1=pressed (GPIO low), 0=open (GPIO high).

### AZ_STATUS / EL_STATUS
```
Bit 0 — ENABLED
Bit 1 — MOVING
Bit 2 — AT_TARGET (within deadband)
Bit 3 — HOMING
Bit 4 — HOMED
Bit 5 — LIMIT1_ACTIVE
Bit 6 — LIMIT2_ACTIVE
Bit 7 — ERROR (see AZ_ERROR / EL_ERROR)
```

---

## 8. Error Codes (AZ_ERROR / EL_ERROR)

| Code | Meaning                  |
|------|--------------------------|
| 0x00 | No error                 |
| 0x01 | Encoder fault            |
| 0x02 | Limit switch fault       |
| 0x03 | Homing failed            |
| 0x04 | PID output saturated     |
| 0x05 | Position command out of range |

---

## 9. Standard Modbus Exception Responses

| Exception Code | Meaning                  |
|----------------|--------------------------|
| 0x01           | Illegal Function         |
| 0x02           | Illegal Register Address |
| 0x03           | Illegal Data Value       |
| 0x04           | Slave Device Failure     |

Exception frame: `[ADDR][FC|0x80][EXCEPTION_CODE][CRC-16]`

---

## 10. Example Transactions

### Read AZ encoder position (FC 0x03)
```
Request:  [01][03][00 41][00 02][xx xx]
           addr fc  reg   qty    crc
Response: [01][03][04][00 01][23 45][xx xx]
           addr fc  bc  POS_HI     POS_LO  crc
```

### Enable AZ motor (FC 0x06)
```
Request:  [01][06][00 30][00 01][xx xx]
           addr fc  reg   val    crc
Response: Echo of request
```

### Set AZ PID gains (FC 0x10, write 6 registers)
```
Request:  [01][10][00 20][00 06][0C][...12 bytes of float32 P/I/D...][crc]
Response: [01][10][00 20][00 06][crc]
```

### Broadcast stop all axes (FC 0x06, address 0xFF)
```
Request:  [FF][06][00 37][00 01][xx xx]   <- AZ stop
          [FF][06][00 97][00 01][xx xx]   <- EL stop
No response expected from any board.
```

### Read user switches and fan state (FC 0x03)
```
Request:  [01][03][00 06][00 02][xx xx]   <- read SYS_USER_SW + SYS_FAN
Response: [01][03][04][00 05][00 01][xx xx]
                       SW=0b0101  FAN=on
SW bits: USER_SW_1 and USER_SW_3 pressed
```

### Turn fan on (FC 0x06)
```
Request:  [01][06][00 07][00 01][xx xx]
           addr fc  reg   val=1  crc
Response: Echo of request
```
