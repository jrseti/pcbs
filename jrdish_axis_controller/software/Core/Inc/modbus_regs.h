#ifndef MODBUS_REGS_H
#define MODBUS_REGS_H

/* -----------------------------------------------------------------------
 * modbus_regs.h
 * Register address definitions for Axis Controller Modbus protocol Rev 1.1
 * ----------------------------------------------------------------------- */

/* Total register count */
#define MODBUS_REG_COUNT        0x00D1

/* ----------------------------------------------------------------------- */
/* System (0x0000 - 0x0007)                                                */
/* ----------------------------------------------------------------------- */
#define REG_SYS_ADDR            0x0000  /* Node address (read-only)        */
#define REG_SYS_FW_VER          0x0001  /* Firmware version                */
#define REG_SYS_UPTIME_HI       0x0002  /* Uptime HIGH word (ms)           */
#define REG_SYS_UPTIME_LO       0x0003  /* Uptime LOW word (ms)            */
#define REG_SYS_RESET           0x0004  /* Write 0xDEAD to reset           */
#define REG_SYS_STATUS          0x0005  /* Global status flags             */
#define REG_SYS_USER_SW         0x0006  /* User switch states (read-only)  */
#define REG_SYS_FAN             0x0007  /* Fan control: 0=off, 1=on        */

/* ----------------------------------------------------------------------- */
/* AZ (connector 2) Encoder Config (0x0010 - 0x0017)                           */
/* ----------------------------------------------------------------------- */
#define REG_AZ_GEAR_RATIO_HI    0x0010  /* Motor:output gear reduction, HIGH word (default 20) */
#define REG_AZ_GEAR_RATIO_LO    0x0011  /* Motor:output gear reduction, LOW word                 */
                                        /* 0x0012-0x0013 reserved                        */
#define REG_AZ_LIM1_POS_HI      0x0014  /* Always 0 — encoder is zeroed at LIMIT1 during homing */
#define REG_AZ_LIM1_POS_LO      0x0015
#define REG_AZ_LIM2_POS_HI      0x0016  /* Encoder count at LIMIT2 — negative (see calibration) */
#define REG_AZ_LIM2_POS_LO      0x0017

/* ----------------------------------------------------------------------- */
/* AZ (connector 2) PID Config (0x0020 - 0x0026)                               */
/* ----------------------------------------------------------------------- */
#define REG_AZ_PID_P_HI         0x0020
#define REG_AZ_PID_P_LO         0x0021
#define REG_AZ_PID_I_HI         0x0022
#define REG_AZ_PID_I_LO         0x0023
#define REG_AZ_PID_D_HI         0x0024
#define REG_AZ_PID_D_LO         0x0025
#define REG_AZ_PID_MAX          0x0026

/* ----------------------------------------------------------------------- */
/* AZ (connector 2) Motion Command (0x0030 - 0x0037)                           */
/* ----------------------------------------------------------------------- */
#define REG_AZ_CMD_ENABLE       0x0030
#define REG_AZ_CMD_MODE         0x0031
#define REG_AZ_CMD_POS_HI       0x0032
#define REG_AZ_CMD_POS_LO       0x0033
#define REG_AZ_CMD_RPM_HI       0x0034
#define REG_AZ_CMD_RPM_LO       0x0035
#define REG_AZ_CMD_HOME         0x0036  /* LIMIT1 is the CW-side switch — home by seeking CW */
#define REG_AZ_CMD_STOP         0x0037

/* ----------------------------------------------------------------------- */
/* AZ (connector 2) Status (0x0040 - 0x0049)                                   */
/* ----------------------------------------------------------------------- */
#define REG_AZ_STATUS           0x0040
#define REG_AZ_POS_HI           0x0041
#define REG_AZ_POS_LO           0x0042
#define REG_AZ_POS_DEG_HI       0x0043
#define REG_AZ_POS_DEG_LO       0x0044
#define REG_AZ_RPM_HI           0x0045
#define REG_AZ_RPM_LO           0x0046
#define REG_AZ_PID_OUT_HI       0x0047
#define REG_AZ_PID_OUT_LO       0x0048
#define REG_AZ_ERROR            0x0049

/* ----------------------------------------------------------------------- */
/* AZ (connector 2) Calibration / Velocity Config (0x004A - 0x0050)         */
/* Placed in the gap after AZ Status — see docs/modbus_protocol.md         */
/* ----------------------------------------------------------------------- */
#define REG_AZ_DRIVER_PPR       0x004A  /* DM556Y pulses/rev DIP setting (default 400) */
#define REG_AZ_MAX_RPM_HI       0x004B  /* Max output-shaft RPM, float32 HIGH (default 1.5) */
#define REG_AZ_MAX_RPM_LO       0x004C
#define REG_AZ_LIM1_ANGLE_HI    0x004D  /* Measured angle at LIMIT1 (home), float32 HIGH */
#define REG_AZ_LIM1_ANGLE_LO    0x004E
#define REG_AZ_LIM2_ANGLE_HI    0x004F  /* Measured angle at LIMIT2, float32 HIGH */
#define REG_AZ_LIM2_ANGLE_LO    0x0050
                                        /* 0x0051-0x006F reserved for future growth */

/* ----------------------------------------------------------------------- */
/* EL (connector 1) Encoder Config (0x0070 - 0x0077)                           */
/* ----------------------------------------------------------------------- */
#define REG_EL_GEAR_RATIO_HI    0x0070  /* Motor:output gear reduction, HIGH word (default 20) */
#define REG_EL_GEAR_RATIO_LO    0x0071  /* Motor:output gear reduction, LOW word                 */
                                        /* 0x0072-0x0073 reserved                        */
#define REG_EL_LIM1_POS_HI      0x0074  /* Always 0 — encoder is zeroed at LIMIT1 during homing */
#define REG_EL_LIM1_POS_LO      0x0075
#define REG_EL_LIM2_POS_HI      0x0076  /* Encoder count at LIMIT2 — negative (see calibration) */
#define REG_EL_LIM2_POS_LO      0x0077

/* ----------------------------------------------------------------------- */
/* EL (connector 1) PID Config (0x0080 - 0x0086)                               */
/* ----------------------------------------------------------------------- */
#define REG_EL_PID_P_HI         0x0080
#define REG_EL_PID_P_LO         0x0081
#define REG_EL_PID_I_HI         0x0082
#define REG_EL_PID_I_LO         0x0083
#define REG_EL_PID_D_HI         0x0084
#define REG_EL_PID_D_LO         0x0085
#define REG_EL_PID_MAX          0x0086

/* ----------------------------------------------------------------------- */
/* EL (connector 1) Motion Command (0x0090 - 0x0097)                           */
/* ----------------------------------------------------------------------- */
#define REG_EL_CMD_ENABLE       0x0090
#define REG_EL_CMD_MODE         0x0091
#define REG_EL_CMD_POS_HI       0x0092
#define REG_EL_CMD_POS_LO       0x0093
#define REG_EL_CMD_RPM_HI       0x0094
#define REG_EL_CMD_RPM_LO       0x0095
#define REG_EL_CMD_HOME         0x0096  /* LIMIT1 is the CW-side switch — home by seeking CW */
#define REG_EL_CMD_STOP         0x0097

/* ----------------------------------------------------------------------- */
/* EL (connector 1) Status (0x00A0 - 0x00A9)                                   */
/* ----------------------------------------------------------------------- */
#define REG_EL_STATUS           0x00A0
#define REG_EL_POS_HI           0x00A1
#define REG_EL_POS_LO           0x00A2
#define REG_EL_POS_DEG_HI       0x00A3
#define REG_EL_POS_DEG_LO       0x00A4
#define REG_EL_RPM_HI           0x00A5
#define REG_EL_RPM_LO           0x00A6
#define REG_EL_PID_OUT_HI       0x00A7
#define REG_EL_PID_OUT_LO       0x00A8
#define REG_EL_ERROR            0x00A9

/* ----------------------------------------------------------------------- */
/* GPS / Timing (0x00B0 - 0x00B4)                                          */
/* ----------------------------------------------------------------------- */
#define REG_GPS_STATUS          0x00B0
#define REG_GPS_UTC_HH_MM       0x00B1
#define REG_GPS_UTC_SS          0x00B2
#define REG_GPS_1PPS_COUNT_HI   0x00B3
#define REG_GPS_1PPS_COUNT_LO   0x00B4

/* ----------------------------------------------------------------------- */
/* SYS_STATUS bits                                                          */
/* ----------------------------------------------------------------------- */
#define SYS_STATUS_AZ_OK        (1u << 0)
#define SYS_STATUS_EL_OK        (1u << 1)
#define SYS_STATUS_GPS_FIX      (1u << 2)
#define SYS_STATUS_RS485_ACTIVE (1u << 3)

/* ----------------------------------------------------------------------- */
/* AXIS_STATUS bits                                                         */
/* ----------------------------------------------------------------------- */
#define AXIS_STATUS_ENABLED     (1u << 0)
#define AXIS_STATUS_MOVING      (1u << 1)
#define AXIS_STATUS_AT_TARGET   (1u << 2)
#define AXIS_STATUS_HOMING      (1u << 3)
#define AXIS_STATUS_HOMED       (1u << 4)
#define AXIS_STATUS_LIMIT1      (1u << 5)
#define AXIS_STATUS_LIMIT2      (1u << 6)
#define AXIS_STATUS_ERROR       (1u << 7)

/* ----------------------------------------------------------------------- */
/* Axis error codes                                                         */
/* ----------------------------------------------------------------------- */
#define AXIS_ERR_NONE           0x00
#define AXIS_ERR_ENCODER        0x01
#define AXIS_ERR_LIMIT_FAULT    0x02
#define AXIS_ERR_HOMING_FAILED  0x03
#define AXIS_ERR_PID_SATURATED  0x04
#define AXIS_ERR_POS_RANGE      0x05
#define AXIS_ERR_NOT_CALIBRATED 0x06  /* LIM2_ANGLE==LIM1_ANGLE or LIM2_POS==0 */

/* ----------------------------------------------------------------------- */
/* SPI Flash (0x00C0 - 0x00C2)                                             */
/* ----------------------------------------------------------------------- */
#define REG_FLASH_CMD           0x00C0  /* 1=write, 2=read, 3=erase        */
#define REG_FLASH_DATA          0x00C1  /* 16-bit value to write/read      */
#define REG_FLASH_STATUS        0x00C2  /* 0=idle,1=busy,2=pass,3=fail     */

/* FLASH_CMD values */
#define FLASH_CMD_WRITE         0x0001
#define FLASH_CMD_READ          0x0002
#define FLASH_CMD_ERASE         0x0003

/* FLASH_STATUS values */
#define FLASH_STATUS_IDLE       0x0000
#define FLASH_STATUS_BUSY       0x0001
#define FLASH_STATUS_PASS       0x0002
#define FLASH_STATUS_FAIL       0x0003

/* ----------------------------------------------------------------------- */
/* PWM Frequency and Direction (0x00C3 - 0x00C6)                           */
/* ----------------------------------------------------------------------- */
#define REG_AZ_PWM_FREQ         0x00C3  /* AZ pulse freq in Hz (1-10000)   */
#define REG_EL_PWM_FREQ         0x00C4  /* EL pulse freq in Hz (1-10000)   */
#define REG_AZ_PWM_DIR          0x00C5  /* AZ direction: 0=CW, 1=CCW       */
#define REG_EL_PWM_DIR          0x00C6  /* EL direction: 0=CW, 1=CCW       */

/* ----------------------------------------------------------------------- */
/* Encoder Z pulse counts (0x00C7 - 0x00C8)                                */
/* ----------------------------------------------------------------------- */
#define REG_AZ_Z_COUNT          0x00C7
#define REG_EL_Z_COUNT          0x00C8

/* ----------------------------------------------------------------------- */
/* Limit switch states (0x00C9)                                             */
/* ----------------------------------------------------------------------- */
#define REG_LIMIT_SW            0x00C9  /* bit0=AZ_LIM1, bit1=AZ_LIM2,    */
                                        /* bit2=EL_LIM1, bit3=EL_LIM2      */

/* ----------------------------------------------------------------------- */
/* EL (connector 1) Calibration / Velocity Config (0x00CA - 0x00D0)        */
/* EL's mirrored gap (0x00AA-0x00AF) is only 6 registers, too small for    */
/* this 7-register block — appended after the map end instead.            */
/* ----------------------------------------------------------------------- */
#define REG_EL_DRIVER_PPR       0x00CA  /* DM556Y pulses/rev DIP setting (default 400) */
#define REG_EL_MAX_RPM_HI       0x00CB  /* Max output-shaft RPM, float32 HIGH (default 1.5) */
#define REG_EL_MAX_RPM_LO       0x00CC
#define REG_EL_LIM1_ANGLE_HI    0x00CD  /* Measured angle at LIMIT1 (home), float32 HIGH */
#define REG_EL_LIM1_ANGLE_LO    0x00CE
#define REG_EL_LIM2_ANGLE_HI    0x00CF  /* Measured angle at LIMIT2, float32 HIGH */
#define REG_EL_LIM2_ANGLE_LO    0x00D0

/* ----------------------------------------------------------------------- */
/* Modbus constants                                                         */
/* ----------------------------------------------------------------------- */
#define MODBUS_BROADCAST_ADDR   0xFF
#define MODBUS_RESET_KEY        0xDEAD
#define MODBUS_FW_VERSION       0x0100  /* v1.0 */

#define FC_READ_REGS            0x03
#define FC_WRITE_SINGLE         0x06
#define FC_WRITE_MULTI          0x10

#define MB_EX_ILLEGAL_FUNC      0x01
#define MB_EX_ILLEGAL_ADDR      0x02
#define MB_EX_ILLEGAL_VALUE     0x03
#define MB_EX_DEVICE_FAILURE    0x04

#endif /* MODBUS_REGS_H */
