#!/usr/bin/env python3
"""
modbus_menu.py
Interactive Modbus RTU menu for Axis Controller board Rev 1.1

Usage:
    python3 modbus_menu.py --port /dev/tty.usbserial-XXXX --addr 1
"""

import serial
import struct
import threading
import time
import argparse
import select
import sys
import os

# -----------------------------------------------------------------------
# Register addresses
# -----------------------------------------------------------------------
REG_SYS_ADDR        = 0x0000
REG_SYS_FW_VER      = 0x0001
REG_SYS_UPTIME_HI   = 0x0002
REG_SYS_UPTIME_LO   = 0x0003
REG_SYS_STATUS      = 0x0005
REG_SYS_USER_SW     = 0x0006
REG_SYS_FAN         = 0x0007

REG_AZ_GEAR_RATIO_HI = 0x0010
REG_AZ_LIM1_POS_HI   = 0x0014
REG_AZ_LIM2_POS_HI   = 0x0016

REG_AZ_CMD_ENABLE   = 0x0030
REG_AZ_CMD_MODE     = 0x0031
REG_AZ_CMD_POS_HI   = 0x0032
REG_AZ_CMD_HOME     = 0x0036
REG_AZ_CMD_STOP     = 0x0037

REG_AZ_STATUS       = 0x0040
REG_AZ_POS_HI       = 0x0041
REG_AZ_POS_DEG_HI   = 0x0043
REG_AZ_RPM_HI       = 0x0045
REG_AZ_PID_OUT_HI   = 0x0047
REG_AZ_ERROR        = 0x0049

REG_AZ_DRIVER_PPR    = 0x004A
REG_AZ_MAX_RPM_HI    = 0x004B
REG_AZ_LIM1_ANGLE_HI = 0x004D
REG_AZ_LIM2_ANGLE_HI = 0x004F

REG_EL_GEAR_RATIO_HI = 0x0070
REG_EL_LIM1_POS_HI   = 0x0074
REG_EL_LIM2_POS_HI   = 0x0076

REG_EL_CMD_ENABLE   = 0x0090
REG_EL_CMD_MODE     = 0x0091
REG_EL_CMD_POS_HI   = 0x0092
REG_EL_CMD_HOME     = 0x0096
REG_EL_CMD_STOP     = 0x0097

REG_EL_STATUS       = 0x00A0
REG_EL_POS_HI       = 0x00A1
REG_EL_POS_DEG_HI   = 0x00A3
REG_EL_RPM_HI       = 0x00A5
REG_EL_PID_OUT_HI   = 0x00A7
REG_EL_ERROR        = 0x00A9

REG_EL_DRIVER_PPR    = 0x00CA
REG_EL_MAX_RPM_HI    = 0x00CB
REG_EL_LIM1_ANGLE_HI = 0x00CD
REG_EL_LIM2_ANGLE_HI = 0x00CF

REG_GPS_STATUS      = 0x00B0
REG_GPS_UTC_HH_MM   = 0x00B1
REG_GPS_UTC_SS      = 0x00B2

REG_FLASH_CMD       = 0x00C0
REG_FLASH_DATA      = 0x00C1
REG_FLASH_STATUS    = 0x00C2

REG_AZ_PWM_FREQ     = 0x00C3
REG_EL_PWM_FREQ     = 0x00C4
REG_AZ_PWM_DIR      = 0x00C5
REG_EL_PWM_DIR      = 0x00C6
REG_AZ_Z_COUNT      = 0x00C7
REG_EL_Z_COUNT      = 0x00C8
REG_LIMIT_SW        = 0x00C9

FLASH_CMD_ERASE     = 1
FLASH_CMD_WRITE     = 2
FLASH_CMD_READ      = 3

FLASH_STATUS = {0: "Idle", 1: "Busy", 2: "PASS ✓", 3: "FAIL ✗"}

# Pulse rate used for "Move to angle" moves. The board's encoder inputs lose
# counts above ~150 Hz (RC low-pass on the PCB), so a full-speed move arrives
# at a position the encoder under-counted. 50 Hz keeps every edge well within
# the input's bandwidth. The firmware position-move loop has no direct
# frequency knob — it derives speed from MAX_RPM — so action_move_to_angle
# hits this rate by temporarily setting MAX_RPM to the equivalent value.
MOVE_FREQ_HZ = 50

AXIS_STATUS_NAMES = {
    0: "ENABLED",
    1: "MOVING",
    2: "AT_TARGET",
    3: "HOMING",
    4: "HOMED",
    5: "LIMIT1",
    6: "LIMIT2",
    7: "ERROR",
}

ERROR_NAMES = {
    0x00: "None",
    0x01: "Encoder fault",
    0x02: "Limit switch fault",
    0x03: "Homing failed",
    0x04: "PID saturated",
    0x05: "Position out of range",
    0x06: "Not calibrated",
}

# -----------------------------------------------------------------------
# CRC-16
# -----------------------------------------------------------------------
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def append_crc(data: bytes) -> bytes:
    c = crc16(data)
    return data + bytes([c & 0xFF, (c >> 8) & 0xFF])

def check_crc(data: bytes) -> bool:
    if len(data) < 4:
        return False
    return crc16(data[:-2]) == (data[-2] | (data[-1] << 8))

def to_signed32(raw: int) -> int:
    """Reinterpret a 32-bit unsigned value as signed — encoder counts on the
    LIMIT2 side of home are negative (LIMIT1 is the CW end of travel)."""
    return raw - 0x100000000 if raw & 0x80000000 else raw

# -----------------------------------------------------------------------
# Modbus client
# -----------------------------------------------------------------------
class ModbusClient:
    def __init__(self, port, baud, addr):
        self.addr = addr
        self.ser  = serial.Serial(
            port=port, baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5
        )
        # Several menu actions poll the bus from a background thread (e.g. the
        # live encoder display in action_motor_control) while the main thread
        # is also issuing writes. Without a lock, two threads can interleave
        # reset_input_buffer()/write()/read() on the same half-duplex RS485
        # port, corrupting or misattributing each other's responses — a write
        # (e.g. a direction change) can silently fail to reach the MCU while
        # the UI still believes it took effect.
        self._lock = threading.Lock()
        time.sleep(0.1)

    def close(self):
        self.ser.close()

    def _transact(self, frame: bytes, expected: int) -> bytes | None:
        with self._lock:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            resp = self.ser.read(expected)
        if len(resp) < 4 or not check_crc(resp):
            return None
        if resp[0] != self.addr or resp[1] & 0x80:
            return None
        return resp

    def read_regs(self, start: int, qty: int) -> list[int] | None:
        frame = append_crc(struct.pack('>BBHH', self.addr, 0x03, start, qty))
        resp  = self._transact(frame, 5 + qty * 2)
        if resp is None:
            return None
        return [(resp[3 + i*2] << 8) | resp[4 + i*2] for i in range(qty)]

    def write_reg(self, reg: int, value: int) -> bool:
        frame = append_crc(struct.pack('>BBHH', self.addr, 0x06, reg, value))
        return self._transact(frame, 8) is not None

    def write_regs(self, start: int, values: list[int]) -> bool:
        """FC 0x10 — write multiple registers atomically (e.g. a float32
        HI/LO pair). This is the race-free way to set a target position."""
        byte_count = len(values) * 2
        payload = struct.pack('>BBHHB', self.addr, 0x10, start, len(values), byte_count)
        for v in values:
            payload += struct.pack('>H', v & 0xFFFF)
        return self._transact(append_crc(payload), 8) is not None

    def write_float(self, reg_hi: int, value: float) -> bool:
        raw = struct.unpack('>I', struct.pack('>f', value))[0]
        return self.write_regs(reg_hi, [(raw >> 16) & 0xFFFF, raw & 0xFFFF])

    def write_int32(self, reg_hi: int, value: int) -> bool:
        raw = value & 0xFFFFFFFF
        return self.write_regs(reg_hi, [(raw >> 16) & 0xFFFF, raw & 0xFFFF])

    def read_reg(self, reg: int) -> int | None:
        r = self.read_regs(reg, 1)
        return r[0] if r else None

    def read_float(self, reg_hi: int) -> float | None:
        r = self.read_regs(reg_hi, 2)
        if r is None:
            return None
        return struct.unpack('>f', struct.pack('>I', (r[0] << 16) | r[1]))[0]

    def read_int32(self, reg_hi: int) -> int | None:
        r = self.read_regs(reg_hi, 2)
        return to_signed32((r[0] << 16) | r[1]) if r else None

# -----------------------------------------------------------------------
# Display helpers
# -----------------------------------------------------------------------
def clear():
    os.system('cls' if os.name == 'nt' else 'clear')

def axis_status_str(status: int) -> str:
    active = [name for bit, name in AXIS_STATUS_NAMES.items() if status & (1 << bit)]
    return ", ".join(active) if active else "IDLE"

def fmt_uptime(ms: int) -> str:
    s  = ms // 1000
    m  = s  // 60
    h  = m  // 60
    s %= 60
    m %= 60
    return f"{h:02d}:{m:02d}:{s:02d}.{(ms % 1000)//100}"

def header(mb: ModbusClient, port: str):
    print("=" * 56)
    print(f"  Axis Controller  |  Port: {port}  |  Node: {mb.addr}")
    print("=" * 56)

# -----------------------------------------------------------------------
# Menu actions
# -----------------------------------------------------------------------
def action_query_system(mb: ModbusClient):
    print("\n[ System Status ]\n")
    vals = mb.read_regs(REG_SYS_ADDR, 8)
    if vals is None:
        print("  ERROR: No response from board")
        return

    addr   = vals[0]
    fw     = vals[1]
    uptime = (vals[2] << 16) | vals[3]
    status = vals[5]
    sw     = vals[6]
    fan    = vals[7]

    print(f"  Node address  : {addr}")
    print(f"  Firmware      : v{fw >> 8}.{fw & 0xFF}")
    print(f"  Uptime        : {fmt_uptime(uptime)}")
    print(f"  SYS_STATUS    : {status:#06x}")
    print(f"  Fan           : {'ON  ◀' if fan else 'off'}")
    print(f"  USER_SW       : SW1={'▣' if sw&1 else '□'}  "
          f"SW2={'▣' if (sw>>1)&1 else '□'}  "
          f"SW3={'▣' if (sw>>2)&1 else '□'}  "
          f"SW4={'▣' if (sw>>3)&1 else '□'}")

def action_query_axis(mb: ModbusClient, name: str, base: int, pos_hi: int,
                       deg_hi: int, rpm_hi: int, pid_hi: int, err_reg: int):
    print(f"\n[ {name} Axis Status ]\n")
    vals = mb.read_regs(base, 10)
    if vals is None:
        print("  ERROR: No response from board")
        return

    status = vals[0]
    pos    = to_signed32((vals[1] << 16) | vals[2])
    deg_r  = (vals[3] << 16) | vals[4]
    rpm_r  = (vals[5] << 16) | vals[6]
    pid_r  = (vals[7] << 16) | vals[8]
    error  = vals[9]

    deg = struct.unpack('>f', struct.pack('>I', deg_r))[0]
    rpm = struct.unpack('>f', struct.pack('>I', rpm_r))[0]
    pid = struct.unpack('>f', struct.pack('>I', pid_r))[0]

    print(f"  Status        : {axis_status_str(status)}")
    print(f"  Position      : {pos} counts  /  {deg:.4f}°")
    print(f"  RPM           : {rpm:.3f}")
    print(f"  PID output    : {pid:.3f}")
    print(f"  Error         : {ERROR_NAMES.get(error, f'Unknown ({error:#04x})')}")

def action_fan_on(mb: ModbusClient):
    print("\n  Turning fan ON...")
    ok = mb.write_reg(REG_SYS_FAN, 1)
    print("  Done." if ok else "  ERROR: No response")

def action_fan_off(mb: ModbusClient):
    print("\n  Turning fan OFF...")
    ok = mb.write_reg(REG_SYS_FAN, 0)
    print("  Done." if ok else "  ERROR: No response")

def action_monitor(mb: ModbusClient):
    """Live monitor — refreshes every second until Enter is pressed."""
    print("\n  Live monitor — press Enter to stop\n")
    import threading
    stop = threading.Event()

    def wait_enter():
        input()
        stop.set()

    threading.Thread(target=wait_enter, daemon=True).start()

    while not stop.is_set():
        vals_sys = mb.read_regs(REG_SYS_ADDR, 8)
        vals_az  = mb.read_regs(REG_AZ_STATUS, 10)
        vals_el  = mb.read_regs(REG_EL_STATUS, 10)

        clear()
        print("  Live Monitor  (press Enter to stop)\n")

        if vals_sys:
            uptime = (vals_sys[2] << 16) | vals_sys[3]
            sw     = vals_sys[6]
            fan    = vals_sys[7]
            print(f"  Uptime : {fmt_uptime(uptime)}   "
                  f"Fan: {'ON' if fan else 'off'}   "
                  f"SW: {sw:#06x}")
        else:
            print("  System: no response")

        for name, vals in [("AZ", vals_az), ("EL", vals_el)]:
            if vals:
                status = vals[0]
                pos    = to_signed32((vals[1] << 16) | vals[2])
                deg_r  = (vals[3] << 16) | vals[4]
                deg    = struct.unpack('>f', struct.pack('>I', deg_r))[0]
                err    = vals[9]
                print(f"  {name}     : {axis_status_str(status):<30}  "
                      f"pos={pos}  {deg:.4f}°  err={err:#04x}")
            else:
                print(f"  {name}: no response")

        time.sleep(1.0)

def action_query_gps(mb: ModbusClient):
    print("\n[ GPS Status ]\n")
    vals = mb.read_regs(REG_GPS_STATUS, 3)
    if vals is None:
        print("  ERROR: No response from board")
        return
    gps_status = vals[0]
    hhmm       = vals[1]
    ss         = vals[2]
    status_str = {0: "No fix", 1: "Fix", 2: "Disciplined"}.get(gps_status, "Unknown")
    print(f"  GPS status    : {status_str}")
    print(f"  UTC time      : {hhmm // 100:02d}:{hhmm % 100:02d}:{ss:02d}")

def action_motor_control(mb: ModbusClient):
    print("\n[ Motor Control ]\n")

    axis = input("  Axis (1=AZ, 2=EL): ").strip()
    if axis == '1':
        en_reg, stop_reg, freq_reg, dir_reg, err_reg, lim_bits, name = \
            REG_AZ_CMD_ENABLE, REG_AZ_CMD_STOP, REG_AZ_PWM_FREQ, REG_AZ_PWM_DIR, REG_AZ_ERROR, (0, 1), "AZ"
    elif axis == '2':
        en_reg, stop_reg, freq_reg, dir_reg, err_reg, lim_bits, name = \
            REG_EL_CMD_ENABLE, REG_EL_CMD_STOP, REG_EL_PWM_FREQ, REG_EL_PWM_DIR, REG_EL_ERROR, (2, 3), "EL"
    else:
        print("  Invalid axis.")
        return

    current_freq = mb.read_reg(freq_reg) or 0
    current_dir  = mb.read_reg(dir_reg)  or 0
    dir_str      = lambda d: "CCW" if d else "CW"

    print(f"\n  {name} axis  |  Freq: {current_freq} Hz  |  Dir: {dir_str(current_dir)}\n")
    print("  Commands: e=enable  d=disable  s=stop  f <hz>=freq  cw/ccw=dir  q=quit\n")

    # --- Background encoder display thread ---
    import threading
    stop_display = threading.Event()

    def encoder_display():
        prev_key = None
        while not stop_display.is_set():
            az    = mb.read_int32(REG_AZ_POS_HI)
            el    = mb.read_int32(REG_EL_POS_HI)
            az_z  = mb.read_reg(REG_AZ_Z_COUNT)
            el_z  = mb.read_reg(REG_EL_Z_COUNT)
            lim   = mb.read_reg(REG_LIMIT_SW)
            err   = mb.read_reg(err_reg)
            freq  = mb.read_reg(freq_reg)
            drv   = mb.read_reg(dir_reg)
            if az is not None and el is not None:
                lim1 = 'TRIG' if (lim is not None and lim & (1 << lim_bits[0])) else 'open'
                lim2 = 'TRIG' if (lim is not None and lim & (1 << lim_bits[1])) else 'open'
                err_str  = ERROR_NAMES.get(err, f'{err:#04x}') if err else 'none'
                freq_str = f'{freq}Hz' if freq is not None else '?'
                dir_str2 = dir_str(drv) if drv is not None else '?'
                line = (f"  ENC AZ: {az:>12}  EL: {el:>12}  Z_AZ: {az_z or 0:>5}  Z_EL: {el_z or 0:>5}  "
                        f"{name}_LIM1: {lim1}  {name}_LIM2: {lim2}  {name}_ERR: {err_str:<16}  "
                        f"{name}_FREQ: {freq_str:<7}  {name}_DIR: {dir_str2}")
                # A momentary switch can spring back before the next poll, and
                # errors latch until the next home/move — so overwriting a
                # single line (\r) loses exactly the moment we need to see.
                # Print a fresh line only when lim/freq/dir/err/Z-count
                # actually change, giving a persistent log of which bit
                # tripped, when, and whether firmware actually cut the pulse
                # rate to 0 or the motor stalled while freq stayed nonzero
                # (mechanical, not a firmware-commanded stop). Z-count is
                # included so each encoder revolution's ENC value is logged —
                # the delta between consecutive Z ticks is the ground-truth
                # check for missed/extra quadrature counts.
                key = (lim1, lim2, err, freq, drv, az_z, el_z)
                if key != prev_key:
                    print(f"\n{line}", flush=True)
                    prev_key = key
                else:
                    print(f"\r{line}", end='', flush=True)
            stop_display.wait(0.2)

    t = threading.Thread(target=encoder_display, daemon=True)
    t.start()

    print()  # blank line so encoder display has its own line

    while True:
        prompt = f"\n  [{name} {current_freq}Hz {dir_str(current_dir)}] > "
        cmd = input(prompt).strip().lower()

        if cmd == 'e':
            time.sleep(0.05)
            ok = mb.write_reg(en_reg, 1)
            print(f"  {'Enabled' if ok else 'ERROR'}")

        elif cmd == 'd':
            time.sleep(0.05)
            ok = mb.write_reg(en_reg, 0)
            print(f"  {'Disabled' if ok else 'ERROR'}")

        elif cmd == 's':
            time.sleep(0.05)
            ok = mb.write_reg(stop_reg, 1)
            print(f"  {'Stopped' if ok else 'ERROR'}")

        elif cmd.startswith('f '):
            try:
                hz = int(cmd[2:].strip())
                if 1 <= hz <= 10000:
                    time.sleep(0.05)
                    ok = mb.write_reg(freq_reg, hz)
                    if ok:
                        current_freq = hz
                        print(f"  Frequency set to {hz} Hz")
                    else:
                        print("  ERROR")
                else:
                    print("  Frequency must be 1-10000 Hz")
            except ValueError:
                print("  Invalid frequency")

        elif cmd == 'cw':
            time.sleep(0.05)
            ok = mb.write_reg(dir_reg, 0)
            if ok:
                current_dir = 0
                print("  Direction: CW")
            else:
                print("  ERROR")

        elif cmd == 'ccw':
            time.sleep(0.05)
            ok = mb.write_reg(dir_reg, 1)
            if ok:
                current_dir = 1
                print("  Direction: CCW")
            else:
                print("  ERROR")

        elif cmd == 'q':
            stop_display.set()
            t.join(timeout=1.0)
            mb.write_reg(en_reg, 0)
            print("\n  Motor disabled. Exiting.")
            break

        else:
            print("  Unknown command. Use: e, d, s, f <hz>, cw, ccw, q")


def _home_and_wait(mb: ModbusClient, home_reg: int, status_reg: int, error_reg: int,
                    pos_reg: int | None = None) -> bool:
    """Write CMD_HOME and poll until HOMED/ERROR/timeout. Returns True on
    success. If pos_reg is given, also prints a live status/position line —
    pass None for a quieter, one-line-per-outcome caller (e.g. the
    calibration wizard, which already has its own step numbering)."""
    mb.write_reg(home_reg, 1)
    for _ in range(3100):  # ~310s — comfortably past firmware's own 5min timeout
        time.sleep(0.1)
        status = mb.read_reg(status_reg)
        if status is None:
            continue
        if pos_reg is not None:
            pos = mb.read_int32(pos_reg)
            print(f"\r  {axis_status_str(status):<40}  pos={pos}   ", end='', flush=True)
        if status & (1 << 4):   # HOMED
            if pos_reg is not None:
                print("\n\n  Homed.")
            return True
        if status & (1 << 7):   # ERROR
            err = mb.read_reg(error_reg)
            print(f"\n  ERROR: Homing failed ({ERROR_NAMES.get(err, err)})")
            return False
    print("\n  ERROR: Homing did not complete in time.")
    return False


# -----------------------------------------------------------------------
# LIMIT2 approach (calibration wizard)
#
# Firmware's CMD_HOME only seeks LIMIT1, using a 3-phase seek/back-off/creep
# sequence (see AZ/EL_HomingTick in pwm_test.c) so the trigger point is
# repeatable rather than whatever overtravel a raw approach leaves it at.
# LIMIT2 has no firmware equivalent — the calibration wizard jogs there
# manually — so this reproduces the same back-off-then-creep shape here,
# using the same speed fractions/margins firmware uses for LIMIT1.
# -----------------------------------------------------------------------
HOME_SEEK_RPM_FRACTION  = 0.2
HOME_SEEK_RPM_MIN       = 0.05
HOME_CREEP_RPM_FRACTION = 0.05
HOME_CREEP_RPM_MIN      = 0.02
HOME_BACKOFF_EXTRA_S    = 0.3

def _seek_rpm(max_rpm: float) -> float:
    return max(max_rpm * HOME_SEEK_RPM_FRACTION, HOME_SEEK_RPM_MIN)

def _creep_rpm(max_rpm: float) -> float:
    return max(max_rpm * HOME_CREEP_RPM_FRACTION, HOME_CREEP_RPM_MIN)

def _rpm_to_hz(mb: ModbusClient, gear_reg: int, ppr_reg: int, rpm: float) -> int:
    """Mirrors firmware's AZ/EL_RPM_to_Hz (pwm_test.c) so speeds commanded
    here match what firmware itself would send for the same output RPM."""
    gear_ratio = mb.read_int32(gear_reg) or 20
    ppr        = mb.read_reg(ppr_reg) or 400
    hz = (rpm / 60.0) * gear_ratio * ppr
    return int(max(1.0, min(10000.0, hz)) + 0.5)

def _approach_limit2(mb: ModbusClient, freq_reg: int, dir_reg: int, pos_reg: int,
                      max_rpm_reg: int, gear_reg: int, ppr_reg: int,
                      lim_bit: int, name: str) -> int | None:
    """Called the instant a manual jog trips LIMIT2. Backs off until the
    switch releases (plus a small margin, same as firmware's
    HOME_BACKOFF_EXTRA_MS), then creeps back in slowly so the trigger point
    is repeatable instead of overtravel-dependent."""
    max_rpm  = mb.read_float(max_rpm_reg) or 1.5
    seek_hz  = _rpm_to_hz(mb, gear_reg, ppr_reg, _seek_rpm(max_rpm))
    creep_hz = _rpm_to_hz(mb, gear_reg, ppr_reg, _creep_rpm(max_rpm))

    print(f"\n\n  {name} LIMIT2 triggered — backing off...")
    mb.write_reg(dir_reg, 0)   # CW — back away from LIMIT2
    mb.write_reg(freq_reg, seek_hz)

    released_since = None
    while True:
        time.sleep(0.1)
        lim = mb.read_reg(REG_LIMIT_SW)
        pos = mb.read_int32(pos_reg)
        print(f"\r  Backing off...  pos={pos}   ", end='', flush=True)
        if lim is None:
            continue
        if lim & (1 << lim_bit):
            released_since = None  # still triggered, or bounced back onto it
        elif released_since is None:
            released_since = time.monotonic()
        elif time.monotonic() - released_since >= HOME_BACKOFF_EXTRA_S:
            break

    print(f"\n  {name} creeping back in to LIMIT2...")
    mb.write_reg(dir_reg, 1)   # CCW — creep back toward LIMIT2
    mb.write_reg(freq_reg, creep_hz)

    while True:
        time.sleep(0.1)
        lim = mb.read_reg(REG_LIMIT_SW)
        pos = mb.read_int32(pos_reg)
        print(f"\r  Creeping in...  pos={pos}   ", end='', flush=True)
        if lim is not None and lim & (1 << lim_bit):
            break

    mb.write_reg(freq_reg, 0)
    lim2_pos = mb.read_int32(pos_reg)
    print(f"\n  {name} LIMIT2 triggered at pos={lim2_pos}.")
    return lim2_pos


def action_home_axis(mb: ModbusClient):
    """Home a single axis to LIMIT1 only — no angle prompts, no jogging to
    LIMIT2. For the full two-point calibration, use 'Calibrate axis' instead."""
    print("\n[ Home Axis ]\n")

    axis = input("  Axis to home (1=AZ, 2=EL): ").strip()
    if axis == '1':
        en_reg, home_reg, status_reg, error_reg, pos_reg, name = \
            REG_AZ_CMD_ENABLE, REG_AZ_CMD_HOME, REG_AZ_STATUS, REG_AZ_ERROR, REG_AZ_POS_HI, "AZ"
    elif axis == '2':
        en_reg, home_reg, status_reg, error_reg, pos_reg, name = \
            REG_EL_CMD_ENABLE, REG_EL_CMD_HOME, REG_EL_STATUS, REG_EL_ERROR, REG_EL_POS_HI, "EL"
    else:
        print("  Invalid axis.")
        return

    print(f"  Enabling {name} driver...")
    mb.write_reg(en_reg, 1)
    time.sleep(0.1)

    print(f"  Homing {name} to LIMIT1 (CW)...\n")
    _home_and_wait(mb, home_reg, status_reg, error_reg, pos_reg)


def action_flash_test(mb: ModbusClient):
    print("\n[ SPI Flash Write/Read Test ]\n")

    raw = input("  Enter a 16-bit hex value to write (e.g. 0xABCD): ").strip()
    try:
        value = int(raw, 16) & 0xFFFF
    except ValueError:
        print("  Invalid input.")
        return

    print(f"\n  Value to write: {value:#06x}\n")

    # Step 1: Erase sector
    print("  [1/3] Erasing sector...")
    mb.write_reg(REG_FLASH_DATA, value)
    time.sleep(0.05)
    mb.write_reg(REG_FLASH_CMD, FLASH_CMD_ERASE)
    for i in range(20):
        time.sleep(0.1)
        status = mb.read_reg(REG_FLASH_STATUS)
        if status in (2, 3):
            break
    print(f"        Status: {FLASH_STATUS.get(status, '?')}")
    if status != 2:
        print("  Erase failed — aborting.")
        return

    # Step 2: Write value
    print("  [2/3] Writing value to flash...")
    mb.write_reg(REG_FLASH_DATA, value)
    time.sleep(0.05)
    mb.write_reg(REG_FLASH_CMD, FLASH_CMD_WRITE)
    for i in range(10):
        time.sleep(0.1)
        status = mb.read_reg(REG_FLASH_STATUS)
        if status in (2, 3):
            break
    print(f"        Status: {FLASH_STATUS.get(status, '?')}")
    if status != 2:
        print("  Write failed — aborting.")
        return

    # Step 3: Read back
    print("  [3/3] Reading back from flash...")
    mb.write_reg(REG_FLASH_CMD, FLASH_CMD_READ)
    time.sleep(0.1)
    status   = mb.read_reg(REG_FLASH_STATUS)
    readback = mb.read_reg(REG_FLASH_DATA)
    print(f"        Status: {FLASH_STATUS.get(status, '?')}")
    print(f"        Read back: {readback:#06x}")

    print()
    if readback == value:
        print(f"  PASS — wrote {value:#06x}, read back {readback:#06x} ✓")
    else:
        print(f"  FAIL — wrote {value:#06x}, read back {readback:#06x} ✗")


def action_limit_switches(mb: ModbusClient):
    print("\n[ Limit Switch Monitor — press Enter to stop ]\n")
    print("  Bit 0 = AZ_LIM1 (LIMIT2_SW1)")
    print("  Bit 1 = AZ_LIM2 (LIMIT2_SW2)")
    print("  Bit 2 = EL_LIM1 (LIMIT1_SW1)")
    print("  Bit 3 = EL_LIM2 (LIMIT1_SW2)\n")

    import threading
    stop = threading.Event()

    def wait_enter():
        input()
        stop.set()

    threading.Thread(target=wait_enter, daemon=True).start()

    prev = None
    while not stop.is_set():
        lim = mb.read_reg(REG_LIMIT_SW)
        if lim is not None and lim != prev:
            az_pos = mb.read_int32(REG_AZ_POS_HI)
            el_pos = mb.read_int32(REG_EL_POS_HI)
            az1 = '▣ TRIGGERED' if lim & (1<<0) else '□ open'
            az2 = '▣ TRIGGERED' if lim & (1<<1) else '□ open'
            el1 = '▣ TRIGGERED' if lim & (1<<2) else '□ open'
            el2 = '▣ TRIGGERED' if lim & (1<<3) else '□ open'
            print(f"  AZ_LIM1: {az1}   AZ_LIM2: {az2}   AZ pos: {az_pos}")
            print(f"  EL_LIM1: {el1}   EL_LIM2: {el2}   EL pos: {el_pos}")
            print()
            prev = lim
        time.sleep(0.1)


def action_encoder_monitor(mb: ModbusClient):
    print("\n[ Encoder Monitor — press Enter to stop ]\n")
    import threading
    stop = threading.Event()

    def wait_enter():
        input()
        stop.set()

    threading.Thread(target=wait_enter, daemon=True).start()

    while not stop.is_set():
        az = mb.read_int32(REG_AZ_POS_HI)
        el = mb.read_int32(REG_EL_POS_HI)
        if az is not None and el is not None:
            print(f"  AZ: {az:>12}  EL: {el:>12}", end='\r')
        time.sleep(0.1)

    print()


def action_calibrate_axis(mb: ModbusClient):
    print("\n[ Axis Calibration ]\n")
    print("  Homes the axis to LIMIT1 (the CW-side switch), zeroing the")
    print("  encoder there, then has you jog to LIMIT2 while you measure the")
    print("  real-world angle at each end with an external instrument.")
    print("  LIMIT2 is detected automatically: on contact the axis backs off")
    print("  and creeps back in slowly for a repeatable trigger point, the")
    print("  same way CMD_HOME approaches LIMIT1.")
    print("  Calibration is volatile — repeat after every power cycle.\n")

    axis = input("  Axis to calibrate (1=AZ, 2=EL): ").strip()
    if axis == '1':
        (en_reg, home_reg, freq_reg, dir_reg, status_reg, error_reg,
         pos_reg, lim2_pos_reg, lim1_angle_reg, lim2_angle_reg,
         gear_reg, ppr_reg, max_rpm_reg, lim_bit, name) = (
            REG_AZ_CMD_ENABLE, REG_AZ_CMD_HOME, REG_AZ_PWM_FREQ, REG_AZ_PWM_DIR,
            REG_AZ_STATUS, REG_AZ_ERROR, REG_AZ_POS_HI, REG_AZ_LIM2_POS_HI,
            REG_AZ_LIM1_ANGLE_HI, REG_AZ_LIM2_ANGLE_HI,
            REG_AZ_GEAR_RATIO_HI, REG_AZ_DRIVER_PPR, REG_AZ_MAX_RPM_HI, 1, "AZ")
    elif axis == '2':
        (en_reg, home_reg, freq_reg, dir_reg, status_reg, error_reg,
         pos_reg, lim2_pos_reg, lim1_angle_reg, lim2_angle_reg,
         gear_reg, ppr_reg, max_rpm_reg, lim_bit, name) = (
            REG_EL_CMD_ENABLE, REG_EL_CMD_HOME, REG_EL_PWM_FREQ, REG_EL_PWM_DIR,
            REG_EL_STATUS, REG_EL_ERROR, REG_EL_POS_HI, REG_EL_LIM2_POS_HI,
            REG_EL_LIM1_ANGLE_HI, REG_EL_LIM2_ANGLE_HI,
            REG_EL_GEAR_RATIO_HI, REG_EL_DRIVER_PPR, REG_EL_MAX_RPM_HI, 3, "EL")
    else:
        print("  Invalid axis.")
        return

    print(f"\n  Step 1/5: Enabling {name} driver...")
    mb.write_reg(en_reg, 1)
    time.sleep(0.1)

    print("  Step 2/5: Homing to LIMIT1 (CW)...")
    if not _home_and_wait(mb, home_reg, status_reg, error_reg):
        return
    print("  Homed. Encoder zeroed at LIMIT1.")

    try:
        lim1_angle = float(input("\n  Enter measured angle at LIMIT1 (deg): ").strip())
    except ValueError:
        print("  Invalid angle.")
        return

    print("\n  Step 3/5: Jog toward LIMIT2 manually.")
    print("  Commands: cw/ccw=dir  f <hz>=freq  s=stop jogging  q=abort")
    print("  LIMIT2 is picked up automatically — no need to stop it yourself.\n")

    mb.write_reg(dir_reg, 1)  # CCW — away from LIMIT1, the only direction available
    print()

    # A blocking input() here would either miss LIMIT2 entirely (it only gets
    # checked between commands) or, if run on a background thread, leave a
    # second reader competing with the LIMIT2-angle prompt below for the same
    # stdin — so poll for both the switch and a typed command in one loop.
    lim2_pos = None
    while True:
        lim = mb.read_reg(REG_LIMIT_SW)
        pos = mb.read_int32(pos_reg)
        if lim is not None and pos is not None:
            trig = '▣ TRIGGERED' if lim & (1 << lim_bit) else '□ open'
            print(f"\r  LIMIT2: {trig}   pos={pos:>12}   ", end='', flush=True)
            if lim & (1 << lim_bit):
                mb.write_reg(freq_reg, 0)
                lim2_pos = _approach_limit2(mb, freq_reg, dir_reg, pos_reg,
                                             max_rpm_reg, gear_reg, ppr_reg,
                                             lim_bit, name)
                break

        ready, _, _ = select.select([sys.stdin], [], [], 0.1)
        if not ready:
            continue
        cmd = sys.stdin.readline().strip().lower()

        if cmd == 'cw':
            mb.write_reg(dir_reg, 0)
        elif cmd == 'ccw':
            mb.write_reg(dir_reg, 1)
        elif cmd.startswith('f '):
            try:
                hz = int(cmd[2:].strip())
                if 1 <= hz <= 10000:
                    mb.write_reg(freq_reg, hz)
                else:
                    print("  Frequency must be 1-10000 Hz")
            except ValueError:
                print("  Invalid frequency")
        elif cmd == 's':
            mb.write_reg(freq_reg, 0)
        elif cmd == 'q':
            mb.write_reg(freq_reg, 0)
            print("\n  Aborted — LIMIT2 not reached.")
            return
        else:
            print("  Unknown command. Use: cw, ccw, f <hz>, s, q")

    if lim2_pos is None:
        print("  ERROR: Could not determine LIMIT2 position.")
        return
    print(f"\n  Encoder count at LIMIT2: {lim2_pos}")

    try:
        lim2_angle = float(input("  Enter measured angle at LIMIT2 (deg): ").strip())
    except ValueError:
        print("  Invalid angle.")
        return

    print("\n  Step 4/5: Writing calibration registers...")
    ok  = mb.write_float(lim1_angle_reg, lim1_angle)
    ok &= mb.write_float(lim2_angle_reg, lim2_angle)
    ok &= mb.write_int32(lim2_pos_reg, lim2_pos)
    print("  Done." if ok else "  ERROR writing calibration registers.")

    print("\n  Step 5/5: Calibration complete.")
    print("  NOTE: calibration is volatile — repeat this procedure after every power cycle.")


def action_move_to_angle(mb: ModbusClient):
    print("\n[ Move to Angle ]\n")

    axis = input("  Axis (1=AZ, 2=EL): ").strip()
    if axis == '1':
        (en_reg, mode_reg, pos_reg, status_reg, error_reg, deg_reg,
         stop_reg, max_rpm_reg, gear_reg, ppr_reg, name) = (
            REG_AZ_CMD_ENABLE, REG_AZ_CMD_MODE, REG_AZ_CMD_POS_HI, REG_AZ_STATUS,
            REG_AZ_ERROR, REG_AZ_POS_DEG_HI, REG_AZ_CMD_STOP, REG_AZ_MAX_RPM_HI,
            REG_AZ_GEAR_RATIO_HI, REG_AZ_DRIVER_PPR, "AZ")
    elif axis == '2':
        (en_reg, mode_reg, pos_reg, status_reg, error_reg, deg_reg,
         stop_reg, max_rpm_reg, gear_reg, ppr_reg, name) = (
            REG_EL_CMD_ENABLE, REG_EL_CMD_MODE, REG_EL_CMD_POS_HI, REG_EL_STATUS,
            REG_EL_ERROR, REG_EL_POS_DEG_HI, REG_EL_CMD_STOP, REG_EL_MAX_RPM_HI,
            REG_EL_GEAR_RATIO_HI, REG_EL_DRIVER_PPR, "EL")
    else:
        print("  Invalid axis.")
        return

    try:
        target = float(input(f"  Target angle for {name} (deg): ").strip())
    except ValueError:
        print("  Invalid angle.")
        return

    # Temporarily set MAX_RPM so the firmware move loop runs at MOVE_FREQ_HZ
    # (see MOVE_FREQ_HZ comment). RPM_to_Hz = (rpm/60) * gear * ppr, so the
    # rpm that yields MOVE_FREQ_HZ is MOVE_FREQ_HZ * 60 / (gear * ppr).
    gear = mb.read_int32(gear_reg) or 20
    ppr  = mb.read_reg(ppr_reg) or 400
    orig_max_rpm = mb.read_float(max_rpm_reg)
    if orig_max_rpm is None:
        print("  ERROR reading current MAX_RPM — aborting.")
        return
    slow_rpm = MOVE_FREQ_HZ * 60.0 / (gear * ppr)

    mb.write_reg(en_reg, 1)
    time.sleep(0.05)
    mb.write_reg(mode_reg, 0)  # position mode
    time.sleep(0.05)
    if not mb.write_float(max_rpm_reg, slow_rpm):
        print("  ERROR setting move speed.")
        return
    time.sleep(0.05)

    try:
        if not mb.write_float(pos_reg, target):
            print("  ERROR sending move command.")
            return

        print(f"\n  Moving {name} to {target}° at {MOVE_FREQ_HZ} Hz"
              f"  (press Enter to abort)\n")

        import threading
        stop = threading.Event()

        def wait_enter():
            input()
            stop.set()

        threading.Thread(target=wait_enter, daemon=True).start()

        while not stop.is_set():
            status = mb.read_reg(status_reg)
            deg    = mb.read_float(deg_reg)
            err    = mb.read_reg(error_reg)
            if status is not None and deg is not None:
                print(f"\r  {axis_status_str(status):<40}  pos={deg:.4f}°  err={err:#04x}   ",
                      end='', flush=True)
                if status & (1 << 2):   # AT_TARGET
                    print("\n\n  Arrived.")
                    break
                if status & (1 << 7):   # ERROR
                    print(f"\n\n  ERROR: {ERROR_NAMES.get(err, err)}")
                    break
            time.sleep(0.1)

        if stop.is_set():
            # Aborted mid-move: stop the axis before the finally block restores
            # MAX_RPM, or the move loop would pick up the faster speed and
            # finish the travel at full (lossy) rate.
            mb.write_reg(stop_reg, 1)
            print("\n\n  Aborted.")
    finally:
        mb.write_float(max_rpm_reg, orig_max_rpm)


# -----------------------------------------------------------------------
# Menu
# -----------------------------------------------------------------------
MENU = [
    ("Query system registers",          action_query_system),
    ("Query AZ axis status",            lambda mb: action_query_axis(
        mb, "AZ", REG_AZ_STATUS, REG_AZ_POS_HI, REG_AZ_POS_DEG_HI,
        REG_AZ_RPM_HI, REG_AZ_PID_OUT_HI, REG_AZ_ERROR)),
    ("Query EL axis status",            lambda mb: action_query_axis(
        mb, "EL", REG_EL_STATUS, REG_EL_POS_HI, REG_EL_POS_DEG_HI,
        REG_EL_RPM_HI, REG_EL_PID_OUT_HI, REG_EL_ERROR)),
    ("Query GPS status",                action_query_gps),
    ("Fan ON",                          action_fan_on),
    ("Fan OFF",                         action_fan_off),
    ("Motor control",                   action_motor_control),
    ("Home axis",                       action_home_axis),
    ("Calibrate axis",                  action_calibrate_axis),
    ("Move to angle",                   action_move_to_angle),
    ("Encoder monitor",                 action_encoder_monitor),
    ("Limit switch monitor",            action_limit_switches),
    ("SPI flash write/read test",       action_flash_test),
    ("Live monitor (all, 1s refresh)",  action_monitor),
    ("Quit",                            None),
]

def run_menu(mb: ModbusClient, port: str):
    while True:
        clear()
        header(mb, port)
        print()
        for i, (label, _) in enumerate(MENU, 1):
            print(f"  {i}.  {label}")
        print()

        choice = input("  Select: ").strip()

        if not choice.isdigit():
            continue

        idx = int(choice) - 1
        if idx < 0 or idx >= len(MENU):
            continue

        label, action = MENU[idx]

        if action is None:
            print("\n  Goodbye.\n")
            break

        print()
        action(mb)
        print()
        input("  Press Enter to return to menu...")

# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Axis Controller interactive Modbus menu")
    parser.add_argument("--port",  required=True,       help="Serial port")
    parser.add_argument("--addr",  type=int, default=1, help="Board node address (default: 1)")
    parser.add_argument("--baud",  type=int, default=115200)
    args = parser.parse_args()

    try:
        mb = ModbusClient(args.port, args.baud, args.addr)
    except serial.SerialException as e:
        print(f"ERROR: Could not open port: {e}")
        sys.exit(1)

    try:
        run_menu(mb, args.port)
    except KeyboardInterrupt:
        print("\n  Interrupted.")
    finally:
        mb.close()

if __name__ == "__main__":
    main()
