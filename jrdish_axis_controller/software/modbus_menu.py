#!/usr/bin/env python3
"""
modbus_menu.py
Interactive Modbus RTU menu for Axis Controller board Rev 1.1

Usage:
    python3 modbus_menu.py --port /dev/tty.usbserial-XXXX --addr 1
"""

import serial
import struct
import time
import argparse
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

REG_AZ_CMD_ENABLE   = 0x0030
REG_AZ_CMD_MODE     = 0x0031
REG_AZ_CMD_STOP     = 0x0037

REG_AZ_STATUS       = 0x0040
REG_AZ_POS_HI       = 0x0041
REG_AZ_POS_DEG_HI   = 0x0043
REG_AZ_RPM_HI       = 0x0045
REG_AZ_PID_OUT_HI   = 0x0047
REG_AZ_ERROR        = 0x0049

REG_EL_CMD_ENABLE   = 0x0090
REG_EL_CMD_MODE     = 0x0091
REG_EL_CMD_STOP     = 0x0097

REG_EL_STATUS       = 0x00A0
REG_EL_POS_HI       = 0x00A1
REG_EL_POS_DEG_HI   = 0x00A3
REG_EL_RPM_HI       = 0x00A5
REG_EL_PID_OUT_HI   = 0x00A7
REG_EL_ERROR        = 0x00A9

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
        time.sleep(0.1)

    def close(self):
        self.ser.close()

    def _transact(self, frame: bytes, expected: int) -> bytes | None:
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
        return ((r[0] << 16) | r[1]) if r else None

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
    pos    = (vals[1] << 16) | vals[2]
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
                pos    = (vals[1] << 16) | vals[2]
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
        en_reg, stop_reg, freq_reg, dir_reg, name = \
            REG_AZ_CMD_ENABLE, REG_AZ_CMD_STOP, REG_AZ_PWM_FREQ, REG_AZ_PWM_DIR, "AZ"
    elif axis == '2':
        en_reg, stop_reg, freq_reg, dir_reg, name = \
            REG_EL_CMD_ENABLE, REG_EL_CMD_STOP, REG_EL_PWM_FREQ, REG_EL_PWM_DIR, "EL"
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
        while not stop_display.is_set():
            az    = mb.read_int32(REG_AZ_POS_HI)
            el    = mb.read_int32(REG_EL_POS_HI)
            az_z  = mb.read_reg(REG_AZ_Z_COUNT)
            el_z  = mb.read_reg(REG_EL_Z_COUNT)
            if az is not None and el is not None:
                print(f"\r  ENC AZ: {az:>12}  EL: {el:>12}  Z_AZ: {az_z or 0:>5}  Z_EL: {el_z or 0:>5}   ",
                      end='', flush=True)
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
            az1 = '▣ TRIGGERED' if lim & (1<<0) else '□ open'
            az2 = '▣ TRIGGERED' if lim & (1<<1) else '□ open'
            el1 = '▣ TRIGGERED' if lim & (1<<2) else '□ open'
            el2 = '▣ TRIGGERED' if lim & (1<<3) else '□ open'
            print(f"  AZ_LIM1: {az1}   AZ_LIM2: {az2}")
            print(f"  EL_LIM1: {el1}   EL_LIM2: {el2}")
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
