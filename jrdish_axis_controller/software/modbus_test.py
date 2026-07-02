#!/usr/bin/env python3
"""
modbus_test.py
Modbus RTU test script for Axis Controller board Rev 1.1

Tests: system registers, user switches, fan control, axis status reads.

Usage:
    python3 modbus_test.py --port /dev/tty.usbserial-XXXX --addr 1
    python3 modbus_test.py --port /dev/ttyUSB0 --addr 1 --baud 115200
"""

import serial
import struct
import time
import argparse
import sys

# -----------------------------------------------------------------------
# Modbus register addresses
# -----------------------------------------------------------------------
REG_SYS_ADDR        = 0x0000
REG_SYS_FW_VER      = 0x0001
REG_SYS_UPTIME_HI   = 0x0002
REG_SYS_UPTIME_LO   = 0x0003
REG_SYS_RESET       = 0x0004
REG_SYS_STATUS      = 0x0005
REG_SYS_USER_SW     = 0x0006
REG_SYS_FAN         = 0x0007

REG_AZ_STATUS       = 0x0040
REG_AZ_POS_HI       = 0x0041
REG_AZ_POS_LO       = 0x0042
REG_AZ_POS_DEG_HI   = 0x0043
REG_AZ_POS_DEG_LO   = 0x0044
REG_AZ_RPM_HI       = 0x0045
REG_AZ_RPM_LO       = 0x0046
REG_AZ_ERROR        = 0x0049

REG_EL_STATUS       = 0x00A0
REG_EL_POS_HI       = 0x00A1
REG_EL_POS_LO       = 0x00A2
REG_EL_POS_DEG_HI   = 0x00A3
REG_EL_POS_DEG_LO   = 0x00A4
REG_EL_RPM_HI       = 0x00A5
REG_EL_RPM_LO       = 0x00A6
REG_EL_ERROR        = 0x00A9

BROADCAST_ADDR      = 0xFF
RESET_KEY           = 0xDEAD

# -----------------------------------------------------------------------
# CRC-16 (Modbus)
# -----------------------------------------------------------------------
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def append_crc(data: bytes) -> bytes:
    c = crc16(data)
    return data + bytes([c & 0xFF, (c >> 8) & 0xFF])

def check_crc(data: bytes) -> bool:
    if len(data) < 4:
        return False
    payload = data[:-2]
    rx_crc  = data[-2] | (data[-1] << 8)
    return crc16(payload) == rx_crc

# -----------------------------------------------------------------------
# Modbus client
# -----------------------------------------------------------------------
class ModbusClient:
    def __init__(self, port, baud=115200, addr=1, timeout=0.5):
        self.addr = addr
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        time.sleep(0.1)  # let adapter settle

    def close(self):
        self.ser.close()

    def _send_recv(self, frame: bytes, expected_len: int) -> bytes | None:
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        resp = self.ser.read(expected_len)
        if len(resp) < 4:
            return None
        if not check_crc(resp):
            print(f"  CRC ERROR: {resp.hex()}")
            return None
        if resp[0] != self.addr:
            print(f"  ADDRESS MISMATCH: got {resp[0]:#04x}")
            return None
        if resp[1] & 0x80:
            print(f"  EXCEPTION: FC={resp[1]:#04x} code={resp[2]:#04x}")
            return None
        return resp

    def read_regs(self, start: int, qty: int) -> list[int] | None:
        frame = append_crc(struct.pack('>BBHH', self.addr, 0x03, start, qty))
        expected = 5 + qty * 2  # addr + fc + bytecount + data + 2 crc
        resp = self._send_recv(frame, expected)
        if resp is None:
            return None
        values = []
        for i in range(qty):
            val = (resp[3 + i*2] << 8) | resp[4 + i*2]
            values.append(val)
        return values

    def write_reg(self, reg: int, value: int) -> bool:
        frame = append_crc(struct.pack('>BBHH', self.addr, 0x06, reg, value))
        resp = self._send_recv(frame, 8)
        return resp is not None

    def write_regs(self, start: int, values: list[int]) -> bool:
        qty = len(values)
        data = struct.pack('>BBHHB', self.addr, 0x10, start, qty, qty * 2)
        for v in values:
            data += struct.pack('>H', v)
        frame = append_crc(data)
        resp = self._send_recv(frame, 8)
        return resp is not None

    def read_reg(self, reg: int) -> int | None:
        result = self.read_regs(reg, 1)
        return result[0] if result else None

    def read_float(self, reg_hi: int) -> float | None:
        result = self.read_regs(reg_hi, 2)
        if result is None:
            return None
        raw = (result[0] << 16) | result[1]
        return struct.unpack('>f', struct.pack('>I', raw))[0]

    def read_int32(self, reg_hi: int) -> int | None:
        result = self.read_regs(reg_hi, 2)
        if result is None:
            return None
        return (result[0] << 16) | result[1]

# -----------------------------------------------------------------------
# Test helpers
# -----------------------------------------------------------------------
PASS = "\033[92mPASS\033[0m"
FAIL = "\033[91mFAIL\033[0m"

def check(label, value, expected=None):
    if value is None:
        print(f"  {FAIL}  {label}: no response")
        return False
    if expected is not None and value != expected:
        print(f"  {FAIL}  {label}: got {value}, expected {expected}")
        return False
    print(f"  {PASS}  {label}: {value}")
    return True

# -----------------------------------------------------------------------
# Test suites
# -----------------------------------------------------------------------
def test_system(mb: ModbusClient):
    print("\n--- System Registers ---")
    vals = mb.read_regs(REG_SYS_ADDR, 8)
    if vals is None:
        print(f"  {FAIL}  Could not read system registers")
        return

    addr    = vals[0]
    fw_ver  = vals[1]
    uptime  = (vals[2] << 16) | vals[3]
    status  = vals[5]
    sw      = vals[6]
    fan     = vals[7]

    print(f"  {PASS}  Node address : {addr}")
    print(f"  {PASS}  FW version   : {fw_ver >> 8}.{fw_ver & 0xFF}")
    print(f"  {PASS}  Uptime       : {uptime} ms ({uptime/1000:.1f}s)")
    print(f"  {PASS}  SYS_STATUS   : {status:#06x}")
    print(f"  {PASS}  USER_SW      : {sw:#06x} (SW1={sw&1} SW2={(sw>>1)&1} SW3={(sw>>2)&1} SW4={(sw>>3)&1})")
    print(f"  {PASS}  FAN          : {'ON' if fan else 'OFF'}")

def test_fan(mb: ModbusClient):
    print("\n--- Fan Control ---")

    print("  Turning fan ON...")
    ok = mb.write_reg(REG_SYS_FAN, 1)
    if not ok:
        print(f"  {FAIL}  Write failed")
        return
    time.sleep(0.2)
    val = mb.read_reg(REG_SYS_FAN)
    check("Fan ON readback", val, 1)

    time.sleep(1.0)

    print("  Turning fan OFF...")
    mb.write_reg(REG_SYS_FAN, 0)
    time.sleep(0.2)
    val = mb.read_reg(REG_SYS_FAN)
    check("Fan OFF readback", val, 0)

def test_user_switches(mb: ModbusClient):
    print("\n--- User Switches ---")
    print("  Reading switch state (press switches manually to verify)...")
    for _ in range(5):
        sw = mb.read_reg(REG_SYS_USER_SW)
        if sw is not None:
            print(f"  SW register: {sw:#06x}  "
                  f"SW1={'ON' if sw&1 else 'off'}  "
                  f"SW2={'ON' if (sw>>1)&1 else 'off'}  "
                  f"SW3={'ON' if (sw>>2)&1 else 'off'}  "
                  f"SW4={'ON' if (sw>>3)&1 else 'off'}")
        else:
            print(f"  {FAIL}  No response")
        time.sleep(0.5)

def test_axis_status(mb: ModbusClient, name: str, base: int):
    print(f"\n--- {name} Axis Status ---")
    vals = mb.read_regs(base, 10)
    if vals is None:
        print(f"  {FAIL}  Could not read {name} status registers")
        return

    status  = vals[0]
    pos     = (vals[1] << 16) | vals[2]
    deg_raw = (vals[3] << 16) | vals[4]
    rpm_raw = (vals[5] << 16) | vals[6]
    error   = vals[9]

    deg = struct.unpack('>f', struct.pack('>I', deg_raw))[0]
    rpm = struct.unpack('>f', struct.pack('>I', rpm_raw))[0]

    print(f"  {PASS}  Status flags : {status:#06x}  "
          f"(enabled={(status>>0)&1} moving={(status>>1)&1} "
          f"at_target={(status>>2)&1} homed={(status>>4)&1})")
    print(f"  {PASS}  Position     : {pos} counts  ({deg:.3f} deg)")
    print(f"  {PASS}  RPM          : {rpm:.3f}")
    print(f"  {PASS}  Error code   : {error:#04x}")

def test_uptime_advancing(mb: ModbusClient):
    print("\n--- Uptime Advancing ---")
    t1 = mb.read_int32(REG_SYS_UPTIME_HI)
    time.sleep(1.0)
    t2 = mb.read_int32(REG_SYS_UPTIME_HI)
    if t1 is None or t2 is None:
        print(f"  {FAIL}  Could not read uptime")
        return
    delta = t2 - t1
    ok = 800 < delta < 1200
    symbol = PASS if ok else FAIL
    print(f"  {symbol}  Uptime delta: {delta}ms (expected ~1000ms)")

def test_broadcast_fan(ser_port, baud):
    """Send a broadcast fan-off command — no response expected."""
    print("\n--- Broadcast Fan OFF (0xFF) ---")
    addr  = BROADCAST_ADDR
    frame = append_crc(struct.pack('>BBHH', addr, 0x06, REG_SYS_FAN, 0))
    with serial.Serial(ser_port, baud, timeout=0.3) as s:
        s.write(frame)
        resp = s.read(8)
        if len(resp) == 0:
            print(f"  {PASS}  No response received (correct for broadcast)")
        else:
            print(f"  {FAIL}  Unexpected response: {resp.hex()}")

# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Modbus RTU test for Axis Controller")
    parser.add_argument("--port",  required=True,        help="Serial port")
    parser.add_argument("--addr",  type=int, default=1,  help="Board node address (default: 1)")
    parser.add_argument("--baud",  type=int, default=115200)
    args = parser.parse_args()

    print(f"Connecting to {args.port} at {args.baud} baud, node address {args.addr}")

    try:
        mb = ModbusClient(args.port, args.baud, args.addr)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    try:
        test_system(mb)
        test_uptime_advancing(mb)
        test_fan(mb)
        test_user_switches(mb)
        test_axis_status(mb, "AZ", REG_AZ_STATUS)
        test_axis_status(mb, "EL", REG_EL_STATUS)
        test_broadcast_fan(args.port, args.baud)

        print("\nAll tests complete.\n")

    finally:
        mb.close()

if __name__ == "__main__":
    main()
