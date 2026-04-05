#!/usr/bin/env python3
"""
mcu_test_host.py  —  MCU Test Board exerciser for Raspberry Pi
Board: STM32G431KBT6TR  |  Protocol: ModBus-RTU over RS-232
Rev 1.5

Usage:
    python3 mcu_test_host.py [--port /dev/ttyUSB0] [--addr 1] [--baud 115200] [--driver-ppr 400] [--gear-ratio 1.0]

Runs an interactive CLI that lets you:
  • Configure and query the encoder
  • Poll encoder position, velocity and limit switch status
  • Control motor PWM / direction / enable
  • Read I2C sensors (SHT45, ADXL345)
  • Run a full automated test sequence

All communication is host-initiated polling — no unsolicited data from MCU.

Dependencies:
    pip install pyserial
"""

import argparse
import configparser
import os
import struct
import sys
import threading
import time
import logging
from dataclasses import dataclass
from typing import Optional, Tuple

import serial

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Register addresses — must match PROTOCOL.md
# ---------------------------------------------------------------------------
REG_ENC_PPR          = 0x0000
REG_ENC_LIM1_POS_HI  = 0x0001
REG_ENC_LIM1_POS_LO  = 0x0002
REG_ENC_LIM2_POS_HI  = 0x0003
REG_ENC_LIM2_POS_LO  = 0x0004

REG_ENC_POS_HI       = 0x0010
REG_ENC_POS_LO       = 0x0011
REG_ENC_VELOCITY     = 0x0012
REG_ENC_STATUS       = 0x0013

REG_MOT_ENABLE       = 0x0020
REG_MOT_DIR          = 0x0021
REG_MOT_PWM_FREQ     = 0x0022
REG_MOT_PWM_DUTY     = 0x0023
REG_MOT_STATUS       = 0x0024
REG_MOT_DRIVER_PPR   = 0x0025
REG_MOT_GEAR_NUM     = 0x0026
REG_MOT_GEAR_DEN     = 0x0027
REG_MOT_TARGET_RPM   = 0x0028
REG_MOT_ACTUAL_RPM   = 0x0029
REG_MOT_ACCEL_HZ_S   = 0x002A
REG_MOT_DECEL_HZ_S   = 0x002B
REG_MOT_JERK_HZ_S2   = 0x002C
REG_MOT_CURRENT_HZ   = 0x002D
REG_MOT_MAX_RPM      = 0x002E

REG_SHT45_TEMP       = 0x0030
REG_SHT45_RH         = 0x0031
REG_SHT45_STATUS     = 0x0032

REG_ADXL_X           = 0x0038
REG_ADXL_Y           = 0x0039
REG_ADXL_Z           = 0x003A
REG_ADXL_STATUS      = 0x003B

REG_SYS_ADDR         = 0x00F0
REG_SYS_UPTIME_HI    = 0x00F1
REG_SYS_UPTIME_LO    = 0x00F2
REG_SYS_FW_VER       = 0x00F3
REG_SYS_RESET        = 0x00F4

FC_READ_HOLDING      = 0x03
FC_WRITE_SINGLE      = 0x06
FC_WRITE_MULTIPLE    = 0x10

MODBUS_EXCEPTION_MASK = 0x80

EXCEPTION_CODES = {
    0x01: "Illegal Function",
    0x02: "Illegal Register Address",
    0x03: "Illegal Data Value",
    0x04: "Slave Device Failure",
}

# ENC_STATUS bitfield
ENC_STATUS_AT_LIMIT1  = (1 << 0)
ENC_STATUS_AT_LIMIT2  = (1 << 1)
ENC_STATUS_INDEX_SEEN = (1 << 2)
ENC_STATUS_OVERFLOW   = (1 << 3)

# ---------------------------------------------------------------------------
# CRC-16 ModBus
# ---------------------------------------------------------------------------

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
    crc = crc16(data)
    return data + struct.pack("<H", crc)


def check_crc(frame: bytes) -> bool:
    if len(frame) < 3:
        return False
    payload, crc_bytes = frame[:-2], frame[-2:]
    expected = struct.pack("<H", crc16(payload))
    return crc_bytes == expected

# ---------------------------------------------------------------------------
# ModBus frame builders
# ---------------------------------------------------------------------------

def build_read_holding(addr: int, reg: int, count: int) -> bytes:
    pdu = struct.pack(">BBHH", addr, FC_READ_HOLDING, reg, count)
    return append_crc(pdu)


def build_write_single(addr: int, reg: int, value: int) -> bytes:
    pdu = struct.pack(">BBHH", addr, FC_WRITE_SINGLE, reg, value & 0xFFFF)
    return append_crc(pdu)


def build_write_multiple(addr: int, reg: int, values: list[int]) -> bytes:
    count = len(values)
    byte_count = count * 2
    pdu = struct.pack(">BBHHB", addr, FC_WRITE_MULTIPLE, reg, count, byte_count)
    for v in values:
        pdu += struct.pack(">H", v & 0xFFFF)
    return append_crc(pdu)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def regs_to_int32(hi: int, lo: int) -> int:
    raw = (hi << 16) | (lo & 0xFFFF)
    if raw >= 0x80000000:
        raw -= 0x100000000
    return raw


def int32_to_regs(value: int) -> Tuple[int, int]:
    if value < 0:
        value += 0x100000000
    return (value >> 16) & 0xFFFF, value & 0xFFFF

# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class EncoderStatus:
    position:   int     # signed 32-bit counts
    velocity:   int     # signed 16-bit counts/100ms
    at_limit1:  bool
    at_limit2:  bool
    index_seen: bool
    overflow:   bool


@dataclass
class MotorStatus:
    enabled:   bool
    running:   bool
    dir_cw:    bool
    pwm_freq:  int      # Hz
    pwm_duty:  int      # 0-1000 (tenths of %)
    direction: int      # 0=CW, 1=CCW


@dataclass
class SHT45Data:
    temperature_c: float
    humidity_pct:  float
    ok:            bool


@dataclass
class ADXL345Data:
    x_mg: float
    y_mg: float
    z_mg: float
    ok:   bool


class ModBusError(Exception):
    pass

# ---------------------------------------------------------------------------
# ModBus client — pure polled, no background push handling
# ---------------------------------------------------------------------------

class MCUClient:
    """
    Thread-safe ModBus-RTU client (polled only).
    Background reader thread accumulates incoming bytes into frames
    and signals _response_ready when a complete frame arrives.
    """
    INTER_FRAME_GAP  = 0.005   # 5 ms silence before transmitting
    RESPONSE_TIMEOUT = 1.0     # seconds to wait for a response
    FRAME_GAP        = 0.020   # 20 ms inter-frame gap for frame detection

    def __init__(self, port: str, baud: int, node_addr: int):
        self.addr = node_addr
        self._ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
        self._lock             = threading.Lock()
        self._stop_event       = threading.Event()
        self._pending_response: Optional[bytearray] = None
        self._response_ready   = threading.Event()
        self._rx_thread        = threading.Thread(target=self._rx_loop, daemon=False)
        self._rx_thread.start()
        log.info("MCUClient connected to %s @ %d baud, node %d", port, baud, node_addr)

    # ------------------------------------------------------------------
    # Background reader — accumulates bytes, dispatches complete frames
    # ------------------------------------------------------------------

    def _rx_loop(self):
        buf = bytearray()
        last_byte_time = time.monotonic()

        while not self._stop_event.is_set():
            try:
                chunk = self._ser.read(64)
            except Exception:
                break
            now = time.monotonic()
            if chunk:
                if buf and (now - last_byte_time > self.FRAME_GAP):
                    # Gap before new data — dispatch what we have first
                    self._dispatch_frame(bytes(buf))
                    buf = bytearray()
                buf.extend(chunk)
                last_byte_time = now
            else:
                # read() timed out — check for end of frame
                if buf and (now - last_byte_time > self.FRAME_GAP):
                    self._dispatch_frame(bytes(buf))
                    buf = bytearray()

    def _dispatch_frame(self, frame: bytes):
        if len(frame) < 4:
            return
        if not check_crc(frame):
            log.warning("RX CRC error: %s", frame.hex())
            return
        # Ignore frames not addressed to this node
        if frame[0] != self.addr:
            log.debug("Ignoring frame for addr %d (ours=%d)", frame[0], self.addr)
            return
        log.debug("RX: %s", frame.hex())
        self._pending_response = bytearray(frame)
        self._response_ready.set()

    # ------------------------------------------------------------------
    # Raw request/response exchange
    # ------------------------------------------------------------------

    def _transact(self, request: bytes) -> bytes:
        with self._lock:
            self._response_ready.clear()
            self._pending_response = None
            time.sleep(self.INTER_FRAME_GAP)
            self._ser.write(request)
            log.debug("TX: %s", request.hex())

            deadline = time.monotonic() + self.RESPONSE_TIMEOUT
            while time.monotonic() < deadline:
                if self._response_ready.wait(timeout=0.05):
                    resp = bytes(self._pending_response)
                    self._check_exception(resp)
                    return resp
            raise ModBusError("Timeout waiting for response")

    @staticmethod
    def _check_exception(frame: bytes):
        if len(frame) >= 3 and (frame[1] & MODBUS_EXCEPTION_MASK):
            code = frame[2]
            msg = EXCEPTION_CODES.get(code, f"Unknown({code:#x})")
            raise ModBusError(f"ModBus exception 0x{code:02X}: {msg}")

    # ------------------------------------------------------------------
    # ModBus primitives
    # ------------------------------------------------------------------

    def read_registers(self, reg: int, count: int) -> list[int]:
        req  = build_read_holding(self.addr, reg, count)
        resp = self._transact(req)
        byte_count = resp[2]
        if byte_count != count * 2:
            raise ModBusError(f"Expected {count*2} data bytes, got {byte_count}")
        values = []
        for i in range(count):
            val, = struct.unpack(">H", resp[3 + i*2 : 5 + i*2])
            values.append(val)
        return values

    def write_register(self, reg: int, value: int) -> None:
        req = build_write_single(self.addr, reg, value)
        self._transact(req)

    def write_registers(self, reg: int, values: list[int]) -> None:
        req = build_write_multiple(self.addr, reg, values)
        self._transact(req)

    # ------------------------------------------------------------------
    # Encoder API
    # ------------------------------------------------------------------

    def set_encoder_config(
        self,
        ppr: int,
        limit1_pos: int = 0,
        limit2_pos: int = 0,
    ) -> None:
        """Write encoder PPR and limit switch positions."""
        l1_hi, l1_lo = int32_to_regs(limit1_pos)
        l2_hi, l2_lo = int32_to_regs(limit2_pos)
        values = [ppr, l1_hi, l1_lo, l2_hi, l2_lo]
        self.write_registers(REG_ENC_PPR, values)
        log.info("Encoder config: PPR=%d lim1=%d lim2=%d", ppr, limit1_pos, limit2_pos)

    def get_encoder_config(self) -> dict:
        regs = self.read_registers(REG_ENC_PPR, 5)
        return {
            "ppr":        regs[0],
            "limit1_pos": regs_to_int32(regs[1], regs[2]),
            "limit2_pos": regs_to_int32(regs[3], regs[4]),
        }

    def get_encoder_status(self) -> EncoderStatus:
        regs = self.read_registers(REG_ENC_POS_HI, 4)
        pos  = regs_to_int32(regs[0], regs[1])
        vel_raw  = regs[2]
        velocity = vel_raw if vel_raw < 0x8000 else vel_raw - 0x10000
        status   = regs[3]
        return EncoderStatus(
            position=pos,
            velocity=velocity,
            at_limit1=bool(status & ENC_STATUS_AT_LIMIT1),
            at_limit2=bool(status & ENC_STATUS_AT_LIMIT2),
            index_seen=bool(status & ENC_STATUS_INDEX_SEEN),
            overflow=bool(status & ENC_STATUS_OVERFLOW),
        )

    # ------------------------------------------------------------------
    # Motor API
    # ------------------------------------------------------------------

    def motor_enable(self, enable: bool) -> None:
        # DM556Y ENA input is active LOW (opto-isolated).
        # MCU register 1 = enabled (GPIO high → opto ON → ENA- pulled low).
        # Register 0 = disabled (GPIO low → opto OFF → ENA- pulled high).
        self.write_register(REG_MOT_ENABLE, int(enable))
        log.info("Motor %s", "ENABLED" if enable else "DISABLED")

    def motor_set_direction(self, cw: bool) -> None:
        self.write_register(REG_MOT_DIR, 0 if cw else 1)
        log.info("Motor direction: %s", "CW" if cw else "CCW")

    def motor_set_pwm(self, freq_hz: int, duty_tenths: int = 500) -> None:
        if not (1 <= freq_hz <= 50000):
            raise ValueError(f"PWM freq out of range: {freq_hz}")
        if not (0 <= duty_tenths <= 1000):
            raise ValueError(f"PWM duty out of range: {duty_tenths}")
        self.write_registers(REG_MOT_PWM_FREQ, [freq_hz, duty_tenths])
        log.info("Motor PWM: %d Hz, duty %.1f%%", freq_hz, duty_tenths / 10.0)

    def motor_get_status(self) -> MotorStatus:
        regs = self.read_registers(REG_MOT_ENABLE, 5)
        status_bits = regs[4]
        return MotorStatus(
            enabled=bool(regs[0]),
            dir_cw=(regs[1] == 0),
            pwm_freq=regs[2],
            pwm_duty=regs[3],
            direction=regs[1],
            running=bool(status_bits & 0x02),
        )

    # ------------------------------------------------------------------
    # Motion config API
    # ------------------------------------------------------------------

    def set_motion_config(self, driver_ppr: int, gear_num: int, gear_den: int) -> None:
        """Write driver PPR and gear ratio to MCU. Gear ratio = gear_num:gear_den."""
        if not (1 <= driver_ppr <= 65535): raise ValueError(f"driver_ppr out of range: {driver_ppr}")
        if not (1 <= gear_num   <= 65535): raise ValueError(f"gear_num out of range: {gear_num}")
        if not (1 <= gear_den   <= 65535): raise ValueError(f"gear_den out of range: {gear_den}")
        self.write_registers(REG_MOT_DRIVER_PPR, [driver_ppr, gear_num, gear_den])
        log.info("Motion config: driver_ppr=%d gear=%d:%d", driver_ppr, gear_num, gear_den)

    def get_motion_config(self) -> dict:
        regs = self.read_registers(REG_MOT_DRIVER_PPR, 3)
        return {"driver_ppr": regs[0], "gear_num": regs[1], "gear_den": regs[2]}

    def set_target_rpm(self, rpm: int) -> None:
        """Command output shaft RPM. MCU calculates and applies PWM frequency.
           rpm=0 stops motion (duty zeroed, motor stays enabled)."""
        if not (0 <= rpm <= 65535): raise ValueError(f"RPM out of range: {rpm}")
        self.write_register(REG_MOT_TARGET_RPM, rpm)
        log.info("Target RPM: %d", rpm)

    def get_actual_rpm(self) -> int:
        """Read actual output shaft RPM calculated from encoder velocity."""
        return self.read_registers(REG_MOT_ACTUAL_RPM, 1)[0]

    def set_ramp_config(self, accel_hz_s: int, decel_hz_s: int,
                        jerk_hz_s2: int = 0) -> None:
        """Set PWM frequency ramp parameters.
           accel_hz_s: max accel rate Hz/sec (0=instant)
           decel_hz_s: max decel rate Hz/sec (0=instant)
           jerk_hz_s2: how fast accel ramps Hz/sec² (0=linear)
        """
        self.write_registers(REG_MOT_ACCEL_HZ_S, [accel_hz_s, decel_hz_s, jerk_hz_s2])
        log.info("Ramp: accel=%d decel=%d jerk=%d", accel_hz_s, decel_hz_s, jerk_hz_s2)

    def get_ramp_config(self) -> dict:
        """Read ramp config and current ramp position from MCU."""
        regs = self.read_registers(REG_MOT_ACCEL_HZ_S, 4)
        return {
            "accel_hz_s":  regs[0],
            "decel_hz_s":  regs[1],
            "jerk_hz_s2":  regs[2],
            "current_hz":  regs[3],
        }

    def set_max_motor_rpm(self, max_rpm: int) -> None:
        """Set max motor shaft RPM. 0 = no limit."""
        self.write_register(REG_MOT_MAX_RPM, max_rpm)
        log.info("Max motor RPM: %d", max_rpm)

    def get_max_motor_rpm(self) -> int:
        return self.read_registers(REG_MOT_MAX_RPM, 1)[0]

    def save_settings_ini(self, filepath: str) -> None:
        """Read all settings from MCU and save to an INI file."""
        enc  = self.get_encoder_config()
        mot  = self.get_motion_config()
        ramp = self.get_ramp_config()
        info = self.get_system_info()

        cfg = configparser.ConfigParser()
        cfg["info"] = {
            "; node address read from SW1 on MCU (informational, not written back)": "",
            "node_addr": str(info["node_addr"]),
            "fw_ver":    info["fw_ver"],
        }
        cfg["encoder"] = {
            "; encoder pulses per revolution (E6B2-CWZ6C = 2000)": "",
            "ppr":        str(enc["ppr"]),
            "; encoder count at limit switch 1 position": "",
            "limit1_pos": str(enc["limit1_pos"]),
            "; encoder count at limit switch 2 position": "",
            "limit2_pos": str(enc["limit2_pos"]),
        }
        cfg["motor"] = {
            "; DM556Y pulses per rev set by switches on driver": "",
            "driver_ppr": str(mot["driver_ppr"]),
            "; gear ratio as numerator:denominator  e.g. 20:1 = num=20 den=1": "",
            "gear_num":   str(mot["gear_num"]),
            "gear_den":   str(mot["gear_den"]),
            "; PWM duty cycle 0-1000 (tenths of %, 500 = 50%%)": "",
            "pwm_duty":   str(self.read_registers(REG_MOT_PWM_DUTY, 1)[0]),
            "; max motor shaft RPM before gears (0 = no limit)": "",
            "max_rpm":    str(self.get_max_motor_rpm()),
        }
        cfg["ramp"] = {
            "; acceleration rate in Hz per second (0 = instant)": "",
            "accel_hz_s": str(ramp["accel_hz_s"]),
            "; deceleration rate in Hz per second (0 = instant)": "",
            "decel_hz_s": str(ramp["decel_hz_s"]),
            "; jerk in Hz per second squared - how fast accel ramps (0 = linear)": "",
            "jerk_hz_s2": str(ramp["jerk_hz_s2"]),
        }
        with open(filepath, "w") as f:
            cfg.write(f)
        log.info("Settings saved to %s", filepath)

    def load_settings_ini(self, filepath: str) -> None:
        """Load settings from INI file and write to MCU. Motor left disabled."""
        cfg = configparser.ConfigParser()
        cfg.read(filepath)

        if "encoder" in cfg:
            enc = cfg["encoder"]
            self.set_encoder_config(
                ppr=        cfg.getint("encoder", "ppr",        fallback=2000),
                limit1_pos= cfg.getint("encoder", "limit1_pos", fallback=0),
                limit2_pos= cfg.getint("encoder", "limit2_pos", fallback=0),
            )

        if "motor" in cfg:
            self.set_motion_config(
                driver_ppr= cfg.getint("motor", "driver_ppr", fallback=400),
                gear_num=   cfg.getint("motor", "gear_num",   fallback=1),
                gear_den=   cfg.getint("motor", "gear_den",   fallback=1),
            )
            self.write_register(
                REG_MOT_PWM_DUTY,
                cfg.getint("motor", "pwm_duty", fallback=500))
            self.set_max_motor_rpm(
                cfg.getint("motor", "max_rpm", fallback=0))

        if "ramp" in cfg:
            self.set_ramp_config(
                accel_hz_s= cfg.getint("ramp", "accel_hz_s", fallback=2000),
                decel_hz_s= cfg.getint("ramp", "decel_hz_s", fallback=2000),
                jerk_hz_s2= cfg.getint("ramp", "jerk_hz_s2", fallback=500),
            )

        self.motor_enable(False)
        log.info("Settings loaded from %s — motor left DISABLED", filepath)

    # ------------------------------------------------------------------
    # I2C sensor API
    # ------------------------------------------------------------------

    def get_sht45(self) -> SHT45Data:
        regs = self.read_registers(REG_SHT45_TEMP, 3)
        temp_raw = regs[0] if regs[0] < 0x8000 else regs[0] - 0x10000
        return SHT45Data(
            temperature_c=temp_raw / 100.0,
            humidity_pct=regs[1] / 100.0,
            ok=(regs[2] == 0),
        )

    def get_adxl345(self) -> ADXL345Data:
        regs  = self.read_registers(REG_ADXL_X, 4)
        def s16(v): return v if v < 0x8000 else v - 0x10000
        scale = 3.9   # mg per LSB at ±2g full resolution
        return ADXL345Data(
            x_mg=s16(regs[0]) * scale,
            y_mg=s16(regs[1]) * scale,
            z_mg=s16(regs[2]) * scale,
            ok=(regs[3] == 0),
        )

    # ------------------------------------------------------------------
    # System
    # ------------------------------------------------------------------

    def get_system_info(self) -> dict:
        regs      = self.read_registers(REG_SYS_ADDR, 4)
        uptime_ms = (regs[1] << 16) | regs[2]
        fw        = regs[3]
        return {
            "node_addr": regs[0],
            "uptime_ms": uptime_ms,
            "uptime_s":  uptime_ms / 1000.0,
            "fw_ver":    f"{(fw >> 8) & 0xFF}.{fw & 0xFF}",
        }

    def soft_reset(self) -> None:
        log.warning("Sending soft-reset to MCU")
        try:
            self.write_register(REG_SYS_RESET, 0xDEAD)
        except ModBusError:
            pass   # MCU resets before it can respond

    def close(self) -> None:
        self._stop_event.set()
        self._ser.close()          # unblocks pending read() in _rx_loop
        self._rx_thread.join(timeout=2.0)

# ---------------------------------------------------------------------------
# Automated test suite
# ---------------------------------------------------------------------------

class TestSuite:
    PASS = "\033[92mPASS\033[0m"
    FAIL = "\033[91mFAIL\033[0m"

    def __init__(self, client: MCUClient, driver_ppr: int = 400, gear_ratio: float = 1.0):
        self.c = client
        self.driver_ppr = driver_ppr
        self.gear_ratio = gear_ratio
        self.results: list[tuple[str, bool, str]] = []

    def _assert(self, name: str, condition: bool, detail: str = ""):
        status = self.PASS if condition else self.FAIL
        self.results.append((name, condition, detail))
        print(f"  [{status}] {name}" + (f"  — {detail}" if detail else ""))
        return condition

    def run_all(self):
        print("\n" + "="*60)
        print("  MCU TEST BOARD — Full Automated Test Suite")
        print("="*60)

        self._test_system()
        self._test_encoder_config()
        self._test_encoder_read()
        self._test_limit_switches()
        self._test_motor()
        self._test_sht45()
        self._test_adxl345()

        passed = sum(1 for _, ok, _ in self.results if ok)
        total  = len(self.results)
        print("\n" + "="*60)
        print(f"  Results: {passed}/{total} passed")
        print("="*60 + "\n")
        return passed == total

    def _test_system(self):
        print("\n[System]")
        try:
            info = self.c.get_system_info()
            self._assert("System info readable", True,
                         f"addr={info['node_addr']} fw={info['fw_ver']} up={info['uptime_s']:.1f}s")
            self._assert("Node address matches CLI",
                         info["node_addr"] == self.c.addr,
                         f"SW1 reports {info['node_addr']}, expected {self.c.addr}")
        except ModBusError as e:
            self._assert("System info readable", False, str(e))

    def _test_encoder_config(self):
        print("\n[Encoder Configuration]")
        try:
            self.c.set_encoder_config(ppr=2000, limit1_pos=0, limit2_pos=50000)
            cfg = self.c.get_encoder_config()
            self._assert("PPR written/read back",    cfg["ppr"] == 2000,       f"got {cfg['ppr']}")
            self._assert("Limit1 pos writeback",     cfg["limit1_pos"] == 0,   f"got {cfg['limit1_pos']}")
            self._assert("Limit2 pos writeback",     cfg["limit2_pos"] == 50000, f"got {cfg['limit2_pos']}")
        except ModBusError as e:
            self._assert("Encoder config transaction", False, str(e))

    def _test_encoder_read(self):
        print("\n[Encoder Status]")
        try:
            enc = self.c.get_encoder_status()
            self._assert("Encoder position readable", True,
                         f"pos={enc.position} vel={enc.velocity} cnt/100ms")
            self._assert("No overflow on fresh board", not enc.overflow,
                         "Overflow flag set — check encoder wiring")
        except ModBusError as e:
            self._assert("Encoder status readable", False, str(e))

    def _test_limit_switches(self):
        print("\n[Limit Switches]")
        try:
            enc = self.c.get_encoder_status()
            self._assert("Limit switch 1 readable", True,
                         f"{'ACTIVE' if enc.at_limit1 else 'not active'}")
            self._assert("Limit switch 2 readable", True,
                         f"{'ACTIVE' if enc.at_limit2 else 'not active'}")
            print("  NOTE: manually trigger each limit switch to verify ACTIVE state")
        except ModBusError as e:
            self._assert("Limit switch read", False, str(e))

    def _test_motor(self):
        print("\n[Motor Control]")
        try:
            self.c.motor_enable(False)
            st = self.c.motor_get_status()
            self._assert("Motor disables",          not st.enabled)

            self.c.motor_set_direction(cw=True)
            st = self.c.motor_get_status()
            self._assert("Direction set CW",        st.dir_cw)

            self.c.motor_set_direction(cw=False)
            st = self.c.motor_get_status()
            self._assert("Direction set CCW",       not st.dir_cw)

            self.c.motor_set_pwm(freq_hz=1000, duty_tenths=500)
            st = self.c.motor_get_status()
            motor_rpm  = (st.pwm_freq / self.driver_ppr) * 60.0 if self.driver_ppr > 0 else 0.0
            output_rpm = motor_rpm / self.gear_ratio if self.gear_ratio > 0 else 0.0
            self._assert("PWM freq 1000 Hz",        st.pwm_freq == 1000,
                         f"got {st.pwm_freq} (motor {motor_rpm:.1f} RPM, output {output_rpm:.1f} RPM)")
            self._assert("PWM duty 50.0%",          st.pwm_duty == 500,  f"got {st.pwm_duty}")

            self.c.motor_enable(True)
            st = self.c.motor_get_status()
            self._assert("Motor enables",           st.enabled)
            time.sleep(0.5)

            self.c.motor_enable(False)
            st = self.c.motor_get_status()
            self._assert("Motor disables again",    not st.enabled)
        except ModBusError as e:
            self._assert("Motor test", False, str(e))

    def _test_sht45(self):
        print("\n[SHT45 Temp/Humidity]")
        try:
            d = self.c.get_sht45()
            self._assert("SHT45 I2C read OK",    d.ok, "I2C error" if not d.ok else "")
            self._assert("Temperature plausible",
                         -10.0 < d.temperature_c < 60.0, f"{d.temperature_c:.2f} °C")
            self._assert("Humidity plausible",
                         0.0 <= d.humidity_pct <= 100.0, f"{d.humidity_pct:.2f} %")
        except ModBusError as e:
            self._assert("SHT45 readable", False, str(e))

    def _test_adxl345(self):
        print("\n[ADXL345 Accelerometer]")
        try:
            d = self.c.get_adxl345()
            self._assert("ADXL345 I2C read OK", d.ok, "I2C error" if not d.ok else "")
            total_g = ((d.x_mg**2 + d.y_mg**2 + d.z_mg**2) ** 0.5) / 1000.0
            self._assert("Gravity vector ~1g",
                         0.8 < total_g < 1.2,
                         f"|g|={total_g:.3f}g  X={d.x_mg:.0f} Y={d.y_mg:.0f} Z={d.z_mg:.0f} mg")
        except ModBusError as e:
            self._assert("ADXL345 readable", False, str(e))

# ---------------------------------------------------------------------------
# Interactive CLI
# ---------------------------------------------------------------------------

MENU = """
╔══════════════════════════════════════════════╗
║       MCU Test Board — Interactive CLI       ║
╠══════════════════════════════════════════════╣
║  1) System info                              ║
║  2) Read encoder + limit switch status       ║
║  3) Configure encoder (PPR, limit positions) ║
║  4) Poll encoder continuously                ║
║  5) Motor: enable/disable                    ║
║  6) Motor: set direction                     ║
║  7) Motor: set PWM freq & duty               ║
║  8) Motor: get status (Hz + RPM)             ║
║  9) Configure motion (driver PPR, gear ratio)║
║ 10) Set target output shaft RPM              ║
║ 11) Configure ramp (accel/decel/jerk)        ║
║ 12) Set max motor shaft RPM                  ║
║ 13) Read SHT45 sensor                        ║
║ 14) Read ADXL345 accelerometer               ║
║ 15) Save settings to file                    ║
║ 16) Load settings from file                  ║
║ 17) Run automated test suite                 ║
║ 18) Soft-reset MCU                           ║
║  x) EMERGENCY STOP — disable motor now       ║
║  q) Quit                                     ║
╚══════════════════════════════════════════════╝
"""

def _limit_str(active: bool) -> str:
    return "\033[91mACTIVE\033[0m" if active else "off"

def _print_encoder(enc: EncoderStatus):
    print(f"\n  Position   : {enc.position:+10d} counts")
    print(f"  Velocity   : {enc.velocity:+6d} counts/100ms")
    print(f"  Limit SW 1 : {_limit_str(enc.at_limit1)}")
    print(f"  Limit SW 2 : {_limit_str(enc.at_limit2)}")
    print(f"  Index      : {'seen' if enc.index_seen else 'not seen'}")
    print(f"  Overflow   : {'YES' if enc.overflow else 'no'}")

def interactive_cli(client: MCUClient, driver_ppr: int = 400, gear_ratio: float = 1.0):
    while True:
        print(MENU)
        choice = input("Select: ").strip().lower()

        if choice == "q":
            break

        elif choice == "1":
            try:
                info = client.get_system_info()
                print(f"\n  Node address : {info['node_addr']}")
                print(f"  Firmware ver : {info['fw_ver']}")
                print(f"  Uptime       : {info['uptime_s']:.1f} s  ({info['uptime_ms']} ms)")
            except ModBusError as e:
                print(f"  Error: {e}")

        elif choice == "2":
            try:
                _print_encoder(client.get_encoder_status())
            except ModBusError as e:
                print(f"  Error: {e}")

        elif choice == "3":
            try:
                ppr  = int(input("  PPR [2000]: ") or 2000)
                lim1 = int(input("  Limit1 position counts [0]: ") or 0)
                lim2 = int(input("  Limit2 position counts [50000]: ") or 50000)
                client.set_encoder_config(ppr, lim1, lim2)
                cfg = client.get_encoder_config()
                print(f"  Verified: PPR={cfg['ppr']}  lim1={cfg['limit1_pos']}  lim2={cfg['limit2_pos']}")
            except ModBusError as e:
                print(f"  Error: {e}")

        elif choice == "4":
            print("\n  Polling encoder — press Ctrl+C to stop\n")
            try:
                while True:
                    enc = client.get_encoder_status()
                    ts  = time.strftime("%H:%M:%S")
                    l1  = "\033[91mL1!\033[0m" if enc.at_limit1 else "   "
                    l2  = "\033[91mL2!\033[0m" if enc.at_limit2 else "   "
                    print(f"  [{ts}]  pos={enc.position:+10d}  vel={enc.velocity:+5d}  {l1}  {l2}",
                          end="\r", flush=True)
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n  (stopped)")
            except ModBusError as e:
                print(f"\n  Error: {e}")

        elif choice == "x":
            try:
                client.motor_enable(False)
                print("  *** Motor DISABLED ***")
            except ModBusError as e:
                print(f"  Error: {e}")

        elif choice == "5":
            try:
                en = input("  Enable motor? [y/N]: ").strip().lower() == "y"
                client.motor_enable(en)
            except ModBusError as e:
                print(f"  Error: {e}")

        elif choice == "6":
            try:
                d = input("  Direction CW? [y/N]: ").strip().lower() == "y"
                client.motor_set_direction(cw=d)
            except ModBusError as e:
                print(f"  Error: {e}")

        elif choice == "7":
            try:
                freq = int(input("  Frequency Hz [1000]: ") or 1000)
                duty = int(float(input("  Duty % [50.0]: ") or 50.0) * 10)
                client.motor_set_pwm(freq, duty)
            except (ModBusError, ValueError) as e:
                print(f"  Error: {e}")

        elif choice == "8":
            try:
                st = client.motor_get_status()
                motor_rpm   = (st.pwm_freq / driver_ppr) * 60.0 if driver_ppr > 0 else 0.0
                output_rpm  = motor_rpm / gear_ratio if gear_ratio > 0 else 0.0
                print(f"\n  Enabled        : {st.enabled}")
                print(f"  Running        : {st.running}")
                print(f"  Direction      : {'CW' if st.dir_cw else 'CCW'}")
                print(f"  PWM freq       : {st.pwm_freq} Hz")
                print(f"  Motor shaft    : {motor_rpm:.1f} RPM  ({driver_ppr} pulse/rev)")
                print(f"  Output shaft   : {output_rpm:.1f} RPM  (gear ratio {gear_ratio:.3g}:1)")
                print(f"  PWM duty       : {st.pwm_duty / 10.0:.1f} %")
            except ModBusError as e:
                print(f"  Error: {e}")

        elif choice == "9":
            try:
                cfg = client.get_motion_config()
                ratio = cfg["gear_num"] / cfg["gear_den"]
                print(f"\n  Current motion config:")
                print(f"  Driver PPR : {cfg['driver_ppr']} pulses/rev")
                print(f"  Gear ratio : {cfg['gear_num']}:{cfg['gear_den']}  ({ratio:.4g}:1)")
                ppr  = int(input(f"  New driver PPR [{cfg['driver_ppr']}]: ") or cfg['driver_ppr'])
                gnum = int(input(f"  Gear ratio numerator [{cfg['gear_num']}]: ") or cfg['gear_num'])
                gden = int(input(f"  Gear ratio denominator [{cfg['gear_den']}]: ") or cfg['gear_den'])
                client.set_motion_config(ppr, gnum, gden)
                cfg = client.get_motion_config()
                print(f"  Verified: PPR={cfg['driver_ppr']}  gear={cfg['gear_num']}:{cfg['gear_den']}")
            except (ModBusError, ValueError) as e:
                print(f"  Error: {e}")

        elif choice == "10":
            try:
                cfg    = client.get_motion_config()
                actual = client.get_actual_rpm()
                st     = client.motor_get_status()
                print(f"\n  Motion config : {cfg['driver_ppr']} pulse/rev, "
                      f"gear {cfg['gear_num']}:{cfg['gear_den']}")
                print(f"  Actual output : {actual} RPM")
                ramp = client.get_ramp_config()
                print(f"  Current PWM   : {st.pwm_freq} Hz")
                print(f"  Ramp position : {ramp['current_hz']} Hz")
                print(f"  Accel/Decel   : {ramp['accel_hz_s']}/{ramp['decel_hz_s']} Hz/sec")
                rpm = int(input("  Target output shaft RPM (0=stop): ") or 0)
                client.set_target_rpm(rpm)
                st = client.motor_get_status()
                if rpm == 0:
                    print("  Stopping (ramping down to 0)")
                else:
                    print(f"  Ramping to {rpm} RPM  "
                          f"(target PWM ~{st.pwm_freq} Hz)")
            except (ModBusError, ValueError) as e:
                print(f"  Error: {e}")

        elif choice == "11":
            try:
                cfg  = client.get_ramp_config()
                mcfg = client.get_motion_config()
                ppr  = mcfg["driver_ppr"] or 1
                gn   = mcfg["gear_num"]   or 1
                gd   = mcfg["gear_den"]   or 1
                def hz_to_rpm(hz): return hz * 60.0 / ppr * gd / gn
                print(f"\n  Current ramp config:")
                print(f"  Accel      : {cfg['accel_hz_s']} Hz/sec"
                      f"  (~{hz_to_rpm(cfg['accel_hz_s']):.1f} output RPM/sec)")
                print(f"  Decel      : {cfg['decel_hz_s']} Hz/sec"
                      f"  (~{hz_to_rpm(cfg['decel_hz_s']):.1f} output RPM/sec)")
                print(f"  Jerk       : {cfg['jerk_hz_s2']} Hz/sec²  (0=linear)")
                print(f"  Current Hz : {cfg['current_hz']} Hz"
                      f"  (~{hz_to_rpm(cfg['current_hz']):.1f} output RPM)")
                print(f"  (Ramp always starts from current PWM frequency)")
                accel = int(input(f"  Accel Hz/sec [{cfg['accel_hz_s']}] (0=instant): ")
                            or cfg['accel_hz_s'])
                decel = int(input(f"  Decel Hz/sec [{cfg['decel_hz_s']}] (0=instant): ")
                            or cfg['decel_hz_s'])
                jerk  = int(input(f"  Jerk Hz/sec² [{cfg['jerk_hz_s2']}] (0=linear): ")
                            or cfg['jerk_hz_s2'])
                client.set_ramp_config(accel, decel, jerk)
                cfg = client.get_ramp_config()
                print(f"  Verified: accel={cfg['accel_hz_s']} decel={cfg['decel_hz_s']} "
                      f"jerk={cfg['jerk_hz_s2']} Hz/sec")
            except (ModBusError, ValueError) as e:
                print(f"  Error: {e}")

        elif choice == "12":
            try:
                current = client.get_max_motor_rpm()
                print(f"\n  Current max motor shaft RPM: {current}  (0=no limit)")
                val = int(input("  New max motor RPM (0=no limit): ") or current)
                client.set_max_motor_rpm(val)
                print(f"  Set to {client.get_max_motor_rpm()} RPM")
            except (ModBusError, ValueError) as e:
                print(f"  Error: {e}")

        elif choice == "15":
            try:
                default_file = "mcu_settings.ini"
                fname = input(f"  Save to [{default_file}]: ").strip() or default_file
                client.save_settings_ini(fname)
                print(f"  Saved to {fname}")
            except (ModBusError, OSError) as e:
                print(f"  Error: {e}")

        elif choice == "16":
            try:
                default_file = "mcu_settings.ini"
                fname = input(f"  Load from [{default_file}]: ").strip() or default_file
                if not os.path.exists(fname):
                    print(f"  File not found: {fname}")
                else:
                    client.load_settings_ini(fname)
                    print(f"  Settings loaded from {fname}")
                    print(f"  *** Motor left DISABLED — use option 5 to enable ***")
            except (ModBusError, OSError) as e:
                print(f"  Error: {e}")

        elif choice == "13":
            try:
                d = client.get_sht45()
                print(f"\n  Temperature : {d.temperature_c:.2f} °C")
                print(f"  Humidity    : {d.humidity_pct:.2f} %")
                print(f"  I2C Status  : {'OK' if d.ok else 'ERROR'}")
            except ModBusError as e:
                print(f"  Error: {e}")

        elif choice == "14":
            try:
                d = client.get_adxl345()
                g = ((d.x_mg**2 + d.y_mg**2 + d.z_mg**2) ** 0.5) / 1000.0
                print(f"\n  X          : {d.x_mg:+8.1f} mg")
                print(f"  Y          : {d.y_mg:+8.1f} mg")
                print(f"  Z          : {d.z_mg:+8.1f} mg")
                print(f"  |g|        : {g:.3f} g")
                print(f"  I2C Status : {'OK' if d.ok else 'ERROR'}")
            except ModBusError as e:
                print(f"  Error: {e}")

        elif choice == "17":
            suite = TestSuite(client, driver_ppr, gear_ratio)
            suite.run_all()

        elif choice == "18":
            confirm = input("  Soft-reset MCU? [y/N]: ").strip().lower()
            if confirm == "y":
                client.soft_reset()
                print("  Reset sent. Reconnect in a moment.")
                break

        else:
            print("  Unknown option")

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="MCU Test Board Host — ModBus-RTU over RS-232",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Serial port examples:
  USB-to-serial dongle  : --port /dev/ttyUSB0          (default)
  Pi GPIO TX/RX pins    : --pi-uart                     (uses /dev/ttyAMA0)
  Pi GPIO TX/RX pins    : --pi-uart --pi-uart-port s0   (uses /dev/ttyS0)
  Explicit override     : --port /dev/ttyAMA1

Pi GPIO UART notes:
  Requires serial port enabled via raspi-config:
    Interface Options -> Serial Port
    "login shell over serial" -> No / "serial port hardware" -> Yes
  Pi 3/4/5: GPIO14=TX (pin 8), GPIO15=RX (pin 10) -> /dev/ttyAMA0
  If ttyAMA0 is claimed by Bluetooth, use --pi-uart-port s0
""")
    port_group = parser.add_mutually_exclusive_group()
    port_group.add_argument(
        "--port",
        default=None,
        metavar="DEVICE",
        help="Serial port device (e.g. /dev/ttyUSB0). Default: /dev/ttyUSB0")
    port_group.add_argument(
        "--pi-uart",
        action="store_true",
        help="Use the Pi built-in GPIO UART (TX=pin8/GPIO14, RX=pin10/GPIO15)")
    parser.add_argument(
        "--pi-uart-port",
        choices=["ama0", "s0"],
        default="ama0",
        metavar="{ama0,s0}",
        help="Pi UART device: ama0=/dev/ttyAMA0 (default), s0=/dev/ttyS0")
    parser.add_argument("--baud", default=115200, type=int, help="Baud rate (default 115200)")
    parser.add_argument(
        "--driver-ppr",
        default=400,
        type=int,
        metavar="PPR",
        help="Pulses-per-revolution set on the DM556Y driver switches (default 400). "
             "Used to display RPM alongside Hz in motor status.")
    parser.add_argument(
        "--gear-ratio",
        default=1.0,
        type=float,
        metavar="RATIO",
        help="Gear ratio between motor output shaft and load (default 1.0). "
             "Output shaft RPM = motor RPM / gear ratio.")
    parser.add_argument("--addr", default=1,      type=int, help="ModBus node address 1-7 (matches SW1)")
    parser.add_argument(
        "--load-settings",
        default=None, metavar="FILE",
        help="INI settings file to load on startup. Motor left disabled.")
    parser.add_argument("--test", action="store_true",      help="Run automated test suite and exit")
    args = parser.parse_args()

    if args.pi_uart:
        port = "/dev/ttyAMA0" if args.pi_uart_port == "ama0" else "/dev/ttyS0"
        log.info("Using Pi GPIO UART: %s  (TX=GPIO14/pin8, RX=GPIO15/pin10)", port)
        log.info("Ensure raspi-config has serial hardware enabled and login shell disabled.")
    else:
        port = args.port or "/dev/ttyUSB0"

    log.info("Opening %s @ %d baud, ModBus addr %d", port, args.baud, args.addr)
    client = MCUClient(port=port, baud=args.baud, node_addr=args.addr)

    if args.load_settings:
        if not os.path.exists(args.load_settings):
            log.error("Settings file not found: %s", args.load_settings)
            sys.exit(1)
        client.load_settings_ini(args.load_settings)
        log.info("Settings loaded from %s — motor is DISABLED", args.load_settings)
    else:
        client.motor_enable(False)

    if args.load_settings:
        if not os.path.exists(args.load_settings):
            log.error("Settings file not found: %s", args.load_settings)
            sys.exit(1)
        client.load_settings_ini(args.load_settings)
        log.info("Settings loaded from %s — motor is DISABLED", args.load_settings)
    else:
        # Always disable motor on connect
        client.motor_enable(False)

    try:
        if args.test:
            suite = TestSuite(client, args.driver_ppr, args.gear_ratio)
            ok = suite.run_all()
            sys.exit(0 if ok else 1)
        else:
            interactive_cli(client, args.driver_ppr, args.gear_ratio)
    finally:
        client.close()


if __name__ == "__main__":
    main()
