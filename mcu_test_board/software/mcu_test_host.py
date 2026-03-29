#!/usr/bin/env python3
"""
mcu_test_host.py  —  MCU Test Board exerciser for Raspberry Pi
Board: STM32G431KBT6TR  |  Protocol: ModBus-RTU over RS-232
Rev 1.1

Usage:
    python3 mcu_test_host.py [--port /dev/ttyUSB0] [--addr 1] [--baud 115200]

Runs an interactive CLI that lets you:
  • Configure and query the encoder
  • Control motor PWM / direction / enable
  • Read I2C sensors (SHT45, ADXL345)
  • Monitor unsolicited encoder push notifications
  • Run a full automated test sequence

Dependencies:
    pip install pyserial
"""

import argparse
import struct
import sys
import threading
import time
import queue
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
REG_ENC_PUSH_EN      = 0x0005
REG_ENC_PUSH_THRESH  = 0x0006

REG_ENC_POS_HI       = 0x0010
REG_ENC_POS_LO       = 0x0011
REG_ENC_VELOCITY     = 0x0012
REG_ENC_STATUS       = 0x0013

REG_MOT_ENABLE       = 0x0020
REG_MOT_DIR          = 0x0021
REG_MOT_PWM_FREQ     = 0x0022
REG_MOT_PWM_DUTY     = 0x0023
REG_MOT_STATUS       = 0x0024

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
FC_PUSH_NOTIFY       = 0x41

MODBUS_EXCEPTION_MASK = 0x80

EXCEPTION_CODES = {
    0x01: "Illegal Function",
    0x02: "Illegal Register Address",
    0x03: "Illegal Data Value",
    0x04: "Slave Device Failure",
}

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
# Helper: 32-bit signed from two 16-bit unsigned registers
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
# ModBus client
# ---------------------------------------------------------------------------

@dataclass
class EncoderStatus:
    position: int          # signed 32-bit counts
    velocity: int          # signed 16-bit counts/100ms
    at_limit1: bool
    at_limit2: bool
    index_seen: bool
    overflow: bool


@dataclass
class MotorStatus:
    enabled: bool
    running: bool
    dir_cw: bool
    pwm_freq: int          # Hz
    pwm_duty: int          # 0–1000 (tenths of %)
    direction: int         # 0=CW, 1=CCW


@dataclass
class SHT45Data:
    temperature_c: float
    humidity_pct: float
    ok: bool


@dataclass
class ADXL345Data:
    x_mg: float
    y_mg: float
    z_mg: float
    ok: bool


class ModBusError(Exception):
    pass


class MCUClient:
    """
    Thread-safe ModBus-RTU client.
    A background reader thread captures unsolicited FC=0x41 push frames.
    """
    INTER_FRAME_GAP = 0.005      # 5 ms between request/response
    RESPONSE_TIMEOUT = 0.5       # seconds

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
        self._lock = threading.Lock()
        self._push_queue: queue.Queue = queue.Queue()
        self._stop_event = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._pending_response: Optional[bytearray] = None
        self._response_ready = threading.Event()
        self._rx_thread.start()
        log.info("MCUClient connected to %s @ %d baud, node %d", port, baud, node_addr)

    # ------------------------------------------------------------------
    # Background reader — routes frames to push queue or response buffer
    # ------------------------------------------------------------------

    def _rx_loop(self):
        buf = bytearray()
        last_byte_time = time.monotonic()

        while not self._stop_event.is_set():
            chunk = self._ser.read(64)
            now = time.monotonic()
            if chunk:
                if now - last_byte_time > 0.004:   # 3.5-char gap: new frame
                    buf = bytearray()
                buf.extend(chunk)
                last_byte_time = now
            else:
                if buf and (now - last_byte_time > 0.004):
                    self._dispatch_frame(bytes(buf))
                    buf = bytearray()

    def _dispatch_frame(self, frame: bytes):
        if len(frame) < 4:
            return
        if not check_crc(frame):
            log.warning("RX CRC error: %s", frame.hex())
            return
        fc = frame[1]
        if fc == FC_PUSH_NOTIFY:
            self._handle_push(frame)
        else:
            # Wake up the waiting request
            self._response_ready.set()
            self._pending_response = bytearray(frame)

    def _handle_push(self, frame: bytes):
        """Parse FC=0x41 unsolicited encoder push."""
        # [addr][0x41][POS_HI 2B][POS_LO 2B][FLAGS 1B][CRC 2B]  = 9 bytes
        if len(frame) < 9:
            log.warning("Push frame too short: %s", frame.hex())
            return
        pos_hi, pos_lo, flags = struct.unpack(">HHB", frame[2:7])
        position = regs_to_int32(pos_hi, pos_lo)
        push = {
            "position": position,
            "limit1": bool(flags & 0x01),
            "limit2": bool(flags & 0x02),
            "index": bool(flags & 0x04),
        }
        self._push_queue.put(push)
        log.debug("PUSH: pos=%d flags=0x%02X", position, flags)

    # ------------------------------------------------------------------
    # Raw request/response exchange
    # ------------------------------------------------------------------

    def _transact(self, request: bytes, expected_min: int = 5) -> bytes:
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
                    log.debug("RX: %s", resp.hex())
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
    # Public ModBus primitives
    # ------------------------------------------------------------------

    def read_registers(self, reg: int, count: int) -> list[int]:
        """Read `count` holding registers starting at `reg`. Returns list of uint16."""
        req = build_read_holding(self.addr, reg, count)
        resp = self._transact(req, expected_min=5 + count * 2)
        byte_count = resp[2]
        if byte_count != count * 2:
            raise ModBusError(f"Expected {count*2} data bytes, got {byte_count}")
        values = []
        for i in range(count):
            val, = struct.unpack(">H", resp[3 + i*2 : 5 + i*2])
            values.append(val)
        return values

    def write_register(self, reg: int, value: int) -> None:
        """Write a single uint16 register."""
        req = build_write_single(self.addr, reg, value)
        self._transact(req)

    def write_registers(self, reg: int, values: list[int]) -> None:
        """Write multiple registers."""
        req = build_write_multiple(self.addr, reg, values)
        self._transact(req)

    # ------------------------------------------------------------------
    # High-level encoder API
    # ------------------------------------------------------------------

    def set_encoder_config(
        self,
        ppr: int,
        limit1_pos: int = 0,
        limit2_pos: int = 0,
        push_enable: bool = True,
        push_thresh: int = 1,
    ) -> None:
        """Write all encoder configuration registers in one transaction."""
        l1_hi, l1_lo = int32_to_regs(limit1_pos)
        l2_hi, l2_lo = int32_to_regs(limit2_pos)
        values = [ppr, l1_hi, l1_lo, l2_hi, l2_lo, int(push_enable), push_thresh]
        self.write_registers(REG_ENC_PPR, values)
        log.info("Encoder config written: PPR=%d lim1=%d lim2=%d push=%s thresh=%d",
                 ppr, limit1_pos, limit2_pos, push_enable, push_thresh)

    def get_encoder_config(self) -> dict:
        regs = self.read_registers(REG_ENC_PPR, 7)
        return {
            "ppr":         regs[0],
            "limit1_pos":  regs_to_int32(regs[1], regs[2]),
            "limit2_pos":  regs_to_int32(regs[3], regs[4]),
            "push_enable": bool(regs[5]),
            "push_thresh": regs[6],
        }

    def get_encoder_status(self) -> EncoderStatus:
        regs = self.read_registers(REG_ENC_POS_HI, 4)
        pos = regs_to_int32(regs[0], regs[1])
        vel_raw = regs[2]
        velocity = vel_raw if vel_raw < 0x8000 else vel_raw - 0x10000
        status_bits = regs[3]
        return EncoderStatus(
            position=pos,
            velocity=velocity,
            at_limit1=bool(status_bits & 0x01),
            at_limit2=bool(status_bits & 0x02),
            index_seen=bool(status_bits & 0x04),
            overflow=bool(status_bits & 0x08),
        )

    def get_push_notification(self, timeout: float = 0.0) -> Optional[dict]:
        """Return next push notification dict, or None if queue empty."""
        try:
            return self._push_queue.get(timeout=timeout) if timeout > 0 else self._push_queue.get_nowait()
        except queue.Empty:
            return None

    # ------------------------------------------------------------------
    # High-level motor API
    # ------------------------------------------------------------------

    def motor_enable(self, enable: bool) -> None:
        self.write_register(REG_MOT_ENABLE, int(enable))
        log.info("Motor %s", "ENABLED" if enable else "DISABLED")

    def motor_set_direction(self, cw: bool) -> None:
        self.write_register(REG_MOT_DIR, 0 if cw else 1)
        log.info("Motor direction: %s", "CW" if cw else "CCW")

    def motor_set_pwm(self, freq_hz: int, duty_tenths: int = 500) -> None:
        """
        Set motor PWM.
        freq_hz: pulse frequency in Hz (1–50000)
        duty_tenths: duty cycle in tenths of percent (0–1000)
        """
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
    # High-level I2C sensor API
    # ------------------------------------------------------------------

    def get_sht45(self) -> SHT45Data:
        regs = self.read_registers(REG_SHT45_TEMP, 3)
        temp_raw = regs[0] if regs[0] < 0x8000 else regs[0] - 0x10000
        ok = (regs[2] == 0)
        return SHT45Data(
            temperature_c=temp_raw / 100.0,
            humidity_pct=regs[1] / 100.0,
            ok=ok,
        )

    def get_adxl345(self) -> ADXL345Data:
        regs = self.read_registers(REG_ADXL_X, 4)
        def signed16(v): return v if v < 0x8000 else v - 0x10000
        ok = (regs[3] == 0)
        scale = 3.9  # mg per LSB at ±2g range
        return ADXL345Data(
            x_mg=signed16(regs[0]) * scale,
            y_mg=signed16(regs[1]) * scale,
            z_mg=signed16(regs[2]) * scale,
            ok=ok,
        )

    # ------------------------------------------------------------------
    # System
    # ------------------------------------------------------------------

    def get_system_info(self) -> dict:
        regs = self.read_registers(REG_SYS_ADDR, 4)
        uptime_ms = (regs[1] << 16) | regs[2]
        fw = regs[3]
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
            pass  # MCU resets before it can respond

    def close(self) -> None:
        self._stop_event.set()
        self._ser.close()

# ---------------------------------------------------------------------------
# Automated test suite
# ---------------------------------------------------------------------------

class TestSuite:
    PASS = "\033[92mPASS\033[0m"
    FAIL = "\033[91mFAIL\033[0m"

    def __init__(self, client: MCUClient):
        self.c = client
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
        self._test_motor()
        self._test_sht45()
        self._test_adxl345()
        self._test_push_notification()

        passed = sum(1 for _, ok, _ in self.results if ok)
        total  = len(self.results)
        print("\n" + "="*60)
        print(f"  Results: {passed}/{total} passed")
        print("="*60 + "\n")
        return passed == total

    # ---- System -----------------------------------------------------------

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

    # ---- Encoder config ---------------------------------------------------

    def _test_encoder_config(self):
        print("\n[Encoder Configuration]")
        try:
            self.c.set_encoder_config(
                ppr=2000,
                limit1_pos=0,
                limit2_pos=50000,
                push_enable=True,
                push_thresh=10,
            )
            cfg = self.c.get_encoder_config()
            self._assert("PPR written/read back",    cfg["ppr"] == 2000,   f"got {cfg['ppr']}")
            self._assert("Limit1 pos writeback",     cfg["limit1_pos"] == 0, f"got {cfg['limit1_pos']}")
            self._assert("Limit2 pos writeback",     cfg["limit2_pos"] == 50000, f"got {cfg['limit2_pos']}")
            self._assert("Push enable writeback",    cfg["push_enable"] is True)
            self._assert("Push threshold writeback", cfg["push_thresh"] == 10)
        except ModBusError as e:
            self._assert("Encoder config transaction", False, str(e))

    # ---- Encoder status ---------------------------------------------------

    def _test_encoder_read(self):
        print("\n[Encoder Status]")
        try:
            enc = self.c.get_encoder_status()
            self._assert("Encoder position readable", True,
                         f"pos={enc.position} vel={enc.velocity} cnt/100ms")
            self._assert("Limit1 status bit readable", True,
                         f"LIMIT1={'ACTIVE' if enc.at_limit1 else 'off'}")
            self._assert("Limit2 status bit readable", True,
                         f"LIMIT2={'ACTIVE' if enc.at_limit2 else 'off'}")
            self._assert("No overflow on fresh board", not enc.overflow,
                         "Overflow flag is set — check encoder wiring")
        except ModBusError as e:
            self._assert("Encoder status readable", False, str(e))

    # ---- Motor ------------------------------------------------------------

    def _test_motor(self):
        print("\n[Motor Control]")
        try:
            # Disable first (safe)
            self.c.motor_enable(False)
            st = self.c.motor_get_status()
            self._assert("Motor disables", not st.enabled, f"enabled={st.enabled}")

            # Set direction CW
            self.c.motor_set_direction(cw=True)
            st = self.c.motor_get_status()
            self._assert("Direction set CW", st.dir_cw, f"dir={st.direction}")

            # Set direction CCW
            self.c.motor_set_direction(cw=False)
            st = self.c.motor_get_status()
            self._assert("Direction set CCW", not st.dir_cw)

            # Set PWM
            self.c.motor_set_pwm(freq_hz=1000, duty_tenths=500)
            st = self.c.motor_get_status()
            self._assert("PWM freq 1000 Hz writeback", st.pwm_freq == 1000, f"got {st.pwm_freq}")
            self._assert("PWM duty 50.0% writeback",   st.pwm_duty == 500,  f"got {st.pwm_duty}")

            # Enable (motor will spin if connected)
            self.c.motor_enable(True)
            st = self.c.motor_get_status()
            self._assert("Motor enables", st.enabled)

            time.sleep(0.5)  # let it run briefly

            # Disable again
            self.c.motor_enable(False)
            st = self.c.motor_get_status()
            self._assert("Motor disables again", not st.enabled)

        except ModBusError as e:
            self._assert("Motor test", False, str(e))

    # ---- SHT45 ------------------------------------------------------------

    def _test_sht45(self):
        print("\n[SHT45 Temp/Humidity Sensor]")
        try:
            data = self.c.get_sht45()
            self._assert("SHT45 I2C read OK",      data.ok, "I2C error or timeout" if not data.ok else "")
            self._assert("Temperature plausible",
                         -10.0 < data.temperature_c < 60.0,
                         f"T = {data.temperature_c:.2f} °C")
            self._assert("Humidity plausible",
                         0.0 <= data.humidity_pct <= 100.0,
                         f"RH = {data.humidity_pct:.2f} %")
        except ModBusError as e:
            self._assert("SHT45 readable", False, str(e))

    # ---- ADXL345 ----------------------------------------------------------

    def _test_adxl345(self):
        print("\n[ADXL345 Accelerometer]")
        try:
            data = self.c.get_adxl345()
            self._assert("ADXL345 I2C read OK", data.ok, "I2C error" if not data.ok else "")
            total_g = ((data.x_mg**2 + data.y_mg**2 + data.z_mg**2) ** 0.5) / 1000.0
            self._assert("Gravity vector ~1g",
                         0.8 < total_g < 1.2,
                         f"|g| = {total_g:.3f} g  X={data.x_mg:.1f} Y={data.y_mg:.1f} Z={data.z_mg:.1f} mg")
        except ModBusError as e:
            self._assert("ADXL345 readable", False, str(e))

    # ---- Push notifications -----------------------------------------------

    def _test_push_notification(self):
        print("\n[Encoder Push Notifications]")
        try:
            # Enable push with threshold=1 so any change triggers
            self.c.set_encoder_config(ppr=2000, push_enable=True, push_thresh=1)
            # Enable motor briefly to rotate encoder
            self.c.motor_set_direction(cw=True)
            self.c.motor_set_pwm(freq_hz=500, duty_tenths=500)
            self.c.motor_enable(True)

            print("    Waiting up to 3 s for push notifications (motor running)...")
            received = []
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                pn = self.c.get_push_notification(timeout=0.2)
                if pn:
                    received.append(pn)
                    print(f"    Push #{len(received)}: pos={pn['position']} "
                          f"L1={pn['limit1']} L2={pn['limit2']} idx={pn['index']}")
                    if len(received) >= 5:
                        break

            self.c.motor_enable(False)
            self._assert("Push notifications received",
                         len(received) > 0,
                         f"Got {len(received)} push frames")
            if received:
                positions = [p["position"] for p in received]
                self._assert("Positions are changing",
                             len(set(positions)) > 1,
                             f"Unique positions: {set(positions)}")
        except ModBusError as e:
            self._assert("Push notification test", False, str(e))

# ---------------------------------------------------------------------------
# Interactive CLI
# ---------------------------------------------------------------------------

MENU = """
╔════════════════════════════════════════════╗
║      MCU Test Board — Interactive CLI      ║
╠════════════════════════════════════════════╣
║  1) System info                            ║
║  2) Read encoder status                    ║
║  3) Configure encoder                      ║
║  4) Motor: enable/disable                  ║
║  5) Motor: set direction                   ║
║  6) Motor: set PWM freq & duty             ║
║  7) Motor: get status                      ║
║  8) Read SHT45 sensor                      ║
║  9) Read ADXL345 accelerometer             ║
║ 10) Monitor encoder push notifications     ║
║ 11) Run automated test suite               ║
║ 12) Soft-reset MCU                         ║
║  q) Quit                                   ║
╚════════════════════════════════════════════╝
"""

def interactive_cli(client: MCUClient):
    while True:
        print(MENU)
        choice = input("Select: ").strip().lower()

        if choice == "q":
            break

        elif choice == "1":
            info = client.get_system_info()
            print(f"\n  Node address : {info['node_addr']}")
            print(f"  Firmware ver : {info['fw_ver']}")
            print(f"  Uptime       : {info['uptime_s']:.1f} s  ({info['uptime_ms']} ms)")

        elif choice == "2":
            enc = client.get_encoder_status()
            print(f"\n  Position  : {enc.position} counts")
            print(f"  Velocity  : {enc.velocity} counts/100ms")
            print(f"  LIMIT1    : {'ACTIVE' if enc.at_limit1 else 'off'}")
            print(f"  LIMIT2    : {'ACTIVE' if enc.at_limit2 else 'off'}")
            print(f"  Index     : {'seen' if enc.index_seen else 'not seen'}")
            print(f"  Overflow  : {'YES' if enc.overflow else 'no'}")

        elif choice == "3":
            ppr  = int(input("  PPR [2000]: ") or 2000)
            lim1 = int(input("  Limit1 position [0]: ") or 0)
            lim2 = int(input("  Limit2 position [50000]: ") or 50000)
            pe   = input("  Push enable [Y/n]: ").strip().lower() != "n"
            thr  = int(input("  Push threshold counts [1]: ") or 1)
            client.set_encoder_config(ppr, lim1, lim2, pe, thr)
            cfg = client.get_encoder_config()
            print(f"  Verified: {cfg}")

        elif choice == "4":
            en = input("  Enable motor? [y/N]: ").strip().lower() == "y"
            client.motor_enable(en)

        elif choice == "5":
            d = input("  Direction CW? [y/N]: ").strip().lower() == "y"
            client.motor_set_direction(cw=d)

        elif choice == "6":
            freq  = int(input("  Frequency Hz [1000]: ") or 1000)
            duty  = int(float(input("  Duty % [50.0]: ") or 50.0) * 10)
            client.motor_set_pwm(freq, duty)

        elif choice == "7":
            st = client.motor_get_status()
            print(f"\n  Enabled  : {st.enabled}")
            print(f"  Running  : {st.running}")
            print(f"  Direction: {'CW' if st.dir_cw else 'CCW'}")
            print(f"  PWM freq : {st.pwm_freq} Hz")
            print(f"  PWM duty : {st.pwm_duty / 10.0:.1f} %")

        elif choice == "8":
            d = client.get_sht45()
            status = "OK" if d.ok else "ERROR"
            print(f"\n  Temperature : {d.temperature_c:.2f} °C")
            print(f"  Humidity    : {d.humidity_pct:.2f} %")
            print(f"  I2C Status  : {status}")

        elif choice == "9":
            d = client.get_adxl345()
            status = "OK" if d.ok else "ERROR"
            g = ((d.x_mg**2 + d.y_mg**2 + d.z_mg**2) ** 0.5) / 1000.0
            print(f"\n  X : {d.x_mg:+8.1f} mg")
            print(f"  Y : {d.y_mg:+8.1f} mg")
            print(f"  Z : {d.z_mg:+8.1f} mg")
            print(f"  |g|: {g:.3f} g")
            print(f"  I2C Status : {status}")

        elif choice == "10":
            print("\n  Monitoring push notifications — press Ctrl+C to stop\n")
            try:
                while True:
                    pn = client.get_push_notification(timeout=1.0)
                    if pn:
                        ts = time.strftime("%H:%M:%S")
                        print(f"  [{ts}] pos={pn['position']:+10d}  "
                              f"L1={'▲' if pn['limit1'] else '·'}  "
                              f"L2={'▲' if pn['limit2'] else '·'}  "
                              f"idx={'▲' if pn['index'] else '·'}")
            except KeyboardInterrupt:
                print("\n  (stopped)")

        elif choice == "11":
            suite = TestSuite(client)
            suite.run_all()

        elif choice == "12":
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
    parser = argparse.ArgumentParser(description="MCU Test Board Host — ModBus-RTU over RS-232")
    parser.add_argument("--port",  default="/dev/ttyUSB0", help="Serial port (default /dev/ttyUSB0)")
    parser.add_argument("--baud",  default=115200, type=int, help="Baud rate (default 115200)")
    parser.add_argument("--addr",  default=1, type=int, help="ModBus node address 1–7 (matches SW1)")
    parser.add_argument("--test",  action="store_true", help="Run automated test suite and exit")
    args = parser.parse_args()

    log.info("Opening %s @ %d baud, ModBus addr %d", args.port, args.baud, args.addr)
    client = MCUClient(port=args.port, baud=args.baud, node_addr=args.addr)

    try:
        if args.test:
            suite = TestSuite(client)
            ok = suite.run_all()
            sys.exit(0 if ok else 1)
        else:
            interactive_cli(client)
    finally:
        client.close()


if __name__ == "__main__":
    main()
