#!/usr/bin/env python3
"""
gps_simulator.py
Raspberry Pi GPS NMEA simulator

Outputs real UTC time in GNRMC, GNGGA, and GNGSV sentences at 1Hz
over UART, and toggles a GPIO pin for 1PPS.

Wiring:
  Pi UART TX  (GPIO14, pin 8)  --> STM32 LPUART1 RX (PB10)
  Pi GPIO17   (pin 11)         --> STM32 GPS_1PPS   (PB9)
  Pi GND      (pin 6)          --> STM32 GND

Usage:
  pip3 install pyserial RPi.GPIO
  python3 gps_simulator.py
  python3 gps_simulator.py --port /dev/ttyAMA0 --baud 115200 --pps-pin 17
"""

import serial
import RPi.GPIO as GPIO
import datetime
import time
import argparse
import math

# -----------------------------------------------------------------------
# NMEA checksum
# -----------------------------------------------------------------------
def nmea_checksum(sentence: str) -> str:
    """Calculate NMEA checksum — XOR of all chars between $ and *"""
    crc = 0
    for c in sentence:
        crc ^= ord(c)
    return f"{crc:02X}"

def nmea_wrap(sentence: str) -> str:
    """Wrap sentence with $ prefix and *XX checksum and CRLF"""
    cs = nmea_checksum(sentence)
    return f"${sentence}*{cs}\r\n"

# -----------------------------------------------------------------------
# Fake satellite data — 8 GPS satellites at plausible positions
# -----------------------------------------------------------------------
FAKE_SATELLITES = [
    # (PRN, elevation_deg, azimuth_deg, snr_dbhz)
    (1,  45, 195, 42),
    (3,  72,  51, 38),
    (4,  28, 304, 35),
    (7,  15, 162, 30),
    (8,  62,  88, 45),
    (11, 33, 270, 28),
    (14, 51, 340, 40),
    (17, 20, 120, 25),
]

# -----------------------------------------------------------------------
# Fake position — Soccoro, NM (near the VLA, close to your dish site)
# -----------------------------------------------------------------------
FAKE_LAT_DEG  = 34.0784      # degrees N
FAKE_LON_DEG  = -107.6184    # degrees W
FAKE_ALT_M    = 1477.0       # meters

def deg_to_nmea_lat(deg: float) -> tuple[str, str]:
    """Convert decimal degrees to NMEA ddmm.mmmm format"""
    d = int(abs(deg))
    m = (abs(deg) - d) * 60.0
    return f"{d:02d}{m:07.4f}", "N" if deg >= 0 else "S"

def deg_to_nmea_lon(deg: float) -> tuple[str, str]:
    """Convert decimal degrees to NMEA dddmm.mmmm format"""
    d = int(abs(deg))
    m = (abs(deg) - d) * 60.0
    return f"{d:03d}{m:07.4f}", "E" if deg >= 0 else "W"

# -----------------------------------------------------------------------
# Sentence builders
# -----------------------------------------------------------------------
def build_gnrmc(utc: datetime.datetime) -> str:
    """
    $GNRMC — Recommended Minimum Specific GNSS Data
    Fields: time, status, lat, N/S, lon, E/W, speed, course, date, mag_var, mode, navStatus
    """
    t = utc.strftime("%H%M%S.00")
    d = utc.strftime("%d%m%y")
    lat, ns = deg_to_nmea_lat(FAKE_LAT_DEG)
    lon, ew = deg_to_nmea_lon(FAKE_LON_DEG)
    sentence = f"GNRMC,{t},A,{lat},{ns},{lon},{ew},0.000,0.00,{d},,,A,V"
    return nmea_wrap(sentence)

def build_gngga(utc: datetime.datetime) -> str:
    """
    $GNGGA — Global Positioning System Fix Data
    Fields: time, lat, N/S, lon, E/W, fix_quality, num_sats, hdop, alt, M, geoid, M, dgps_age, dgps_id
    """
    t = utc.strftime("%H%M%S.00")
    lat, ns = deg_to_nmea_lat(FAKE_LAT_DEG)
    lon, ew = deg_to_nmea_lon(FAKE_LON_DEG)
    num_sats = len(FAKE_SATELLITES)
    sentence = (f"GNGGA,{t},{lat},{ns},{lon},{ew},"
                f"1,{num_sats:02d},1.2,{FAKE_ALT_M:.1f},M,0.0,M,,")
    return nmea_wrap(sentence)

def build_gngsv() -> list[str]:
    """
    $GNGSV — GNSS Satellites in View
    4 satellites per sentence, multiple sentences if more than 4.
    """
    sats = FAKE_SATELLITES
    total_sats = len(sats)
    sentences_needed = math.ceil(total_sats / 4)
    sentences = []

    for s_idx in range(sentences_needed):
        sentence_num = s_idx + 1
        sat_group = sats[s_idx*4 : s_idx*4 + 4]
        fields = f"GNGSV,{sentences_needed},{sentence_num},{total_sats:02d}"
        for prn, elev, azim, snr in sat_group:
            fields += f",{prn:02d},{elev:02d},{azim:03d},{snr:02d}"
        sentences.append(nmea_wrap(fields))

    return sentences

# -----------------------------------------------------------------------
# 1PPS pulse
# -----------------------------------------------------------------------
def pulse_1pps(pin: int, pulse_ms: int = 100):
    """Output a 100ms HIGH pulse on the 1PPS GPIO pin"""
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(pulse_ms / 1000.0)
    GPIO.output(pin, GPIO.LOW)

# -----------------------------------------------------------------------
# Main loop
# -----------------------------------------------------------------------
def run(port: str, baud: int, pps_pin: int):
    print(f"GPS Simulator starting")
    print(f"  UART: {port} at {baud} baud")
    print(f"  1PPS: GPIO{pps_pin}")
    print(f"  Position: {FAKE_LAT_DEG:.4f}N, {abs(FAKE_LON_DEG):.4f}W")
    print(f"  Satellites: {len(FAKE_SATELLITES)}")
    print(f"  Press Ctrl+C to stop\n")

    # Setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pps_pin, GPIO.OUT, initial=GPIO.LOW)

    # Open serial port
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE
    )

    try:
        while True:
            # Get current UTC time
            utc = datetime.datetime.now(datetime.timezone.utc)

            # 1. Fire 1PPS pulse first — this is the precise timing edge
            #    The NMEA sentences follow after, referencing this second
            import threading
            pps_thread = threading.Thread(
                target=pulse_1pps, args=(pps_pin,), daemon=True)
            pps_thread.start()

            # 2. Send NMEA sentences for this second
            rmc = build_gnrmc(utc)
            gga = build_gngga(utc)
            gsv = build_gngsv()

            ser.write(rmc.encode())
            ser.write(gga.encode())
            for s in gsv:
                ser.write(s.encode())

            # Print to console for monitoring
            print(f"  {utc.strftime('%H:%M:%S')} UTC  |  "
                  f"1PPS fired  |  "
                  f"{len(gsv)+2} sentences sent")

            # 3. Wait for next second boundary
            #    Align to the next whole second for accurate 1Hz timing
            now = time.time()
            sleep_time = 1.0 - (now % 1.0)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        GPIO.output(pps_pin, GPIO.LOW)
        GPIO.cleanup()
        ser.close()

# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="GPS NMEA simulator for Raspberry Pi")
    parser.add_argument("--port",    default="/dev/ttyAMA0", help="UART port (default: /dev/ttyAMA0)")
    parser.add_argument("--baud",    type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--pps-pin", type=int, default=17,    help="GPIO pin for 1PPS (default: 17)")
    args = parser.parse_args()

    run(args.port, args.baud, args.pps_pin)
