#!/usr/bin/env python3
"""
rs485_test.py
Simple RS485 ping test - sends PING <counter>, expects ACK <counter>
Usage: python3 rs485_test.py --port /dev/ttyUSB0 --baud 115200
"""

import serial
import argparse
import time
import sys

def run_test(port, baud, delay):
    print(f"Opening {port} at {baud} baud...")
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0
        )
    except serial.SerialException as e:
        print(f"ERROR: Could not open port: {e}")
        sys.exit(1)

    print(f"Port open. Starting ping loop. Ctrl+C to stop.\n")

    counter = 0
    pass_count = 0
    fail_count = 0

    try:
        while True:
            # --- Send PING ---
            tx = f"PING {counter}\n"
            ser.write(tx.encode())
            print(f"TX: {tx.strip()}", end="  ")

            # --- Read response ---
            rx = ser.readline()
            if not rx:
                print(f"TIMEOUT - no response")
                fail_count += 1
                counter += 1
                time.sleep(delay)
                continue

            rx_str = rx.decode(errors='replace').strip()

            # --- Parse ACK <counter> ---
            parts = rx_str.split()
            if len(parts) == 2 and parts[0] == "ACK":
                try:
                    rx_counter = int(parts[1])
                    if rx_counter == counter:
                        print(f"RX: {rx_str}  OK")
                        pass_count += 1
                    else:
                        print(f"RX: {rx_str}  COUNTER MISMATCH (expected {counter})")
                        fail_count += 1
                except ValueError:
                    print(f"RX: {rx_str}  BAD FORMAT")
                    fail_count += 1
            else:
                print(f"RX: {rx_str}  UNEXPECTED RESPONSE")
                fail_count += 1

            counter += 1
            time.sleep(delay)

    except KeyboardInterrupt:
        print(f"\n\nStopped.")
        total = pass_count + fail_count
        print(f"Results: {pass_count}/{total} passed, {fail_count}/{total} failed")

    finally:
        ser.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="RS485 ping test")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--delay", type=float, default=0.5, help="Delay between pings in seconds (default: 0.5)")
    args = parser.parse_args()

    run_test(args.port, args.baud, args.delay)
