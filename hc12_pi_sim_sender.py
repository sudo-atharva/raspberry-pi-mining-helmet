#!/usr/bin/env python3
"""
HC-12 Raspberry Pi Simulator Sender

Purpose:
- Simulate Mining Helmet telemetry and alerts without any sensors/camera.
- Sends once-per-second telemetry lines identical in format to main.py.
- Occasionally sends JSON ALERT messages like main.py's send_alert().

Usage:
  python3 hc12_pi_sim_sender.py --port /dev/ttyUSB0 --baud 9600

Notes:
- Auto-detects a likely serial port if --port not provided.
- Prints what it sends to stdout for debugging.
- Designed to run on Raspberry Pi but works on any Linux.

Dependencies:
  pip install pyserial
"""

import argparse
import json
import math
import os
import random
import sys
import time
from datetime import datetime

import serial
import serial.tools.list_ports

DEFAULT_BAUD = 9600


def find_default_port():
    # Try common candidates and detected ports
    candidates = ["/dev/ttyUSB0"]
    try:
        for p in serial.tools.list_ports.comports():
            if p.device not in candidates:
                candidates.append(p.device)
    except Exception:
        pass
    for port in candidates:
        try:
            if os.path.exists(port):
                return port
        except Exception:
            continue
    return None


def open_serial(port: str, baud: int) -> serial.Serial:
    return serial.Serial(port, baud, timeout=1)


def make_telemetry(t0: float, i: int):
    # Create plausible changing values
    # GPS: small walk around a base coordinate
    base_lat, base_lon = 19.0760, 72.8777  # Mumbai (example)
    lat = base_lat + 0.0001 * math.sin(i / 120.0)
    lon = base_lon + 0.0001 * math.cos(i / 120.0)

    # Environmental
    temperature = 26.0 + 2.0 * math.sin(i / 30.0)
    humidity = 55.0 + 5.0 * math.cos(i / 40.0)

    methane = max(0.0, 120.0 + 30.0 * math.sin(i / 25.0) + random.uniform(-5, 5))
    co = max(0.0, 12.0 + 3.0 * math.sin(i / 18.0) + random.uniform(-1, 1))
    lpg = max(0.0, 60.0 + 10.0 * math.sin(i / 22.0) + random.uniform(-3, 3))
    smoke = max(0.0, 40.0 + 8.0 * math.sin(i / 15.0) + random.uniform(-2, 2))

    air_quality = max(0.0, min(100.0, 100.0 - (methane / 10.0) - (co / 5.0) - (lpg / 10.0) - (smoke / 10.0)))

    danger = "SAFE"
    reasons = []
    if methane > 1000:
        danger = "CRITICAL"; reasons.append("HIGH_METHANE")
    if co > 50:
        danger = "CRITICAL"; reasons.append("HIGH_CO")
    if lpg > 1000:
        danger = "CRITICAL"; reasons.append("HIGH_LPG")
    if smoke > 500 and danger != "CRITICAL":
        danger = "WARNING"; reasons.append("SMOKE_DETECTED")
    if temperature > 35 and danger != "CRITICAL":
        danger = "WARNING"; reasons.append("HIGH_TEMP")

    status = "AWAKE" if (i % 100) < 80 else "DROWSY"  # sometimes drowsy

    line = (
        f"{status},"
        f"GPS:({lat:.6f},{lon:.6f}),"
        f"TEMP:{temperature:.1f},"
        f"HUM:{humidity:.1f},"
        f"METHANE:{methane:.1f},"
        f"CO:{co:.1f},"
        f"LPG:{lpg:.1f},"
        f"SMOKE:{smoke:.1f},"
        f"AIR_QUALITY:{air_quality:.1f},"
        f"DANGER:{danger},"
        f"REASONS:{','.join(reasons) if reasons else 'NONE'},"
        f"TIME:{datetime.now().strftime('%H:%M:%S')}"
    )
    return line


def make_alert(i: int):
    # Occasionally produce alerts similar to HelmetSafetySystem.send_alert
    alert_type = random.choice(["DROWSY", "NO_FACE"]) if (i % 30 == 0) else None
    if not alert_type:
        return None
    lat = 19.0760 + 0.0001 * math.sin(i / 120.0)
    lon = 72.8777 + 0.0001 * math.cos(i / 120.0)
    sensors = {
        "temperature": 26.0 + 2.0 * math.sin(i / 30.0),
        "humidity": 55.0 + 5.0 * math.cos(i / 40.0),
        "methane": 150.0 + random.uniform(-10, 10),
        "co": 15.0 + random.uniform(-2, 2),
        "lpg": 70.0 + random.uniform(-5, 5),
        "smoke": 45.0 + random.uniform(-4, 4),
    }
    obj = {
        "type": "ALERT",
        "alert": alert_type,
        "gps": {"lat": lat, "lon": lon},
        "sensors": sensors,
        "motion": {
            "accel": {"x": 0.0, "y": 0.0, "z": 1.0},
            "gyro": {"x": 0.0, "y": 0.0, "z": 0.0},
        },
        "time": datetime.now().strftime('%H:%M:%S')
    }
    return json.dumps(obj)


def main():
    parser = argparse.ArgumentParser(description="HC-12 Raspberry Pi Simulator Sender")
    parser.add_argument("--port", default=None, help="Serial port for HC-12 (e.g., /dev/ttyUSB0, /dev/ttyS0, /dev/ttyAMA0)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate")
    parser.add_argument("--hz", type=float, default=1.0, help="Send frequency in Hz (default 1 per second)")
    args = parser.parse_args()

    port = args.port or find_default_port()
    if not port:
        print("ERROR: No serial port found. Specify with --port.")
        sys.exit(1)

    try:
        ser = open_serial(port, args.baud)
        print(f"Opened {port} @ {args.baud} baud")
    except Exception as e:
        print(f"Failed to open {port}: {e}")
        sys.exit(2)

    interval = 5.0 / max(0.1, args.hz)
    i = 0
    t0 = time.time()
    try:
        while True:
            line = make_telemetry(t0, i)
            try:
                ser.write((line + "\n").encode("utf-8"))
                ser.flush()
                print(f"TX: {line}")
            except Exception as e:
                print(f"Write error: {e}")
                break

            alert = make_alert(i)
            if alert:
                try:
                    ser.write((alert + "\n").encode("utf-8"))
                    ser.flush()
                    print(f"TX: {alert}")
                except Exception as e:
                    print(f"Write error(alert): {e}")
                    break

            i += 1
            time.sleep(interval)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
