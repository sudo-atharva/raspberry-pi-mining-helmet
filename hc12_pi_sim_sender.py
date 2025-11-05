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
    """Find a default serial port from common candidates."""
    candidates = ["/dev/ttyUSB0", "/dev/ttyS0", "/dev/ttyAMA0"]
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
    """Open serial connection with proper settings."""
    return serial.Serial(port, baud, timeout=1)


def make_telemetry(t0: float, i: int):
    """Generate simulated telemetry data in CSV format."""
    # Create plausible changing values
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

    status = "AWAKE" if (i % 100) < 80 else "DROWSY"  # sometimes drowsy

    # Compact CSV: <S>,<lat>,<lon>,<temp>,<hum>,<methane>,<co>,<lpg>,<smoke>,<air_quality>
    line = (
        f"{status[0]},{lat:.4f},{lon:.4f},{temperature:.1f},{humidity:.1f},"
        f"{methane:.0f},{co:.0f},{lpg:.0f},{smoke:.0f},{air_quality:.0f}"
    )
    return line


def make_alert(i: int):
    """Generate occasional alert messages in JSON format."""
    # Produce alerts every 30 iterations
    if i % 30 != 0:
        return None
    
    alert_type = random.choice(["DROWSY", "NO_FACE"])
    lat = 19.0760 + 0.0001 * math.sin(i / 120.0)
    lon = 72.8777 + 0.0001 * math.cos(i / 120.0)
    sensors = {
        "temperature": round(26.0 + 2.0 * math.sin(i / 30.0), 1),
        "humidity": round(55.0 + 5.0 * math.cos(i / 40.0), 1),
        "methane": round(150.0 + random.uniform(-10, 10), 0),
        "co": round(15.0 + random.uniform(-2, 2), 0),
        "lpg": round(70.0 + random.uniform(-5, 5), 0),
        "smoke": round(45.0 + random.uniform(-4, 4), 0),
    }
    obj = {
        "type": "ALERT",
        "alert": alert_type,
        "gps": {"lat": round(lat, 4), "lon": round(lon, 4)},
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

    interval = 1.0 / max(0.1, args.hz)  # Fixed: was 5.0 / hz
    i = 0
    t0 = time.time()
    
    try:
        while True:
            # Send telemetry
            line = make_telemetry(t0, i)
            try:
                ser.write((line + "\n").encode("utf-8"))
                ser.flush()
                print(f"TX: {line}")
            except Exception as e:
                print(f"Write error: {e}")
                break

            # Send alerts occasionally
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
        print("\nStopping...")
    finally:
        try:
            ser.close()
            print("Serial port closed.")
        except Exception:
            pass


if __name__ == "__main__":
    main()