import serial
import os
import time

def check_gps():
    if not os.path.exists('/dev/ttyS0'):
        print("GPS UART /dev/ttyS0 not found.")
        return
    try:
        gps_serial = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=2)
        print("Reading GPS data...")
        for _ in range(10):
            line = gps_serial.readline().decode('ascii', errors='replace')
            if line.startswith('$GPGGA'):
                print("NMEA Sentence:", line.strip())
        gps_serial.close()
    except Exception as e:
        print("GPS error:", e)

if __name__ == "__main__":
    check_gps()
