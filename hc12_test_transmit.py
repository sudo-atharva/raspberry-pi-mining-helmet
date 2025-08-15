#!/usr/bin/env python3
import serial
import time
import os

def hc12_test_transmit(port='/dev/ttyUSB0', baudrate=9600):
    try:
        ser = serial.Serial(
            port,
            baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        print(f"HC-12 Test Transmitter started on {port}")
        
        count = 0
        while True:
            try:
                message = f"Test message {count}\n"
                ser.write(message.encode())
                print(f"Sent: {message.strip()}")
                count += 1
                time.sleep(2)  # Send every 2 seconds
                
            except KeyboardInterrupt:
                print("\nStopping transmission...")
                break
                
    except Exception as e:
        print(f"Transmitter error: {e}")
    finally:
        try:
            ser.close()
            print("Port closed")
        except:
            pass

if __name__ == "__main__":
    hc12_test_transmit()
