import serial
import time

def hc12_transmitter(port='/dev/ttyUSB0', baudrate=9600):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"HC-12 Transmitter started on {port} at {baudrate} baud.")
        count = 0
        while True:
            msg = f"Hello HC-12 #{count}"
            ser.write((msg + '\n').encode())
            print(f"Sent: {msg}")
            count += 1
            time.sleep(2)
    except Exception as e:
        print(f"Transmitter error: {e}")
    finally:
        try:
            ser.close()
        except:
            pass

if __name__ == "__main__":
    hc12_transmitter()
