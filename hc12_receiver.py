import serial

def hc12_receiver(port='/dev/ttyUSB0', baudrate=9600):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"HC-12 Receiver started on {port} at {baudrate} baud.")
        while True:
            if ser.in_waiting:
                data = ser.readline().decode(errors='replace').strip()
                if data:
                    print(f"Received: {data}")
    except Exception as e:
        print(f"Receiver error: {e}")
    finally:
        try:
            ser.close()
        except:
            pass

if __name__ == "__main__":
    hc12_receiver()
