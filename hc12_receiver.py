import serial
import time
import os
import sys

def check_port(port):
    """Check if serial port exists"""
    return os.path.exists(port)

def configure_hc12(ser):
    """Configure HC-12 module"""
    try:
        # Enter AT mode
        print("Configuring HC-12...")
        ser.write(b'AT\r\n')
        time.sleep(0.1)
        response = ser.read_all()
        print(f"AT response: {response}")
        
        # Set channel (optional, adjust if needed)
        ser.write(b'AT+C001\r\n')  # Channel 1
        time.sleep(0.1)
        response = ser.read_all()
        print(f"Channel set response: {response}")
        
        # Set power level (optional, adjust if needed)
        ser.write(b'AT+P8\r\n')  # Max power
        time.sleep(0.1)
        response = ser.read_all()
        print(f"Power set response: {response}")
        
        print("HC-12 configuration complete")
    except Exception as e:
        print(f"Configuration error: {e}")

def hc12_receiver(baudrate=9600, debug=True):
    # List of possible ports
    ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyAMA0', '/dev/ttyS0']
    ser = None
    
    # Try each port
    for port in ports:
        if not check_port(port):
            print(f"Port {port} not found, trying next...")
            continue
            
        try:
            ser = serial.Serial(
                port,
                baudrate,
                timeout=1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"HC-12 Receiver started on {port} at {baudrate} baud.")
            configure_hc12(ser)  # Configure the HC-12
            break
        except serial.SerialException as e:
            print(f"Error opening {port}: {e}")
            continue
    
    if not ser:
        print("Could not open any serial port. Please check connections.")
        return
        
    print("Waiting for data... Press Ctrl+C to exit")
    try:
        while True:
            try:
                if ser.in_waiting:
                    # Try to read raw bytes first
                    raw_data = ser.read_all()
                    if debug and raw_data:
                        print(f"Raw data received (hex): {raw_data.hex()}")
                    
                    # Try to decode as text
                    try:
                        data = raw_data.decode(errors='replace').strip()
                        if data:
                            print(f"Decoded data: {data}")
                    except Exception as e:
                        if debug:
                            print(f"Decode error: {e}")
                
                # Send test message every 5 seconds if no data received
                if debug and time.time() % 5 < 0.1:
                    test_msg = b"TEST\r\n"
                    ser.write(test_msg)
                    print("Sent test message")
                
                time.sleep(0.1)  # Prevent CPU overuse
                
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                break
            except Exception as e:
                print(f"Other error: {e}")
                time.sleep(1)  # Wait before retrying
    finally:
        if ser:
            try:
                ser.close()
                print("Serial port closed")
            except:
                pass

if __name__ == "__main__":
    hc12_receiver()
