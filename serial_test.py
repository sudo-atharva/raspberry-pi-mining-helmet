#!/usr/bin/env python3
import serial
import time

def test_serial(port='/dev/ttyUSB0', baudrate=9600):
    try:
        # Open serial port with more explicit settings
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
            timeout=1
        )
        
        print(f"Serial port {port} opened successfully")
        print(f"Port settings: {ser.get_settings()}")
        
        # Test if port is really open
        if ser.is_open:
            print("Port is confirmed open")
        
        print("\nTesting serial loopback - sending 'TEST'")
        test_data = b"TEST\r\n"
        
        # Flush any existing data
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Write data
        print(f"Writing {len(test_data)} bytes...")
        bytes_written = ser.write(test_data)
        print(f"Wrote {bytes_written} bytes")
        
        # Wait a bit
        time.sleep(0.5)
        
        # Check if any data received
        if ser.in_waiting:
            received = ser.read_all()
            print(f"\nReceived {len(received)} bytes:")
            print(f"As hex: {received.hex()}")
            print(f"As string: {received.decode(errors='replace')}")
        else:
            print("\nNo data received")
            
        print("\nSerial port information:")
        print(f"CD (Carrier Detect): {ser.cd}")
        print(f"CTS (Clear To Send): {ser.cts}")
        print(f"DSR (Data Set Ready): {ser.dsr}")
        print(f"RI (Ring Indicator): {ser.ri}")
        
        return True
        
    except Exception as e:
        print(f"\nError: {type(e).__name__}: {str(e)}")
        return False
        
    finally:
        try:
            ser.close()
            print("\nPort closed")
        except:
            pass

if __name__ == "__main__":
    # Try multiple baud rates
    BAUD_RATES = [9600, 4800, 2400, 1200]
    PORTS = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyAMA0', '/dev/ttyS0']
    
    print("Available serial ports:")
    import glob
    ports = glob.glob('/dev/tty[A-Za-z]*')
    for port in ports:
        print(f"  {port}")
    
    success = False
    for port in PORTS:
        if port not in ports:
            print(f"\nSkipping {port} - not available")
            continue
            
        print(f"\nTesting {port}")
        for baud in BAUD_RATES:
            print(f"\nTrying {baud} baud...")
            if test_serial(port, baud):
                success = True
                break
        if success:
            break
            
    if not success:
        print("\nCould not establish serial communication on any port/baud combination")
    else:
        print(f"\nSuccessful communication achieved on {port} at {baud} baud")
