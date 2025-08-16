import serial
import time

def parse_gpgga(sentence):
    """
    Parse GPGGA sentence and return dictionary with data
    """
    parts = sentence.split(',')
    fix = parts[6]
    num_sat = parts[7]
    if fix == '0':
        return None  # No valid fix

    # Latitude
    raw_lat = parts[2]
    lat_dir = parts[3]
    # Longitude
    raw_lon = parts[4]
    lon_dir = parts[5]

    # Convert format (ddmm.mmmm) to decimal degrees
    def convert(raw, direction):
        degrees = float(raw[:2])
        minutes = float(raw[2:])
        decimal = degrees + minutes / 60.0
        if direction in ['S', 'W']:
            decimal = -decimal
        return decimal

    latitude = convert(raw_lat, lat_dir)
    longitude = convert(raw_lon, lon_dir)

    return {
        "fix": fix,
        "num_satellites": num_sat,
        "latitude": latitude,
        "longitude": longitude
    }

def read_gps():
    try:
        gps_serial = serial.Serial('/dev/serial0', baudrate=9600, timeout=1)
        print("Waiting for GPS fix...")
        while True:
            sentence = gps_serial.readline().decode('ascii', errors='replace')
            if sentence.startswith('$GPGGA'):
                data = parse_gpgga(sentence)
                if data:
                    print("GPS lock acquired!")
                    print(f"Latitude : {data['latitude']}")
                    print(f"Longitude: {data['longitude']}")
                    print(f"Satellites in use: {data['num_satellites']}")
                    break   # Exit after receiving one valid fix
                else:
                    print("No fix yet...")
        gps_serial.close()
    except Exception as e:
        print("GPS error:", e)

if __name__ == "__main__":
    read_gps()
