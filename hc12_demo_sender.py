import serial
import time
import random

# Change this to your HC-12 serial port (same as used in boss_monitor_gui.py)
HC12_PORT = '/dev/ttyUSB0'  # Update as needed
HC12_BAUDRATE = 9600

# Example message format expected by boss_monitor_gui.py
MESSAGE_TEMPLATE = (
    "{status},GPS:({lat:.6f},{lon:.6f}),TEMP:{temp:.1f},HUM:{hum:.1f},METHANE:{methane:.1f},CO:{co:.1f},LPG:{lpg:.1f},SMOKE:{smoke:.1f},AIR_QUALITY:{air_quality:.1f},DANGER:{danger},REASONS:{reasons},TIME:{time}\n"
)

STATUSES = ['AWAKE', 'DROWSY', 'ALERT', 'NO_FACE']
DANGERS = ['SAFE', 'WARNING', 'CRITICAL']
REASONS = ['NONE', 'HIGH_METHANE', 'HIGH_CO', 'HIGH_LPG', 'SMOKE_DETECTED']


def generate_dummy_message():
    status = random.choice(STATUSES)
    lat = random.uniform(-90, 90)
    lon = random.uniform(-180, 180)
    temp = random.uniform(15, 40)
    hum = random.uniform(20, 90)
    methane = random.uniform(0, 2000)
    co = random.uniform(0, 100)
    lpg = random.uniform(0, 2000)
    smoke = random.uniform(0, 1000)
    air_quality = random.uniform(0, 100)
    danger = random.choice(DANGERS)
    reasons = ','.join(random.sample(REASONS, random.randint(1, 2)))
    now = time.strftime('%H:%M:%S')
    return MESSAGE_TEMPLATE.format(
        status=status,
        lat=lat,
        lon=lon,
        temp=temp,
        hum=hum,
        methane=methane,
        co=co,
        lpg=lpg,
        smoke=smoke,
        air_quality=air_quality,
        danger=danger,
        reasons=reasons,
        time=now
    )


def main():
    print(f"Connecting to HC-12 on {HC12_PORT} at {HC12_BAUDRATE} baud...")
    try:
        ser = serial.Serial(HC12_PORT, HC12_BAUDRATE, timeout=1)
        print("Connected. Sending demo messages...")
        while True:
            msg = generate_dummy_message()
            print(f"Sending: {msg.strip()}")
            ser.write(msg.encode('utf-8'))
            ser.flush()
            time.sleep(2)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            ser.close()
        except:
            pass

if __name__ == "__main__":
    main()
