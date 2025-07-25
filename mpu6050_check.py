import time
try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

def check_mpu6050():
    if not SMBus:
        print("smbus2 library not found.")
        return
    try:
        bus = SMBus(1)
        bus.write_byte_data(0x68, 0x6B, 0)  # Wake up MPU6050
        def read_word(reg):
            h = bus.read_byte_data(0x68, reg)
            l = bus.read_byte_data(0x68, reg+1)
            val = (h << 8) + l
            return val - 65536 if val >= 0x8000 else val
        print("Reading MPU6050 sensor...")
        for _ in range(10):
            accel_x = read_word(0x3B) / 16384.0
            accel_y = read_word(0x3D) / 16384.0
            accel_z = read_word(0x3F) / 16384.0
            gyro_x = read_word(0x43) / 131.0
            gyro_y = read_word(0x45) / 131.0
            gyro_z = read_word(0x47) / 131.0
            print(f"Accel: x={accel_x:.2f}, y={accel_y:.2f}, z={accel_z:.2f} | Gyro: x={gyro_x:.2f}, y={gyro_y:.2f}, z={gyro_z:.2f}")
            time.sleep(0.5)
        bus.close()
    except Exception as e:
        print("MPU6050 error:", e)

if __name__ == "__main__":
    check_mpu6050()
