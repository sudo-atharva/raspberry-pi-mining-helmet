import time
try:
    import board
    import busio
    from adafruit_ads1x15.ads1115 import ADS1115
    from adafruit_ads1x15.analog_in import AnalogIn
except ImportError:
    board = None
    busio = None
    ADS1115 = None
    AnalogIn = None


def check_ads1115_mq():
    if not board or not busio or not ADS1115 or not AnalogIn:
        print("Required libraries for ADS1115 not found. Please install adafruit-circuitpython-ads1x15 and adafruit-blinka.")
        return

    # Initialize I2C bus and ADS1115
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS1115(i2c)

    # MQ sensors typically connect to A0, A1, etc.
    mq9_channel = AnalogIn(ads, 0)  # A0
    mq_channel = AnalogIn(ads, 1)   # A1

    print("Reading MQ9 and MQ sensor values from ADS1115...")
    try:
        for _ in range(10):
            mq9_voltage = mq9_channel.voltage
            mq_voltage = mq_channel.voltage
            print(f"MQ9 (A0) Voltage: {mq9_voltage:.3f} V, MQ (A1) Voltage: {mq_voltage:.3f} V")
            time.sleep(1)
    except Exception as e:
        print(f"Error reading from ADS1115: {e}")

if __name__ == "__main__":
    check_ads1115_mq()
