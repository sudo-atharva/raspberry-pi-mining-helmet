try:
    import spidev
except ImportError:
    spidev = None
import time

MQ_CHANNEL = 0  # MCP3008 channel for MQ sensor

def read_mcp3008(channel):
    if not spidev:
        return -1
    spi = spidev.SpiDev()
    try:
        spi.open(0, 0)
        spi.max_speed_hz = 1350000
        cmd = [1, (8 + channel) << 4, 0]
        r = spi.xfer2(cmd)
        value = ((r[1] & 3) << 8) + r[2]
        spi.close()
        return value
    except Exception:
        spi.close()
        return -1

def check_mq():
    print("Reading MQ gas sensor via MCP3008...")
    for _ in range(10):
        value = read_mcp3008(MQ_CHANNEL)
        print(f"MQ Sensor ADC Value: {value}")
        time.sleep(1)

if __name__ == "__main__":
    check_mq()
