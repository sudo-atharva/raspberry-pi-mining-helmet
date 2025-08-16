import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Create I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create ADC object
ads = ADS.ADS1115(i2c)

# Single-ended reading on channel 0
chan = AnalogIn(ads, ADS.P0)

print("ADS1115 detected at I2C address:", hex(ads.i2c_device.device_address))
print("Voltage: {:.3f} V".format(chan.voltage))
