import time
try:
    import Adafruit_DHT
except ImportError:
    Adafruit_DHT = None

def check_dht11():
    if not Adafruit_DHT:
        print("Adafruit_DHT library not found.")
        return
    DHT_SENSOR = Adafruit_DHT.DHT22
    DHT_PIN = 4
    print("Reading DHT11 sensor...")
    for _ in range(5):
        humidity, temperature = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)
        print(f"Temperature: {temperature}C, Humidity: {humidity}%")
        time.sleep(2)

if __name__ == "__main__":
    check_dht11()
