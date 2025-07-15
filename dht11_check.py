import time

# Use adafruit-circuitpython-dht for DHT22
try:
    import board
    import adafruit_dht
except ImportError:
    board = None
    adafruit_dht = None

def check_dht11():
    if not adafruit_dht or not board:
        print("adafruit-circuitpython-dht or board library not found.")
        return
    dhtDevice = adafruit_dht.DHT22(board.D4)
    print("Reading DHT22 sensor...")
    for _ in range(5):
        try:
            temperature = dhtDevice.temperature
            humidity = dhtDevice.humidity
            print(f"Temperature: {temperature}C, Humidity: {humidity}%")
        except Exception:
            print("Sensor read error.")
        time.sleep(2)

if __name__ == "__main__":
    check_dht11()
