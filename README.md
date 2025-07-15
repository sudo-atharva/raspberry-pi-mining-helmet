

# Mining Worker & Explorer Safety Helmet: Conceptual Design

This project is a conceptual design for a smart safety helmet for mining workers and explorers. It combines drowsiness detection, helmet tilt monitoring, environmental sensing, and emergency alerting using Raspberry Pi 4 and multiple sensors. All data is sent wirelessly via UART (HC-12) to a remote device for real-time monitoring and rescue operations.

---




## Features & Functionalities

- **Gyro Calibration Button (GPIO 17):** Allows the helmet to set its current orientation as zero. Essential for adapting to different head positions and environments.
- **Helmet Tilt Detection:** If the helmet is tilted more than 90° for over 5 seconds (using MPU6050), the system activates the camera and sets GPIO 27 high to turn on an LED. This indicates a possible fall or collapse.
- **Conditional Camera Activation:** OpenCV drowsiness/face detection only runs when the helmet is tilted. This saves power and processing, and only monitors when needed.
- **Collapse Detection & Emergency Alert:** If no face is detected when the helmet is tilted, the system sends a "COLLAPSED" alert and GPS coordinates via HC-12, spamming until a face is detected again. This helps rescuers locate and respond quickly.
- **MQ Gas Sensor Integration (e.g., MQ-2, MQ-7, MQ-135):** If any harmful gas is detected above a threshold, the system sets GPIO 22 high to alert the miner (buzzer/LED) and sends the gas type, percentage, and GPS coordinates via HC-12. This provides real-time hazardous gas monitoring and alerting. The code supports analog MQ sensors via MCP3008 (SPI), and can be extended for digital sensors.
- **Robust Sensor Handling:** The code runs even if some sensors or UART are missing. All sensor values default to -1 if unavailable, ensuring the helmet remains operational in degraded mode.
- **Automatic Camera Selection:** The system auto-selects between USB webcam and PiCamera2 for maximum compatibility.
- **Optimized Performance:** Frame resizing, reduced blocking, and face detection interval logic allow higher FPS and lower CPU usage on Raspberry Pi 4.
- **Debug Output:** The code prints diagnostic information for face/eye detection, EAR values, and FPS for easy troubleshooting.
- **Normal Operation:** When the helmet is upright, only environmental and motion data (temperature, humidity, acceleration, gyro, GPS) are sent via HC-12. The camera remains off, saving battery and reducing distraction.

---

## Quick Start



### 1. Hardware Required

- **Raspberry Pi 4 (4GB RAM recommended)**
- **USB webcam or Pi Camera (Picamera2)**
- **DHT22 sensor** (GPIO 4 for data)
- **MPU6050** (I2C)
- **NEO-6M GPS** (UART, /dev/ttyS0)
- **HC-12 UART module** (UART, /dev/ttyUSB0)
- **Button for Gyro Calibration** (GPIO 17)
- **LED Indicator** (GPIO 27)
- **MQ Gas Sensor** (Analog input via ADC or digital pin, e.g., GPIO 22 for alert)



### 2. GPIO Pinout & Significance

- **GPIO 4:** DHT22 data pin (environmental sensing)
- **GPIO 17:** Button input for gyro calibration (worker can reset helmet orientation)
- **GPIO 27:** LED output (lights up when helmet is tilted/camera is active)
- **GPIO 22:** MQ gas sensor alert output (buzzer/LED for hazardous gas)

### 3. Software Setup


Install dependencies (for both USB webcam and Pi Camera support):

```sh
sudo apt-get update
sudo apt-get install python3-opencv python3-picamera2 python3-pip libgpiod2
pip3 install imutils dlib scipy pyserial smbus2 Adafruit_DHT picamera2
```

If using a Raspberry Pi camera, ensure the camera interface is enabled in `raspi-config`.


### 4. Running the Code


```sh
python3 Drowsiness_Detection.py
```

The script will automatically use a USB webcam if available, or fall back to Pi Camera (Picamera2) if running on a Raspberry Pi and no webcam is detected.

The script will start the webcam, perform drowsiness detection, and attempt to read all sensors. If a sensor is not aconnected, its value will be reported as -1. All data is sent via UART (HC-12) in a single line per frame.

---


## System Details & Significance

### Why is this needed?

Mining and exploration are high-risk activities. Workers may collapse due to exhaustion, injury, or hazardous conditions. Traditional helmets do not provide real-time monitoring or emergency alerts. This system:

- Detects drowsiness and collapse using computer vision and motion sensors
- Sends immediate alerts and GPS coordinates to rescue teams
- Monitors environmental conditions (temperature, humidity)
- Reduces false alarms by only activating camera when needed
- Saves battery and processing by running camera and LED only when helmet is tilted


### How does it work?

- The helmet constantly monitors orientation using the MPU6050 gyro/accelerometer.
- If the helmet is upright, only sensor data is sent via HC-12 UART.
- If the helmet is tilted >90° for >5 seconds, the camera and LED activate, and face/drowsiness detection starts using OpenCV and dlib facial landmarks (EAR method).
- If no face is detected, a collapse alert and GPS coordinates are sent repeatedly via HC-12.
- If a harmful gas is detected by the MQ sensor (via MCP3008 ADC), GPIO 22 is set high to alert the miner and a message with gas type, percentage, and GPS coordinates is sent via HC-12.
- The worker can recalibrate the helmet orientation at any time using the button (GPIO 17).
- All sensor threads run in the background, and the main loop fuses data and sends it via UART.

### Future Improvements

- Add heart rate and SpO2 sensors for health monitoring
- Integrate LoRa or cellular for long-range communication
- Add vibration motor for haptic alerts
- Use AI for more robust activity/fall detection
- Solar or battery management for longer runtime
- Cloud dashboard for remote monitoring and analytics

### Conceptual Design Note

This project is a conceptual prototype. It demonstrates how sensor fusion, computer vision, and IoT can be combined for real-world safety applications in mining and exploration. The design can be adapted for other hazardous environments (construction, firefighting, etc).

#### 1. Facial Landmark Detection

The system uses dlib's pre-trained 68-point facial landmark detector. For each detected face, the model predicts 68 (x, y) coordinates corresponding to facial features. The left and right eyes are extracted using the following indices:

- Left eye: points 42–47
- Right eye: points 36–41

#### 2. Eye Aspect Ratio (EAR)

The EAR is a single scalar value that reflects the degree of eye openness. It is calculated as:

    EAR = (||p2-p6|| + ||p3-p5||) / (2 * ||p1-p4||)

Where p1–p6 are the eye landmark points (see image below). The numerator sums the distances between the vertical eye landmarks, and the denominator is the distance between the horizontal eye landmarks.

<img src="assets/eye1.jpg">

#### 3. Drowsiness Logic

- For each frame, the EAR is computed for both eyes and averaged.
- If the average EAR drops below 0.25 for 20 consecutive frames, the system triggers a drowsiness alert (visual warning on the video frame).
- The threshold and frame count are tunable for different users and environments.

<img src="assets/eye2.png">

#### 4. Robustness

- The EAR method is robust to normal blinking, as blinks are brief and do not persist for 20 frames.
- The system works in real time on Raspberry Pi 4, processing frames at ~10–15 FPS depending on lighting and camera quality.

<img src="assets/eye3.jpg">

#### 5. Limitations

- Requires a clear, unobstructed view of the eyes.
- Performance may degrade in low light or with glasses/sunglasses.

---


### Sensor Integration: Data Acquisition and Fusion


#### DHT22 (Temperature & Humidity)

- Connected to a GPIO pin (default: GPIO 4).
- The Adafruit_DHT library is used to read temperature (°C) and humidity (%).
- If the sensor or library is missing, the code reports -1 for both values.

#### MPU6050 (Accelerometer & Gyroscope)

- Connected via I2C (address 0x68).
- The smbus2 library is used to read raw accelerometer (x, y, z in g) and gyroscope (x, y, z in °/s) data.
- The code initializes the sensor and reads 6 values per frame.
- If the sensor or library is missing, all values are reported as -1.

#### NEO-6M GPS

- Connected via UART (default: /dev/ttyS0).
- The pyserial library reads NMEA sentences from the GPS module.
- The code parses $GPGGA sentences to extract latitude and longitude.
- If the sensor or library is missing, both values are reported as -1.



#### MQ Gas Sensor (e.g., MQ-2, MQ-7, MQ-135)

- Connected via analog input (using an ADC like MCP3008) to the Raspberry Pi SPI interface. GPIO 22 is used for alert output (buzzer/LED).
- The system continuously monitors gas concentration. If a harmful gas (e.g., CO, CH4, LPG, smoke) is detected above a threshold, GPIO 22 is set high to activate a buzzer/LED.
- The helmet sends a message via HC-12 with the gas type, percentage concentration, and GPS coordinates for immediate response.
- Example message: `GAS:CO,PERCENT:35,RAW:350,GPS:(19.123456,72.123456)`
- The code supports multiple gas types and thresholds, and can be extended for other MQ sensors.



#### Sensor Fusion Logic

- All sensor readings, drowsiness status, helmet tilt, and gas alerts are combined into a single string per frame.
- Example: `DROWSY:1,TEMP:28.0,HUM:60.0,GAS:CO,PERCENT:35,RAW:350,ACC:(0.01,0.02,0.98),GYRO:(0.00,0.01,0.00),GPS:(19.123456,72.123456)`
- This string can be parsed by a remote device for real-time monitoring or logging.

---



## Data Format: Protocol Specification

Each frame, the following data is sent via UART:

```
DROWSY:<0|1>,TEMP:<float>,HUM:<float>,GAS:<type>,PERCENT:<percent>,RAW:<adc_value>,ACC:(x,y,z),GYRO:(x,y,z),GPS:(lat,lon)
```

- `DROWSY`: 1 if drowsiness detected, 0 otherwise
- `TEMP`: Temperature in Celsius
- `HUM`: Relative humidity in percent
- `GAS`: Detected gas type (e.g., CO, CH4, LPG, SMOKE)
- `PERCENT`: Percentage concentration of detected gas
- `RAW`: Raw ADC value from MQ sensor
- `ACC`: Accelerometer readings (g)
- `GYRO`: Gyroscope readings (°/s)
- `GPS`: Latitude and longitude

Example:

```
DROWSY:1,TEMP:28.0,HUM:60.0,GAS:CO,PERCENT:35,RAW:350,ACC:(0.01,0.02,0.98),GYRO:(0.00,0.01,0.00),GPS:(19.123456,72.123456)
```

---


## Credits & Attribution

- Algorithm originally by [Akshay Bahadur](https://github.com/akshaybahadur21/Drowsiness_Detection) (see original repo for academic citation)
- This conceptual design, integration, and mining safety adaptation by **sudo atharva**

---


## References

- Adrian Rosebrock, [PyImageSearch Blog](https://www.pyimagesearch.com/2017/05/08/drowsiness-detection-opencv/)
