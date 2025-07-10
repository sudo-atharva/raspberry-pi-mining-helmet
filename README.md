
# Drowsiness Detection & Sensor Fusion for Raspberry Pi 4

This project detects driver drowsiness using computer vision and integrates real-time sensor data (GPS NEO-6M, MPU6050, DHT11) on a Raspberry Pi 4. All data is sent wirelessly via UART (HC-12) to a remote device.

---

## Features

- Real-time drowsiness detection using Eye Aspect Ratio (EAR) and OpenCV
- Sensor fusion: temperature/humidity (DHT11), acceleration/gyroscope (MPU6050), GPS (NEO-6M)
- Robust: Eye detection works even if sensors are not connected
- Data transmission via UART (HC-12)

---

## Quick Start

### 1. Hardware Required

- Raspberry Pi 4 (4GB RAM recommended)
- USB webcam
- DHT11 sensor (GPIO 4 by default)
- MPU6050 (I2C)
- NEO-6M GPS (UART, /dev/ttyS0)
- HC-12 UART module (USB, /dev/ttyUSB0)

### 2. Software Setup

Install dependencies:

```sh
sudo apt-get update
sudo apt-get install python3-opencv python3-pip libgpiod2
pip3 install imutils dlib scipy pyserial smbus2 Adafruit_DHT
```

### 3. Running the Code

```sh
python3 Drowsiness_Detection.py
```

The script will start the webcam, perform drowsiness detection, and attempt to read all sensors. If a sensor is not connected, its value will be reported as -1. All data is sent via UART (HC-12) in a single line per frame.

---

## Deep Analysis

### Drowsiness Detection Algorithm: Technical Details

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

#### DHT11 (Temperature & Humidity)

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

#### HC-12 UART Wireless Module

- Connected via USB-to-serial (default: /dev/ttyUSB0).
- The pyserial library is used to send a formatted string containing all sensor and drowsiness data to a remote receiver.
- If the module or library is missing, data is not sent but the program continues.

#### Sensor Fusion Logic

- All sensor readings and drowsiness status are combined into a single string per frame.
- Example: `DROWSY:1,TEMP:28.0,HUM:60.0,ACC:(0.01,0.02,0.98),GYRO:(0.00,0.01,0.00),GPS:(19.123456,72.123456)`
- This string can be parsed by a remote device for real-time monitoring or logging.

---

## Data Format: Protocol Specification

Each frame, the following data is sent via UART:

```
DROWSY:<0|1>,TEMP:<float>,HUM:<float>,ACC:(x,y,z),GYRO:(x,y,z),GPS:(lat,lon)
```

- `DROWSY`: 1 if drowsiness detected, 0 otherwise
- `TEMP`: Temperature in Celsius
- `HUM`: Relative humidity in percent
- `ACC`: Accelerometer readings (g)
- `GYRO`: Gyroscope readings (°/s)
- `GPS`: Latitude and longitude

Example:

```
DROWSY:1,TEMP:28.0,HUM:60.0,ACC:(0.01,0.02,0.98),GYRO:(0.00,0.01,0.00),GPS:(19.123456,72.123456)
```

---

## Credits & Attribution

- Algorithm originally by [Akshay Bahadur](https://github.com/akshaybahadur21/Drowsiness_Detection) (see original repo for academic citation)
- This implementation and sensor integration by **sudo atharva**

---

## References

- Adrian Rosebrock, [PyImageSearch Blog](https://www.pyimagesearch.com/2017/05/08/drowsiness-detection-opencv/)
