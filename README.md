

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

# Mining Helmet Safety System

## Overview

This project implements a robust, modular safety system for mining helmets using a Raspberry Pi 4. The system integrates drowsiness detection, head movement (wiggle) detection, environmental sensing, GPS location tracking, and wireless alerting. It is designed for reliability, real-time responsiveness, and professional deployment in safety-critical environments.

## Features

- **Drowsiness Detection:** Uses OpenCV, dlib, and facial landmarks to monitor eye aspect ratio (EAR) and detect drowsiness. If drowsiness is detected, a buzzer is activated and alerts are sent.
- **Head Wiggle Detection:** Monitors MPU6050 gyroscope for rapid head movements. Camera and LED are activated upon sufficient wiggling.
- **Environmental Sensing:** Reads temperature and humidity from DHT22, and detects harmful gases (LPG, CO, CH4) using MQ sensor via MCP3008 ADC. Gas alerts are sent wirelessly.
- **GPS Location Tracking:** Acquires real-time location using NEO-6M GPS or compatible modules.
- **Wireless Alerting:** Sends alerts and sensor data via HC-12 UART wireless module.
- **Buzzer Alert:** Activates buzzer (GPIO 23) when drowsiness is detected for immediate physical feedback.
- **Modular Sensor Test Scripts:** Individual scripts for hardware validation and troubleshooting.
- **Automatic Camera Selection:** Supports both USB webcams and PiCamera2, with optimized settings for performance.
- **Robust Error Handling:** All hardware interfaces and threads are protected against runtime errors.
- **Resource Cleanup:** Ensures all GPIO and hardware resources are safely released on exit or error.

## Hardware Requirements

- Raspberry Pi 4 (recommended)
- DHT22 sensor (temperature/humidity)
- MPU6050 sensor (accelerometer/gyroscope)
- MQ gas sensor (LPG/CO/CH4) + MCP3008 ADC
- NEO-6M GPS module (or compatible)
- HC-12 UART wireless module
- USB webcam or PiCamera2
- LED indicator
- Physical button (for gyro calibration)
- Buzzer (for drowsiness alert)

## Wiring and GPIO Pinout

| Component      | GPIO Pin | Notes                       |
|---------------|----------|-----------------------------|
| DHT22         | 4        | Data pin                    |
| MPU6050       | I2C      | SDA/SCL (hardware I2C)      |
| MQ Gas Alert  | 22       | Output pin for gas alert    |
| LED           | 27       | Output pin for status       |
| Button        | 17       | Input pin for calibration   |
| Buzzer        | 23       | Output pin for drowsiness   |

## Software Architecture

- **Language & Libraries:** Python 3, OpenCV, dlib, imutils, RPi.GPIO, smbus2, serial, psutil, gc, Adafruit_DHT, spidev
- **Modular Classes:** Each sensor and subsystem is encapsulated in a dedicated class for maintainability and extensibility.
- **Multi-threaded Sensor Polling:** Ensures real-time responsiveness and non-blocking operation.
- **Main Loop:** Coordinates sensor fusion, camera activation, drowsiness detection, and alerting.
- **Performance Monitoring:** Tracks FPS, processing time, and memory usage for long-term stability.
- **Error Handling:** All hardware and threads are protected against runtime errors, with safe resource cleanup.

## System Operation

1. **Startup:** Initializes all sensors, camera, and communication modules. Gyroscope is automatically calibrated.
2. **Wiggling Detection:** Rapid head movements detected by MPU6050 activate the camera and LED.
3. **Drowsiness Detection:** Camera processes frames for facial landmarks. If eyes are closed (EAR below threshold for several frames), drowsiness is detected and buzzer is activated.
4. **Alerting:**
    - **Drowsiness Detected:** Buzzer activates, GPS coordinates and sensor data are sent via HC-12.
    - **No Face Detected:** Buzzer deactivates, alert sent via HC-12.
    - **Eyes Detected & Not Drowsy:** Camera and buzzer deactivate until next wiggle.
5. **Environmental Sensing:** MQ gas sensor triggers alert and sends data if harmful gas detected.
6. **User Interaction:** Physical button or terminal command recalibrates gyro. 'q' or 'quit' exits system.
7. **Cleanup:** All hardware and resources are safely released on exit or error.

## Example Usage

```bash
python3 Drowsiness_Detection.py
```

## Code Structure

- `Drowsiness_Detection.py`: Main system logic and modular classes for sensors, camera, drowsiness detection, and communication.
- `gps_check.py`, `dht11_check.py`, `mpu6050_check.py`, `mq_check.py`: Individual sensor test scripts for hardware validation.
- `models/shape_predictor_68_face_landmarks.dat`: Required for facial landmark detection (download from dlib.net).
- `assets/`: Example images and resources.

## Professional Implementation Notes

- All hardware interfaces are robustly error-handled for field reliability and safety.
- GPIO pins are initialized and cleaned up to prevent hardware lockups and ensure safe operation.
- Modular class design enables easy extension, maintenance, and future upgrades.
- Performance monitoring and memory management are included for long-term stability in harsh environments.
- Buzzer integration provides immediate physical feedback for drowsiness, enhancing safety.
- System is suitable for real-world mining helmet deployment and can be adapted for other safety-critical applications.

## Extending the System

- Add support for additional sensors (e.g., air quality, vibration, light).
- Integrate cloud-based alerting, remote monitoring, and data logging.
- Expand user interface for configuration, diagnostics, and reporting.
- Add wiring diagrams, PCB layouts, and enclosure designs for manufacturing.

## License

See LICENSE.txt for details.
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
