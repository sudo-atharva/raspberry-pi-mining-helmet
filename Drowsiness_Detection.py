from scipy.spatial import distance
from imutils import face_utils
import imutils
import dlib
import cv2
import platform
import importlib
import serial
import time
import threading
from collections import deque
from datetime import datetime
import math
import os
import sys
import RPi.GPIO as GPIO
# Optional imports for MQ gas sensor and MCP3008
try:
    import spidev
except ImportError:
    spidev = None

# MQ gas sensor configuration
MQ_CHANNEL = 0  # MCP3008 channel for MQ sensor
MQ_ALERT_PIN = 22  # GPIO pin for miner alert
GPIO.setmode(GPIO.BCM)
GPIO.setup(MQ_ALERT_PIN, GPIO.OUT)
GPIO.output(MQ_ALERT_PIN, GPIO.LOW)

# Define supported gases and thresholds (example values)
MQ_GASES = {
    'LPG': {'threshold': 300, 'type': 'LPG'},
    'CO': {'threshold': 250, 'type': 'CO'},
    'CH4': {'threshold': 350, 'type': 'Methane'},
}

# Helper to read MCP3008 channel
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

# Detect harmful gas
def detect_gas():
    value = read_mcp3008(MQ_CHANNEL)
    detected = None
    percent = 0
    for gas, info in MQ_GASES.items():
        if value >= info['threshold']:
            detected = info['type']
            percent = min(100, int((value / 1023) * 100))
            break
    return detected, percent, value

# Optional imports with error handling
try:
    import Adafruit_DHT
except ImportError:
    Adafruit_DHT = None

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

# Global variables for sensor data
sensor_data = {
    'temperature': -1,
    'humidity': -1,
    'accel_x': -1, 'accel_y': -1, 'accel_z': -1,
    'gyro_x': -1, 'gyro_y': -1, 'gyro_z': -1,
    'lat': -1, 'lon': -1
}
sensor_lock = threading.Lock()

# --- Drowsiness Detection Setup ---
def eye_aspect_ratio(eye):
    """Calculate the eye aspect ratio (EAR) for drowsiness detection"""
    A = distance.euclidean(eye[1], eye[5])
    B = distance.euclidean(eye[2], eye[4])
    C = distance.euclidean(eye[0], eye[3])
    ear = (A + B) / (2.0 * C)
    return ear

def check_face_landmarks_file():
    """Check if face landmarks file exists"""
    landmarks_file = "models/shape_predictor_68_face_landmarks.dat"
    if not os.path.exists(landmarks_file):
        print(f"Error: Face landmarks file not found at {landmarks_file}")
        print("Please download it from: http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2")
        print("Extract it and place in the 'models' directory")
        return False
    return True

# Check for required files
if not check_face_landmarks_file():
    sys.exit(1)

# Initialize drowsiness detection variables
EAR_THRESHOLD = 0.25
FRAME_CHECK = 10  # Reduced from 20 for faster response
FACE_DETECT_INTERVAL = 3  # Detect faces every 3 frames instead of every frame

# Initialize face detection
detect = dlib.get_frontal_face_detector()
predict = dlib.shape_predictor("models/shape_predictor_68_face_landmarks.dat")
(lStart, lEnd) = face_utils.FACIAL_LANDMARKS_68_IDXS["left_eye"]
(rStart, rEnd) = face_utils.FACIAL_LANDMARKS_68_IDXS["right_eye"]

# Initialize camera variables
picam2 = None
cap = None

# --- Camera Setup ---
def initialize_camera():
    """Initialize camera with performance optimizations"""
    global cap, picam2
    
    # Try USB webcam first
    try:
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            # Aggressive optimization for performance
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Lower resolution
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer lag
            
            # Test camera
            ret, frame = cap.read()
            if ret and frame is not None:
                print("Using USB webcam")
                return True
            else:
                cap.release()
                cap = None
    except Exception as e:
        print(f"USB webcam failed: {e}")
        cap = None
    
    # Try PiCamera2 on Raspberry Pi
    if platform.system() == 'Linux':
        try:
            picamera2_spec = importlib.util.find_spec("picamera2")
            if picamera2_spec is not None:
                from picamera2 import Picamera2
                picam2 = Picamera2()
                # Optimized for speed
                picam2.configure(picam2.create_preview_configuration(
                    main={"format": 'RGB888', "size": (320, 240)},
                    controls={"FrameRate": 30, "AwbEnable": 0, "AeEnable": 1, "NoiseReductionMode": 0}
                ))
                picam2.start()
                print("Using PiCamera2")
                return True
        except Exception as e:
            print(f"PiCamera2 error: {e}")
    
    return False

# --- GPIO Setup ---
BUTTON_PIN = 17  # Physical button for gyro calibration
LED_PIN = 27     # LED indicator
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.output(LED_PIN, GPIO.LOW)

# --- Gyro Calibration ---
gyro_zero = {'x': 0, 'y': 0, 'z': 0}
def calibrate_gyro():
    with sensor_lock:
        gyro_zero['x'] = sensor_data['gyro_x']
        gyro_zero['y'] = sensor_data['gyro_y']
        gyro_zero['z'] = sensor_data['gyro_z']
    print("Gyro calibrated!")

# --- Helmet Tilt Detection ---
tilt_start_time = None
CAMERA_ACTIVE = False
COLLAPSE_ALERT = False

# --- Sensor Thread Functions ---
def dht_thread():
    """Background thread for DHT11 sensor"""
    if not Adafruit_DHT:
        return
    
    DHT_SENSOR = Adafruit_DHT.DHT11
    DHT_PIN = 4
    
    while True:
        try:
            humidity, temperature = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)
            with sensor_lock:
                sensor_data['temperature'] = temperature if temperature else -1
                sensor_data['humidity'] = humidity if humidity else -1
        except Exception:
            pass
        time.sleep(2)  # Read every 2 seconds

def mpu6050_thread():
    """Background thread for MPU6050 sensor"""
    if not SMBus:
        return
    
    try:
        bus = SMBus(1)
        bus.write_byte_data(0x68, 0x6B, 0)  # Wake up MPU6050
        
        def read_word(reg):
            h = bus.read_byte_data(0x68, reg)
            l = bus.read_byte_data(0x68, reg+1)
            val = (h << 8) + l
            return val - 65536 if val >= 0x8000 else val
        
        while True:
            try:
                accel_x = read_word(0x3B) / 16384.0
                accel_y = read_word(0x3D) / 16384.0
                accel_z = read_word(0x3F) / 16384.0
                gyro_x = read_word(0x43) / 131.0
                gyro_y = read_word(0x45) / 131.0
                gyro_z = read_word(0x47) / 131.0
                
                with sensor_lock:
                    sensor_data.update({
                        'accel_x': accel_x, 'accel_y': accel_y, 'accel_z': accel_z,
                        'gyro_x': gyro_x, 'gyro_y': gyro_y, 'gyro_z': gyro_z
                    })
            except Exception:
                pass
            time.sleep(0.1)  # Read every 100ms
    except Exception:
        pass

def gps_thread():
    """Background thread for GPS data"""
    if not os.path.exists('/dev/ttyS0'):
        return
    
    try:
        gps_serial = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=1)
        
        while True:
            try:
                line = gps_serial.readline().decode('ascii', errors='replace')
                if line.startswith('$GPGGA'):
                    parts = line.split(',')
                    if len(parts) > 5 and parts[2] and parts[4]:
                        lat_raw = float(parts[2])
                        lat = int(lat_raw/100) + (lat_raw % 100)/60
                        if parts[3] == 'S':
                            lat = -lat
                        
                        lon_raw = float(parts[4])
                        lon = int(lon_raw/100) + (lon_raw % 100)/60
                        if parts[5] == 'W':
                            lon = -lon
                        
                        with sensor_lock:
                            sensor_data['lat'] = lat
                            sensor_data['lon'] = lon
            except Exception:
                pass
            time.sleep(1)  # Read every second
    except Exception:
        pass


# --- Main Application ---
def main():
    if not initialize_camera():
        print("Error: No camera available")
        sys.exit(1)

    # Start sensor threads
    if Adafruit_DHT:
        threading.Thread(target=dht_thread, daemon=True).start()
    if SMBus:
        threading.Thread(target=mpu6050_thread, daemon=True).start()
    threading.Thread(target=gps_thread, daemon=True).start()

    # Initialize UART
    uart = None
    if os.path.exists('/dev/ttyUSB0'):
        try:
            uart = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=0.1)
            print("UART initialized")
        except Exception as e:
            print(f"UART error: {e}")

    # Performance monitoring
    frame_times = deque(maxlen=10)
    last_status_print = time.time()

    # Face detection optimization
    flag = 0
    frame_count = 0
    last_faces = []
    face_regions = []

    print("Starting optimized drowsiness detection...")

    try:
        while True:
            loop_start = time.time()

            # --- Gyro Calibration Button ---
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                calibrate_gyro()
                time.sleep(0.5)  # Debounce

            # --- Tilt Detection ---
            with sensor_lock:
                gx = sensor_data['gyro_x'] - gyro_zero['x']
                gy = sensor_data['gyro_y'] - gyro_zero['y']
                gz = sensor_data['gyro_z'] - gyro_zero['z']
            tilt = max(abs(gx), abs(gy), abs(gz))
            if tilt > 90:
                if tilt_start_time is None:
                    tilt_start_time = time.time()
                elif time.time() - tilt_start_time > 5:
                    CAMERA_ACTIVE = True
                    GPIO.output(LED_PIN, GPIO.HIGH)
            else:
                tilt_start_time = None
                CAMERA_ACTIVE = False
                GPIO.output(LED_PIN, GPIO.LOW)

            # --- MQ Gas Sensor Detection ---
            harmful_gas, gas_percent, raw_value = detect_gas()
            if harmful_gas:
                GPIO.output(MQ_ALERT_PIN, GPIO.HIGH)
                if uart:
                    with sensor_lock:
                        data = f"GAS:{harmful_gas},PERCENT:{gas_percent},RAW:{raw_value},GPS:({sensor_data['lat']},{sensor_data['lon']})\n"
                    uart.write(data.encode())
            else:
                GPIO.output(MQ_ALERT_PIN, GPIO.LOW)

            # --- Camera and Detection Only When Tilted ---
            if CAMERA_ACTIVE:
                # ...existing code...
                frame = None
                if cap:
                    ret, frame = cap.read()
                    if not ret:
                        continue
                elif picam2:
                    try:
                        frame = picam2.capture_array()
                        if frame.shape[-1] == 4:
                            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                    except Exception:
                        continue

                if frame is None:
                    continue

                # ...existing code...
                small_frame = imutils.resize(frame, width=160)
                gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)

                # ...existing code...
                frame_times.append(loop_start)
                current_fps = len(frame_times) / (frame_times[-1] - frame_times[0]) if len(frame_times) > 1 else 0

                drowsy = 0

                # ...existing code...
                if frame_count % FACE_DETECT_INTERVAL == 0:
                    faces = detect(gray, 0)
                    if faces:
                        last_faces = faces
                        scale_factor = frame.shape[1] / small_frame.shape[1]
                        face_regions = [(int(face.left() * scale_factor), 
                                       int(face.top() * scale_factor),
                                       int(face.right() * scale_factor), 
                                       int(face.bottom() * scale_factor)) for face in faces]
                    elif not last_faces:
                        face_regions = []

                # ...existing code...
                if last_faces and face_regions:
                    gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                    for i, face in enumerate(last_faces):
                        scale_factor = frame.shape[1] / small_frame.shape[1]
                        scaled_face = dlib.rectangle(
                            int(face.left() * scale_factor),
                            int(face.top() * scale_factor),
                            int(face.right() * scale_factor),
                            int(face.bottom() * scale_factor)
                        )

                        shape = predict(gray_full, scaled_face)
                        shape = face_utils.shape_to_np(shape)

                        leftEye = shape[lStart:lEnd]
                        rightEye = shape[rStart:rEnd]

                        leftEAR = eye_aspect_ratio(leftEye)
                        rightEAR = eye_aspect_ratio(rightEye)
                        ear = (leftEAR + rightEAR) / 2.0

                        leftEyeHull = cv2.convexHull(leftEye)
                        rightEyeHull = cv2.convexHull(rightEye)
                        cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
                        cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)

                        if ear < EAR_THRESHOLD:
                            flag += 1
                            if flag >= FRAME_CHECK:
                                cv2.putText(frame, "DROWSY ALERT!", (10, 30),
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                                drowsy = 1
                        else:
                            flag = 0

                        cv2.putText(frame, f"EAR: {ear:.2f}", (10, 60),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        break

                # ...existing code...
                if not last_faces or not face_regions:
                    COLLAPSE_ALERT = True
                    if uart:
                        with sensor_lock:
                            data = f"COLLAPSED,GPS:({sensor_data['lat']},{sensor_data['lon']})\n"
                        uart.write(data.encode())
                else:
                    COLLAPSE_ALERT = False

                cv2.putText(frame, f"FPS: {current_fps:.1f}", (10, 90),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                cv2.imshow("Drowsiness Detection", frame)

                if uart and frame_count % 10 == 0:
                    try:
                        with sensor_lock:
                            if COLLAPSE_ALERT:
                                data = f"COLLAPSED,GPS:({sensor_data['lat']},{sensor_data['lon']})\n"
                            else:
                                data = f"DROWSY:{drowsy},TEMP:{sensor_data['temperature']:.1f},HUM:{sensor_data['humidity']:.1f},ACC:({sensor_data['accel_x']:.2f},{sensor_data['accel_y']:.2f},{sensor_data['accel_z']:.2f}),GYRO:({sensor_data['gyro_x']:.2f},{sensor_data['gyro_y']:.2f},{sensor_data['gyro_z']:.2f}),GPS:({sensor_data['lat']},{sensor_data['lon']})\n"
                        uart.write(data.encode())
                    except Exception:
                        pass

                if time.time() - last_status_print > 5:
                    print(f"FPS: {current_fps:.1f}, Drowsy: {drowsy}")
                    last_status_print = time.time()

            # ...existing code...
            else:
                if uart and frame_count % 10 == 0:
                    with sensor_lock:
                        data = f"TEMP:{sensor_data['temperature']:.1f},HUM:{sensor_data['humidity']:.1f},ACC:({sensor_data['accel_x']:.2f},{sensor_data['accel_y']:.2f},{sensor_data['accel_z']:.2f}),GYRO:({sensor_data['gyro_x']:.2f},{sensor_data['gyro_y']:.2f},{sensor_data['gyro_z']:.2f}),GPS:({sensor_data['lat']},{sensor_data['lon']})\n"
                    uart.write(data.encode())

            frame_count += 1

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        GPIO.cleanup()
        if cap:
            cap.release()
        if picam2:
            picam2.close()
        if uart:
            uart.close()
        cv2.destroyAllWindows()
        print("Cleanup complete")

if __name__ == "__main__":
    main()