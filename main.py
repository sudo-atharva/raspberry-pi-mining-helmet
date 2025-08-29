#!/usr/bin/env python3
"""
Robust Mining Helmet Safety System
- Optimized for performance and reliability
- Modular design with proper error handling
- Configurable sensor enables/disables
"""

import cv2
import dlib
import time
import threading
import platform
import importlib
import serial
import serial.tools.list_ports
import math
import os
import sys
import select
import psutil
import gc
from collections import deque
from datetime import datetime
from scipy.spatial import distance
from imutils import face_utils
import imutils
import numpy as np

# For logging
import csv
import os

# GPIO and sensor imports with error handling
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("Warning: RPi.GPIO not available. Running in simulation mode.")

try:
    import spidev
    SPI_AVAILABLE = True
except ImportError:
    SPI_AVAILABLE = False


# DHT sensor support (DHT11/DHT22 via adafruit_dht)
try:
    import board
    import adafruit_dht
    DHT_LIB_AVAILABLE = True
except ImportError:
    board = None
    adafruit_dht = None
    DHT_LIB_AVAILABLE = False

try:
    from smbus2 import SMBus
    I2C_AVAILABLE = True
except ImportError:
    I2C_AVAILABLE = False

# =============================================================================
# CONFIGURATION
# =============================================================================


class Config:
    """Configuration class for all system settings"""
    # Hardware enable flags
    ENABLE_HC12 = True
    ENABLE_MQ_SENSOR = True
    ENABLE_DHT22 = True  # Set True for DHT22
    ENABLE_DHT11 = False   # Set True for DHT11
    ENABLE_GPS = True
    ENABLE_MPU6050 = True

    # GPIO Pins
    BUZZER_PIN = 23
    BUTTON_PIN = 17
    LED_PIN = 27
    MQ_ALERT_PIN = 22
    GPS_LED_PIN = 25  # LED to indicate GPS fix status

    # Sensor settings
    DHT_PIN = 4
    MQ_CHANNEL = 0

    # Camera settings
    CAMERA_WIDTH = 320
    CAMERA_HEIGHT = 240
    CAMERA_FPS = 20
    # Camera orientation: 'normal', 'flip', 'rotate_90', 'rotate_180', 'rotate_270'
    CAMERA_ORIENTATION = 'rotate_180'  # Rotate 180 degrees for better view

    # Drowsiness detection
    EAR_THRESHOLD = 0.25
    FRAME_CHECK = 6
    FACE_DETECT_INTERVAL = 3

    # Wiggle detection
    WIGGLE_THRESHOLD = 20
    WIGGLE_WINDOW = 2.0
    WIGGLE_COUNT = 3

    # Camera active duration (seconds)
    CAMERA_ACTIVE_DURATION = 30

    # Performance
    TARGET_FPS = 15
    SENSOR_UPDATE_INTERVAL = 0.05
    DHT_UPDATE_INTERVAL = 3.0
    GPS_UPDATE_INTERVAL = 1.0

    # Communication
    HC12_BAUDRATE = 9600
    GPS_BAUDRATE = 9600

    # Paths
    FACE_LANDMARKS_PATH = "models/shape_predictor_68_face_landmarks.dat"

# =============================================================================
# UTILITY CLASSES
# =============================================================================

class PerformanceMonitor:
    """Monitor system performance and FPS"""
    
    def __init__(self):
        self.reset()
    
    def reset(self):
        self.frame_count = 0
        self.start_time = time.time()
        self.last_fps_check = time.time()
        self.fps = 0
        self.processing_times = deque(maxlen=30)
    
    def update(self, processing_time=None):
        self.frame_count += 1
        if processing_time:
            self.processing_times.append(processing_time)
        
        current_time = time.time()
        if current_time - self.last_fps_check >= 3.0:  # Check every 3 seconds
            self.fps = self.frame_count / (current_time - self.last_fps_check)
            self.frame_count = 0
            self.last_fps_check = current_time
            
            # Memory cleanup if needed
            memory_percent = psutil.virtual_memory().percent
            if memory_percent > 80:
                gc.collect()
            
            # Log performance
            avg_processing = np.mean(self.processing_times) if self.processing_times else 0
            print(f"Performance: FPS={self.fps:.1f}, AvgProcessing={avg_processing*1000:.1f}ms, "
                  f"Memory={memory_percent:.1f}%")
    
    def get_fps(self):
        return self.fps

class SensorData:
    """Thread-safe sensor data container"""
    
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {
            'temperature': -1,
            'humidity': -1,
            'accel_x': 0, 'accel_y': 0, 'accel_z': 0,
            'gyro_x': 0, 'gyro_y': 0, 'gyro_z': 0,
            'lat': 0, 'lon': 0,
            'gas_detected': None,
            'gas_percent': 0
        }
        self.gyro_zero = {'x': 0, 'y': 0, 'z': 0}
    
    def update(self, **kwargs):
        with self.lock:
            self.data.update(kwargs)
    
    def get_data(self):
        with self.lock:
            return self.data.copy()
    
    def calibrate_gyro(self):
        with self.lock:
            self.gyro_zero['x'] = self.data['gyro_x']
            self.gyro_zero['y'] = self.data['gyro_y']
            self.gyro_zero['z'] = self.data['gyro_z']
    
    def get_calibrated_gyro(self):
        with self.lock:
            return {
                'x': self.data['gyro_x'] - self.gyro_zero['x'],
                'y': self.data['gyro_y'] - self.gyro_zero['y'],
                'z': self.data['gyro_z'] - self.gyro_zero['z']
            }

# =============================================================================
# SENSOR CLASSES
# =============================================================================

class MPU6050Sensor:
    """MPU6050 accelerometer and gyroscope sensor"""
    
    def __init__(self, sensor_data):
        self.sensor_data = sensor_data
        self.bus = None
        self.running = False
        self.thread = None
        
        if I2C_AVAILABLE:
            try:
                self.bus = SMBus(1)
                self.bus.write_byte_data(0x68, 0x6B, 0)  # Wake up MPU6050
                print("MPU6050 initialized successfully")
            except Exception as e:
                print(f"MPU6050 initialization failed: {e}")
                self.bus = None
    
    def read_word(self, reg):
        """Read 16-bit word from MPU6050"""
        try:
            h = self.bus.read_byte_data(0x68, reg)
            l = self.bus.read_byte_data(0x68, reg + 1)
            val = (h << 8) + l
            return val - 65536 if val >= 0x8000 else val
        except:
            return 0
    
    def read_sensors(self):
        """Read all sensor values"""
        if not self.bus:
            return
        
        try:
            accel_x = self.read_word(0x3B) / 16384.0
            accel_y = self.read_word(0x3D) / 16384.0
            accel_z = self.read_word(0x3F) / 16384.0
            gyro_x = self.read_word(0x43) / 131.0
            gyro_y = self.read_word(0x45) / 131.0
            gyro_z = self.read_word(0x47) / 131.0
            
            self.sensor_data.update(
                accel_x=accel_x, accel_y=accel_y, accel_z=accel_z,
                gyro_x=gyro_x, gyro_y=gyro_y, gyro_z=gyro_z
            )
        except Exception as e:
            print(f"MPU6050 read error: {e}")
    
    def start(self):
        """Start sensor reading thread"""
        if not self.bus:
            return
        
        self.running = True
        self.thread = threading.Thread(target=self._sensor_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """Stop sensor reading"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
    
    def _sensor_loop(self):
        """Main sensor reading loop"""
        while self.running:
            self.read_sensors()
            time.sleep(Config.SENSOR_UPDATE_INTERVAL)


# General DHT sensor class supporting DHT11 and DHT22
class DHTSensor:
    """DHT11/DHT22 temperature and humidity sensor"""
    def __init__(self, sensor_data):
        self.sensor_data = sensor_data
        self.running = False
        self.thread = None
        self.dht_device = None
        self.sensor_type = None
        # Determine which sensor to use
        if DHT_LIB_AVAILABLE:
            if Config.ENABLE_DHT22:
                self.sensor_type = 'DHT22'
                self.dht_device = adafruit_dht.DHT22(board.D4)
            elif Config.ENABLE_DHT11:
                self.sensor_type = 'DHT11'
                self.dht_device = adafruit_dht.DHT11(board.D4)

    def start(self):
        if not DHT_LIB_AVAILABLE or not (Config.ENABLE_DHT22 or Config.ENABLE_DHT11):
            print("DHT library not available or DHT sensor not enabled.")
            return
        if not self.dht_device:
            print("DHT device not initialized.")
            return
        self.running = True
        self.thread = threading.Thread(target=self._sensor_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)

    def _sensor_loop(self):
        while self.running:
            try:
                temperature = self.dht_device.temperature
                humidity = self.dht_device.humidity
                print(f"[{self.sensor_type}] Read: Temp={temperature}, Hum={humidity}")
                if humidity is not None and temperature is not None:
                    self.sensor_data.update(temperature=temperature, humidity=humidity)
            except Exception as e:
                print(f"{self.sensor_type} read error: {e}")
            time.sleep(Config.DHT_UPDATE_INTERVAL)

class GPSSensor:
    """GPS sensor for location tracking"""
    
    def __init__(self, sensor_data):
        self.sensor_data = sensor_data
        self.running = False
        self.thread = None
        self.serial_port = None
        self.has_fix = False
        self.led_state = False
        self.last_blink = 0
        
        # Setup GPS status LED if GPIO is available
        if GPIO_AVAILABLE:
            GPIO.setup(Config.GPS_LED_PIN, GPIO.OUT)
            GPIO.output(Config.GPS_LED_PIN, GPIO.LOW)
    
    def start(self):
        """Start GPS reading thread"""
        if not Config.ENABLE_GPS:
            return
        
        # Try to open GPS serial port
        for port in ['/dev/ttyS0', '/dev/ttyAMA0', '/dev/ttyUSB0']:
            try:
                self.serial_port = serial.Serial(port, Config.GPS_BAUDRATE, timeout=1)
                print(f"GPS initialized on {port}")
                break
            except:
                continue
        
        if not self.serial_port:
            print("GPS initialization failed: No available ports")
            return
        
        self.running = True
        self.thread = threading.Thread(target=self._sensor_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """Stop GPS reading"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
        if self.serial_port:
            self.serial_port.close()
        if GPIO_AVAILABLE:
            GPIO.output(Config.GPS_LED_PIN, GPIO.LOW)
    
    def _sensor_loop(self):
        """Main GPS reading loop"""
        while self.running:
            try:
                line = self.serial_port.readline().decode('ascii', errors='replace')
                if line.startswith('$GPGGA'):
                    self._parse_gga(line)
            except Exception as e:
                print(f"GPS read error: {e}")
            
            time.sleep(Config.GPS_UPDATE_INTERVAL)
    
    def _parse_gga(self, line):
        """Parse GPGGA sentence"""
        try:
            parts = line.split(',')
            if len(parts) > 6:  # Check fix quality
                fix_quality = int(parts[6]) if parts[6] else 0
                self.has_fix = fix_quality > 0
                
                # Update GPS LED status
                if GPIO_AVAILABLE:
                    current_time = time.time()
                    if self.has_fix:
                        # Solid LED when we have a fix
                        if not self.led_state:
                            GPIO.output(Config.GPS_LED_PIN, GPIO.HIGH)
                            self.led_state = True
                    else:
                        # Blink LED when searching for fix (1 Hz)
                        if current_time - self.last_blink >= 0.5:
                            self.led_state = not self.led_state
                            GPIO.output(Config.GPS_LED_PIN, GPIO.HIGH if self.led_state else GPIO.LOW)
                            self.last_blink = current_time
                
                # Parse coordinates if we have data
                if len(parts) > 5 and parts[2] and parts[4]:
                    # Parse latitude
                    lat_raw = float(parts[2])
                    lat = int(lat_raw/100) + (lat_raw % 100)/60
                    if parts[3] == 'S':
                        lat = -lat
                    
                    # Parse longitude
                    lon_raw = float(parts[4])
                    lon = int(lon_raw/100) + (lon_raw % 100)/60
                    if parts[5] == 'W':
                        lon = -lon
                    
                    self.sensor_data.update(lat=lat, lon=lon)
                    
                    if self.has_fix:
                        print(f"GPS Fix: lat={lat:.6f}, lon={lon:.6f}, Quality={fix_quality}")
                if GPIO_AVAILABLE:
                    GPIO.output(Config.GPS_LED_PIN, GPIO.HIGH)  # Solid LED when fix obtained
        except Exception as e:
            print(f"GPS parse error: {e}")

# =============================================================================
# CAMERA AND VISION CLASSES
# =============================================================================

class CameraManager:
    """Optimized camera management"""
    
    def __init__(self):
        self.cap = None
        self.picam2 = None
        self.is_pi_camera = False
        self.frame_buffer = deque(maxlen=2)
        self.lock = threading.Lock()
        self.orientation = Config.CAMERA_ORIENTATION
    
    def initialize(self):
        """Initialize camera with optimization"""
        # Try USB webcam first
        try:
            self.cap = cv2.VideoCapture(0)
            if self.cap.isOpened():
                # Optimize settings
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, Config.CAMERA_WIDTH)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Config.CAMERA_HEIGHT)
                self.cap.set(cv2.CAP_PROP_FPS, Config.CAMERA_FPS)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                # Test read
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    print("USB camera initialized")
                    return True
                else:
                    self.cap.release()
                    self.cap = None
        except Exception as e:
            print(f"USB camera failed: {e}")
            self.cap = None
        
        # Try Pi Camera
        if platform.system() == 'Linux':
            try:
                picamera2_spec = importlib.util.find_spec("picamera2")
                if picamera2_spec:
                    from picamera2 import Picamera2
                    self.picam2 = Picamera2()
                    config = self.picam2.create_preview_configuration(
                        main={"format": 'RGB888', "size": (Config.CAMERA_WIDTH, Config.CAMERA_HEIGHT)},
                        controls={"FrameRate": Config.CAMERA_FPS}
                    )
                    self.picam2.configure(config)
                    self.picam2.start()
                    self.is_pi_camera = True
                    time.sleep(1)  # Camera warmup
                    print("Pi Camera initialized")
                    return True
            except Exception as e:
                print(f"Pi Camera failed: {e}")
        
        return False
    
    def read_frame(self):
        """Read frame from camera"""
        with self.lock:
            try:
                if self.cap:
                    ret, frame = self.cap.read()
                    if not ret or frame is None:
                        return None
                    frame = self._apply_orientation(frame)
                    return frame
                elif self.picam2:
                    frame = self.picam2.capture_array()
                    if frame.shape[-1] == 4:
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                        frame = self._apply_orientation(frame)
                    return frame
            except Exception as e:
                print(f"Camera read error: {e}")
                return None
    def _apply_orientation(self, frame):
        """Apply orientation to frame based on config"""
        # Debug print to confirm orientation is being used
        if self.orientation != 'normal':
            print(f"Applying camera orientation: {self.orientation}")
        if self.orientation == 'flip':
            return cv2.flip(frame, -1)  # Flip both axes
        elif self.orientation == 'rotate_90':
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.orientation == 'rotate_180':
            return cv2.rotate(frame, cv2.ROTATE_180)
        elif self.orientation == 'rotate_270':
            return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return frame
    
    def release(self):
        """Release camera resources"""
        if self.cap:
            self.cap.release()
        if self.picam2:
            self.picam2.close()

class DrowsinessDetector:
    """Optimized drowsiness detection"""
    
    def __init__(self):
        self.detector = None
        self.predictor = None
        self.initialize()
        
        self.flag = 0
        self.face_detection_skip = 0
        self.cached_faces = []
        
        # Eye landmarks indices
        self.left_eye_start, self.left_eye_end = face_utils.FACIAL_LANDMARKS_68_IDXS["left_eye"]
        self.right_eye_start, self.right_eye_end = face_utils.FACIAL_LANDMARKS_68_IDXS["right_eye"]
    
    def initialize(self):
        """Initialize face detection models"""
        if not os.path.exists(Config.FACE_LANDMARKS_PATH):
            print(f"Error: Face landmarks file not found at {Config.FACE_LANDMARKS_PATH}")
            print("Please download from: http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2")
            return False
        
        try:
            self.detector = dlib.get_frontal_face_detector()
            self.predictor = dlib.shape_predictor(Config.FACE_LANDMARKS_PATH)
            print("Face detection models loaded successfully")
            return True
        except Exception as e:
            print(f"Face detection initialization failed: {e}")
            return False
    
    def eye_aspect_ratio(self, eye):
        """Calculate eye aspect ratio"""
        A = distance.euclidean(eye[1], eye[5])
        B = distance.euclidean(eye[2], eye[4])
        C = distance.euclidean(eye[0], eye[3])
        return (A + B) / (2.0 * C)
    
    def detect_drowsiness(self, frame):
        """Detect drowsiness in frame"""
        if not self.detector or not self.predictor:
            return False, False, frame
        
        # Resize for faster processing
        small_frame = imutils.resize(frame, width=160)
        gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
        
        # Face detection with caching
        faces = []
        if self.face_detection_skip <= 0:
            faces = self.detector(gray, 0)
            if faces:
                self.cached_faces = faces
            self.face_detection_skip = Config.FACE_DETECT_INTERVAL
        else:
            faces = self.cached_faces
            self.face_detection_skip -= 1
        
        face_detected = len(faces) > 0
        drowsy = False
        
        if face_detected:
            gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            for face in faces:
                # Scale face coordinates back to full frame
                scale_factor = frame.shape[1] / small_frame.shape[1]
                scaled_face = dlib.rectangle(
                    int(face.left() * scale_factor),
                    int(face.top() * scale_factor),
                    int(face.right() * scale_factor),
                    int(face.bottom() * scale_factor)
                )
                
                # Get facial landmarks
                shape = self.predictor(gray_full, scaled_face)
                shape = face_utils.shape_to_np(shape)
                
                # Extract eye regions
                left_eye = shape[self.left_eye_start:self.left_eye_end]
                right_eye = shape[self.right_eye_start:self.right_eye_end]
                
                # Calculate EAR
                left_ear = self.eye_aspect_ratio(left_eye)
                right_ear = self.eye_aspect_ratio(right_eye)
                ear = (left_ear + right_ear) / 2.0
                
                # Draw eye contours
                left_eye_hull = cv2.convexHull(left_eye)
                right_eye_hull = cv2.convexHull(right_eye)
                cv2.drawContours(frame, [left_eye_hull], -1, (0, 255, 0), 1)
                cv2.drawContours(frame, [right_eye_hull], -1, (0, 255, 0), 1)
                
                # Drowsiness detection
                if ear < Config.EAR_THRESHOLD:
                    self.flag += 1
                    if self.flag >= Config.FRAME_CHECK:
                        drowsy = True
                        cv2.putText(frame, "DROWSY ALERT!", (10, 30),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                else:
                    self.flag = 0
                
                # Display EAR
                cv2.putText(frame, f"EAR: {ear:.2f}", (10, 60),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                break
        
        return face_detected, drowsy, frame

# =============================================================================
# COMMUNICATION CLASS
# =============================================================================

class HC12Communication:
    """HC-12 wireless communication"""
    
    def __init__(self):
        self.serial_port = None
        self.connected = False
    
    def initialize(self):
        """Initialize HC-12 module with auto-detection"""
        if not Config.ENABLE_HC12:
            return False

        # Candidate ports (common defaults)
        candidates = set(['/dev/ttyAMA0', '/dev/ttyS0', '/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1'])
        # Append OS-reported ports
        try:
            for p in serial.tools.list_ports.comports():
                candidates.add(p.device)
        except Exception:
            pass

        for port in candidates:
            try:
                self.serial_port = serial.Serial(port, Config.HC12_BAUDRATE, timeout=1)
                print(f"HC-12 initialized on {port}")
                self.connected = True
                return True
            except Exception:
                continue

        print("HC-12 initialization failed: no available ports")
        self.connected = False
        self.serial_port = None
        return False
    
    def send_data(self, data):
        """Send data via HC-12"""
        if self.connected and self.serial_port:
            try:
                self.serial_port.write(data.encode())
                self.serial_port.flush()
                return True
            except Exception as e:
                print(f"HC-12 send error: {e}")
                # Mark disconnected to trigger reconnect attempts by caller
                self.connected = False
                try:
                    self.serial_port.close()
                except Exception:
                    pass
                self.serial_port = None
                return False
        return False
    
    def close(self):
        """Close HC-12 connection"""
        if self.serial_port:
            self.serial_port.close()
            self.connected = False

# =============================================================================
# MAIN SYSTEM CLASS
# =============================================================================

class HelmetSafetySystem:
    """Main system coordinator"""
    
    def __init__(self):
        self.sensor_data = SensorData()
        self.performance_monitor = PerformanceMonitor()
        self.camera_manager = CameraManager()
        self.drowsiness_detector = DrowsinessDetector()
        self.hc12 = HC12Communication()
        
        # System state
        self.camera_active = False
        self.running = False
        self.last_alert_time = 0

        # Camera activation timer
        self.camera_activated_time = None
        
        # Wiggle detection
        self.wiggle_events = deque(maxlen=50)
        self.last_gyro = {'x': 0, 'y': 0, 'z': 0}
        
        # Sensors
        self.mpu6050 = MPU6050Sensor(self.sensor_data)
        self.dht = DHTSensor(self.sensor_data)
        self.gps = GPSSensor(self.sensor_data)
        
        # Initialize GPIO
        self.setup_gpio()

        # CSV log file
        self.csv_log_path = os.path.abspath("helmet_log.csv")
        self._init_csv_log()

        # Defaults for indicators and MQ sensor-related placeholders
        self.blink_pin = Config.LED_PIN
        self.last_blink_time = 0
        self.mq_baseline = None
        self.mq_threshold = 0.20  # volts (tweak as needed)
        self.last_send_time = 0

    def _init_csv_log(self):
        if not os.path.exists(self.csv_log_path):
            try:
                with open(self.csv_log_path, "w", newline="") as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow([
                        "timestamp", "event", "status", "gps", "temp", "hum",
                        "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"
                    ])
                print(f"CSV log created at {self.csv_log_path}")
            except Exception as e:
                print(f"CSV log creation error: {e}")

    def log_event(self, event, status=None):
        data = self.sensor_data.get_data()
        gps = f"({data['lat']:.6f},{data['lon']:.6f})"
        temp = data.get('temperature', '-')
        hum = data.get('humidity', '-')
        # If temp/hum is -1 or None, log as 'N/A'
        temp = 'N/A' if temp is None or (isinstance(temp, (int, float)) and temp == -1) else temp
        hum = 'N/A' if hum is None or (isinstance(hum, (int, float)) and hum == -1) else hum
        accel_x = data.get('accel_x', '-')
        accel_y = data.get('accel_y', '-')
        accel_z = data.get('accel_z', '-')
        gyro_x = data.get('gyro_x', '-')
        gyro_y = data.get('gyro_y', '-')
        gyro_z = data.get('gyro_z', '-')
        t = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        try:
            with open(self.csv_log_path, "a", newline="") as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    t, event, status if status else "-", gps, temp, hum,
                    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
                ])
            print(f"Logged event: {event}, status: {status}")
        except Exception as e:
            print(f"CSV log error: {e}")
    
    def setup_gpio(self):
        """Setup GPIO pins"""
        if not GPIO_AVAILABLE:
            print("GPIO not available, running in simulation mode")
            return
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup pins
        GPIO.setup(Config.BUZZER_PIN, GPIO.OUT)
        GPIO.setup(Config.LED_PIN, GPIO.OUT)
        GPIO.setup(Config.BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Initialize outputs
        GPIO.output(Config.BUZZER_PIN, GPIO.LOW)
        GPIO.output(Config.LED_PIN, GPIO.LOW)
        
        print("GPIO initialized successfully")
    
    def initialize(self):
        """Initialize all system components"""
        print("Initializing Mining Helmet Safety System...")
        
        # Initialize camera
        if not self.camera_manager.initialize():
            print("Warning: Camera initialization failed. Continuing without camera.")
        
        # Initialize communication
        self.hc12.initialize()
        
        # Start sensors
        if Config.ENABLE_MPU6050:
            self.mpu6050.start()
            time.sleep(1)  # Wait for initial readings
            self.sensor_data.calibrate_gyro()
            print("MPU6050 calibrated")
        
        if Config.ENABLE_DHT22 or Config.ENABLE_DHT11:
            self.dht.start()
        
        if Config.ENABLE_GPS:
            self.gps.start()
        
        print("System initialization complete")
        return True
    
    def detect_wiggle(self):
        """Detect head wiggling motion"""
        gyro = self.sensor_data.get_calibrated_gyro()
        current_time = time.time()
        
        # Calculate gyro changes
        delta_x = abs(gyro['x'] - self.last_gyro['x'])
        delta_y = abs(gyro['y'] - self.last_gyro['y'])
        delta_z = abs(gyro['z'] - self.last_gyro['z'])
        
        self.last_gyro = gyro
        
        # Check for significant movement
        if (delta_x > Config.WIGGLE_THRESHOLD or 
            delta_y > Config.WIGGLE_THRESHOLD or 
            delta_z > Config.WIGGLE_THRESHOLD):
            self.wiggle_events.append(current_time)
        
        # Remove old events
        while self.wiggle_events and current_time - self.wiggle_events[0] > Config.WIGGLE_WINDOW:
            self.wiggle_events.popleft()
        
        # Check if enough wiggles detected
        return len(self.wiggle_events) >= Config.WIGGLE_COUNT
    
    def activate_camera(self):
        """Activate camera and LED, and set timer"""
        if not self.camera_active:
            self.camera_active = True
            self.camera_activated_time = time.time()
            if GPIO_AVAILABLE:
                GPIO.output(Config.LED_PIN, GPIO.HIGH)
            print("Camera activated by wiggling")
            self.log_event("CAMERA_ACTIVATED")
    
    def deactivate_camera(self):
        """Deactivate camera and LED, reset timer, close camera window"""
        if self.camera_active:
            self.camera_active = False
            self.camera_activated_time = None
            if GPIO_AVAILABLE:
                GPIO.output(Config.LED_PIN, GPIO.LOW)
            try:
                cv2.destroyAllWindows()
                print("cv2.destroyAllWindows() called")
            except Exception as e:
                print(f"cv2.destroyAllWindows error: {e}")
            print("Camera deactivated")
            self.log_event("CAMERA_DEACTIVATED")
    
    def activate_buzzer(self, state):
        """Control buzzer with full power using PWM if available, else digital"""
        if GPIO_AVAILABLE:
            try:
                if not hasattr(self, '_buzzer_pwm'):
                    self._buzzer_pwm = GPIO.PWM(Config.BUZZER_PIN, 2000)  # 2kHz
                    self._buzzer_pwm_started = False
                if state:
                    if not self._buzzer_pwm_started:
                        self._buzzer_pwm.start(100)  # 100% duty cycle
                        self._buzzer_pwm_started = True
                    else:
                        self._buzzer_pwm.ChangeDutyCycle(100)
                else:
                    if self._buzzer_pwm_started:
                        self._buzzer_pwm.ChangeDutyCycle(0)
            except Exception:
                # fallback to digital
                GPIO.output(Config.BUZZER_PIN, GPIO.HIGH if state else GPIO.LOW)
    
    def send_alert(self, alert_type, additional_data=None):
        """Send alert via HC-12 and log to CSV"""
        current_time = time.time()
        if current_time - self.last_alert_time < 5:  # Rate limiting
            return
        data = self.sensor_data.get_data()
        message = (f"{alert_type},GPS:({data['lat']:.6f},{data['lon']:.6f}),"
                  f"TEMP:{data['temperature']:.1f},HUM:{data['humidity']:.1f},"
                  f"TIME:{datetime.now().strftime('%H:%M:%S')}")
        if additional_data:
            message += f",{additional_data}"
        message += "\n"
        if self.hc12.send_data(message):
            self.last_alert_time = current_time
            print(f"Alert sent: {alert_type}")
            self.log_event("ALERT_SENT", alert_type)
    
    def handle_user_input(self):
        """Handle user input (non-blocking)"""
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            cmd = sys.stdin.readline().strip().lower()
            if cmd == "calibrate":
                self.sensor_data.calibrate_gyro()
                print("Gyro calibrated via terminal!")
            elif cmd == "q" or cmd == "quit":
                self.running = False
        
        # Check calibration button
        if GPIO_AVAILABLE and GPIO.input(Config.BUTTON_PIN) == GPIO.LOW:
            self.sensor_data.calibrate_gyro()
            print("Gyro calibrated via button!")
            time.sleep(0.5)  # Debounce
    
    def run(self):
        """Main system loop"""
        if not self.initialize():
            return

        self.running = True
        print("System ready. Wiggle head to activate camera.")
        print("Press 'q' to quit, 'calibrate' to recalibrate gyro")
        # MQ sensor calibration at startup (guarded)
        if Config.ENABLE_MQ_SENSOR:
            try:
                print("Calibrating MQ sensor baseline. Please ensure clean air.")
                mq_samples = []
                calibration_start = time.time()
                while time.time() - calibration_start < 5:
                    mq_voltage = self.sensor_data.get_data().get('gas_percent', 0)
                    mq_samples.append(mq_voltage)
                    # Blink LED during calibration
                    if GPIO_AVAILABLE:
                        GPIO.output(self.blink_pin, GPIO.HIGH)
                        time.sleep(0.2)
                        GPIO.output(self.blink_pin, GPIO.LOW)
                        time.sleep(0.2)
                self.mq_baseline = sum(mq_samples) / max(1, len(mq_samples))
                print(f"MQ sensor baseline calibrated: {self.mq_baseline:.2f} V")
            except Exception as e:
                print(f"MQ calibration skipped due to error: {e}")

        try:
            while self.running:
                loop_start = time.time()

                # Handle user input
                self.handle_user_input()

                # Check for wiggle detection
                if self.detect_wiggle() and not self.camera_active:
                    self.activate_camera()

                # If camera is active, check if duration has passed
                if self.camera_active:
                    # If time exceeded, deactivate camera
                    if self.camera_activated_time and (time.time() - self.camera_activated_time > Config.CAMERA_ACTIVE_DURATION):
                        self.deactivate_camera()
                        self.activate_buzzer(False)
                    else:
                        frame = self.camera_manager.read_frame()
                        if frame is not None:
                                # Always run detection on the original upright frame
                                face_detected, drowsy, processed_frame = self.drowsiness_detector.detect_drowsiness(frame)

                                if drowsy:
                                    self.activate_buzzer(True)
                                    self.send_alert("DROWSY")
                                    self.log_event("DROWSINESS_DETECTED", "DROWSY")
                                elif not face_detected:
                                    self.activate_buzzer(False)
                                    self.send_alert("NO_FACE")
                                    self.log_event("NO_FACE_DETECTED", "NO_FACE")
                                else:
                                    # Face detected and not drowsy - keep camera on
                                    self.activate_buzzer(False)

                                # Rotate only for display/output
                                orientation = Config.CAMERA_ORIENTATION
                                display_frame = processed_frame
                                if orientation == 'flip':
                                    display_frame = cv2.flip(processed_frame, -1)
                                elif orientation == 'rotate_90':
                                    display_frame = cv2.rotate(processed_frame, cv2.ROTATE_90_CLOCKWISE)
                                elif orientation == 'rotate_180':
                                    display_frame = cv2.rotate(processed_frame, cv2.ROTATE_180)
                                elif orientation == 'rotate_270':
                                    display_frame = cv2.rotate(processed_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                                cv2.imshow("Mining Helmet Safety System", display_frame)

                                if cv2.waitKey(1) & 0xFF == ord('q'):
                                    self.running = False

                # Update performance monitoring
                processing_time = time.time() - loop_start
                self.performance_monitor.update(processing_time)

                # Maintain target FPS
                target_loop_time = 1.0 / Config.TARGET_FPS
                if processing_time < target_loop_time:
                    time.sleep(target_loop_time - processing_time)

                # Fixed-interval transmitter independent of loop jitter
                now = time.time()
                if now - self.last_send_time >= 1.0:  # send once per second
                    data = self.sensor_data.get_data()
                    drowsy_status = 'DROWSY' if 'drowsy' in locals() and drowsy else 'AWAKE'
                    # Compose message expected by boss monitor
                    message = (
                        f"{drowsy_status},"
                        f"GPS:({data.get('lat', 0):.6f},{data.get('lon', 0):.6f}),"
                        f"TEMP:{(data.get('temperature') if data.get('temperature') is not None else 0):.1f},"
                        f"HUM:{(data.get('humidity') if data.get('humidity') is not None else 0):.1f},"
                        f"TIME:{datetime.now().strftime('%H:%M:%S')}\n"
                    )
                    if not self.hc12.connected:
                        # Attempt (re)initialization if disconnected
                        self.hc12.initialize()
                    sent_ok = self.hc12.send_data(message)
                    if sent_ok:
                        self.last_send_time = now
                    else:
                        # Log once a while to avoid spam
                        if int(now) % 5 == 0:
                            print("HC-12: send failed or not connected")

                # Optional: MQ sensor logic and heartbeat blink
                if Config.ENABLE_MQ_SENSOR:
                    mq_voltage = data.get('gas_percent', 0)
                    if self.mq_baseline is not None:
                        if mq_voltage > self.mq_baseline + self.mq_threshold:
                            print(
                                f"Gas detected! MQ voltage: {mq_voltage:.2f} V "
                                f"(baseline: {self.mq_baseline:.2f} V)"
                            )
                        else:
                            print(
                                f"No gas detected. MQ voltage: {mq_voltage:.2f} V "
                                f"(baseline: {self.mq_baseline:.2f} V)"
                            )
                if GPIO_AVAILABLE:
                    if now - self.last_blink_time > 5:
                        GPIO.output(self.blink_pin, GPIO.HIGH)
                        time.sleep(0.1)
                        GPIO.output(self.blink_pin, GPIO.LOW)
                        self.last_blink_time = now

        except KeyboardInterrupt:
            print("\nStopping system...")

        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up all resources"""
        print("Cleaning up system...")
        
        self.running = False
        
        # Stop sensors
        self.mpu6050.stop()
        self.dht.stop()
        self.gps.stop()
        
        # Clean up hardware
        if GPIO_AVAILABLE:
            GPIO.output(Config.BUZZER_PIN, GPIO.LOW)
            GPIO.output(Config.LED_PIN, GPIO.LOW)
            GPIO.cleanup()
        
        # Clean up camera and communication
        self.camera_manager.release()
        self.hc12.close()
        cv2.destroyAllWindows()
        
        print("System cleanup complete")

# =============================================================================
# MAIN EXECUTION
# =============================================================================

def main():
    """Main entry point"""
    try:
        system = HelmetSafetySystem()
        system.run()
    except Exception as e:
        print(f"System error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Ensure GPIO cleanup even on error
        if GPIO_AVAILABLE:
            try:
                GPIO.cleanup()
            except:
                pass

if __name__ == "__main__":
    main()
