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

try:
    import Adafruit_DHT
    DHT_AVAILABLE = True
except ImportError:
    DHT_AVAILABLE = False

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
    ENABLE_HC12 = False
    ENABLE_MQ_SENSOR = False
    ENABLE_DHT22 = True
    ENABLE_GPS = True
    ENABLE_MPU6050 = True
    
    # GPIO Pins
    BUZZER_PIN = 23
    BUTTON_PIN = 17
    LED_PIN = 27
    MQ_ALERT_PIN = 22
    
    # Sensor settings
    DHT_PIN = 4
    MQ_CHANNEL = 0
    
    # Camera settings
    CAMERA_WIDTH = 320
    CAMERA_HEIGHT = 240
    CAMERA_FPS = 20
    
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

class DHT22Sensor:
    """DHT22 temperature and humidity sensor"""
    
    def __init__(self, sensor_data):
        self.sensor_data = sensor_data
        self.running = False
        self.thread = None
    
    def start(self):
        """Start DHT22 reading thread"""
        if not DHT_AVAILABLE or not Config.ENABLE_DHT22:
            return
        
        self.running = True
        self.thread = threading.Thread(target=self._sensor_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """Stop DHT22 reading"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
    
    def _sensor_loop(self):
        """Main DHT22 reading loop with debug prints"""
        while self.running:
            try:
                humidity, temperature = Adafruit_DHT.read_retry(
                    Adafruit_DHT.DHT22, Config.DHT_PIN, delay_seconds=0.1
                )
                print(f"[DHT22] Read: Temp={temperature}, Hum={humidity}")
                if humidity is not None and temperature is not None:
                    self.sensor_data.update(temperature=temperature, humidity=humidity)
            except Exception as e:
                print(f"DHT22 read error: {e}")
            time.sleep(Config.DHT_UPDATE_INTERVAL)

class GPSSensor:
    """GPS sensor for location tracking"""
    
    def __init__(self, sensor_data):
        self.sensor_data = sensor_data
        self.running = False
        self.thread = None
        self.serial_port = None
    
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
                    return frame if ret else None
                elif self.picam2:
                    frame = self.picam2.capture_array()
                    if frame.shape[-1] == 4:
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                    return frame
            except Exception as e:
                print(f"Camera read error: {e}")
                return None
        
        return None
    
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
        """Initialize HC-12 module"""
        if not Config.ENABLE_HC12:
            return False
        
        # Try different serial ports
        for port in ['/dev/ttyAMA0', '/dev/ttyS0', '/dev/ttyUSB0']:
            try:
                self.serial_port = serial.Serial(port, Config.HC12_BAUDRATE, timeout=1)
                print(f"HC-12 initialized on {port}")
                self.connected = True
                return True
            except:
                continue
        
        print("HC-12 initialization failed")
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
        self.dht22 = DHT22Sensor(self.sensor_data)
        self.gps = GPSSensor(self.sensor_data)
        
        # Initialize GPIO
        self.setup_gpio()

        # CSV log file
        self.csv_log_path = os.path.abspath("helmet_log.csv")
        self._init_csv_log()

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
            print("Error: Camera initialization failed")
            return False
        
        # Initialize communication
        self.hc12.initialize()
        
        # Start sensors
        if Config.ENABLE_MPU6050:
            self.mpu6050.start()
            time.sleep(1)  # Wait for initial readings
            self.sensor_data.calibrate_gyro()
            print("MPU6050 calibrated")
        
        if Config.ENABLE_DHT22:
            self.dht22.start()
        
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

                            cv2.imshow("Mining Helmet Safety System", processed_frame)

                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                self.running = False

                # Update performance monitoring
                processing_time = time.time() - loop_start
                self.performance_monitor.update(processing_time)

                # Maintain target FPS
                target_loop_time = 1.0 / Config.TARGET_FPS
                if processing_time < target_loop_time:
                    time.sleep(target_loop_time - processing_time)

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
        self.dht22.stop()
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
