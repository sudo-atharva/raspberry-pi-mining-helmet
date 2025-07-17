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
import select
import RPi.GPIO as GPIO
import psutil
import gc

# Optional imports for MQ gas sensor and MCP3008
try:
    import spidev
except ImportError:
    spidev = None

# Optional imports with error handling
try:
    import Adafruit_DHT
except ImportError:
    Adafruit_DHT = None

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

# Configuration flags for sensors (set to True when you get the hardware)
ENABLE_HC12 = False  # Set to True when you get HC-12 module
ENABLE_MQ_SENSOR = False  # Set to True when you get MQ sensor
ENABLE_DHT22 = False  # Set to True if you have DHT22 sensor
ENABLE_GPS = False  # Set to True if you have GPS module

# GPIO Setup
GPIO.setmode(GPIO.BCM)

# MQ gas sensor configuration (for future use)
if ENABLE_MQ_SENSOR:
    MQ_CHANNEL = 0  # MCP3008 channel for MQ sensor
    MQ_ALERT_PIN = 22  # GPIO pin for miner alert
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
    if not spidev or not ENABLE_MQ_SENSOR:
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
        try:
            spi.close()
        except:
            pass
        return -1

# Detect harmful gas
def detect_gas():
    if not ENABLE_MQ_SENSOR:
        return None, 0, 0
    
    value = read_mcp3008(MQ_CHANNEL)
    detected = None
    percent = 0
    for gas, info in MQ_GASES.items():
        if value >= info['threshold']:
            detected = info['type']
            percent = min(100, int((value / 1023) * 100))
            break
    return detected, percent, value

# Global variables for sensor data
sensor_data = {
    'temperature': -1,
    'humidity': -1,
    'accel_x': -1, 'accel_y': -1, 'accel_z': -1,
    'gyro_x': -1, 'gyro_y': -1, 'gyro_z': -1,
    'lat': -1, 'lon': -1
}
sensor_lock = threading.Lock()

# Performance monitoring
performance_stats = {
    'frame_count': 0,
    'processing_time': 0,
    'last_fps_check': time.time(),
    'fps': 0
}

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
FRAME_CHECK = 8  # Reduced for faster response
FACE_DETECT_INTERVAL = 2  # Detect faces every 2 frames for better performance

# Initialize face detection
detect = dlib.get_frontal_face_detector()
predict = dlib.shape_predictor("models/shape_predictor_68_face_landmarks.dat")
(lStart, lEnd) = face_utils.FACIAL_LANDMARKS_68_IDXS["left_eye"]
(rStart, rEnd) = face_utils.FACIAL_LANDMARKS_68_IDXS["right_eye"]

# Initialize camera variables
picam2 = None
cap = None

# --- Camera Setup with Performance Optimizations ---
def initialize_camera():
    """Initialize camera with aggressive performance optimizations"""
    global cap, picam2
    
    # Try USB webcam first
    try:
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            # Aggressive optimization for performance
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
            cap.set(cv2.CAP_PROP_FPS, 15)  # Reduced FPS for stability
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Faster exposure
            
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
                # Optimized config for stable performance
                config = picam2.create_preview_configuration(
                    main={"format": 'RGB888', "size": (320, 240)},
                    controls={
                        "FrameRate": 15,  # Reduced for stability
                        "AwbEnable": 0,
                        "AeEnable": 1,
                        "NoiseReductionMode": 0,
                        "Brightness": 0.1,
                        "Contrast": 1.0
                    }
                )
                picam2.configure(config)
                picam2.start()
                time.sleep(2)  # Camera warm-up time
                print("Using PiCamera2")
                return True
        except Exception as e:
            print(f"PiCamera2 error: {e}")
    
    return False

# --- GPIO Setup ---
BUTTON_PIN = 17  # Physical button for gyro calibration
LED_PIN = 27     # LED indicator
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
    """Background thread for DHT22 sensor"""
    if not Adafruit_DHT or not ENABLE_DHT22:
        return
    
    DHT_SENSOR = Adafruit_DHT.DHT22
    DHT_PIN = 4
    
    while True:
        try:
            humidity, temperature = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)
            with sensor_lock:
                sensor_data['temperature'] = temperature if temperature else -1
                sensor_data['humidity'] = humidity if humidity else -1
        except Exception:
            pass
        time.sleep(3)  # Read every 3 seconds to reduce CPU load

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
            time.sleep(0.05)  # Read every 50ms for better responsiveness
    except Exception:
        pass

def gps_thread():
    """Background thread for GPS data"""
    if not ENABLE_GPS or not os.path.exists('/dev/ttyS0'):
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

# --- Initialize HC-12 Communication ---
def initialize_hc12():
    """Initialize HC-12 communication (for future use)"""
    if not ENABLE_HC12:
        return None
    
    try:
        # Try different serial ports for HC-12
        for port in ['/dev/ttyAMA0', '/dev/ttyS0', '/dev/ttyUSB0']:
            try:
                hc12 = serial.Serial(port, 9600, timeout=1)
                print(f"HC-12 initialized on {port}")
                return hc12
            except:
                continue
        print("HC-12 initialization failed: No available ports")
        return None
    except Exception as e:
        print(f"HC-12 initialization failed: {e}")
        return None

def send_data_hc12(hc12, data):
    """Send data via HC-12"""
    if hc12 and ENABLE_HC12:
        try:
            hc12.write(data.encode())
            hc12.flush()
        except Exception as e:
            print(f"HC-12 send error: {e}")

# --- Performance monitoring ---
def update_performance_stats():
    """Update performance statistics"""
    current_time = time.time()
    performance_stats['frame_count'] += 1
    
    if current_time - performance_stats['last_fps_check'] >= 5.0:  # Every 5 seconds
        fps = performance_stats['frame_count'] / (current_time - performance_stats['last_fps_check'])
        performance_stats['fps'] = fps
        performance_stats['frame_count'] = 0
        performance_stats['last_fps_check'] = current_time
        
        # Memory usage
        memory_percent = psutil.virtual_memory().percent
        cpu_percent = psutil.cpu_percent()
        
        print(f"Performance: FPS={fps:.1f}, CPU={cpu_percent:.1f}%, Memory={memory_percent:.1f}%")
        
        # Force garbage collection if memory usage is high
        if memory_percent > 80:
            gc.collect()

# --- Main Application ---
def main():
    global COLLAPSE_ALERT
    
    if not initialize_camera():
        print("Error: No camera available")
        sys.exit(1)

    # Initialize HC-12 (will be None if not enabled)
    hc12 = initialize_hc12()

    # Start sensor threads
    if ENABLE_DHT22 and Adafruit_DHT:
        threading.Thread(target=dht_thread, daemon=True).start()
    
    if SMBus:
        threading.Thread(target=mpu6050_thread, daemon=True).start()
    
    if ENABLE_GPS:
        threading.Thread(target=gps_thread, daemon=True).start()

    # Automatic gyro calibration on startup
    time.sleep(1)  # Wait for sensor data
    calibrate_gyro()
    print("Gyro automatically calibrated on startup.")

    # Initialize variables
    flag = 0
    frame_count = 0
    last_faces = []
    last_detection_time = 0

    # Wiggling detection parameters
    WIGGLE_THRESHOLD = 25  # Reduced threshold for better detection
    WIGGLE_WINDOW = 2      # seconds to monitor for wiggling
    WIGGLE_COUNT = 3       # Reduced count for faster activation

    wiggle_events = []
    last_gyro = {'x': 0, 'y': 0, 'z': 0}
    camera_active = False

    # Face detection optimization
    face_detection_skip = 0
    cached_faces = []

    print("System ready. Camera will activate on head wiggling.")
    print("Press 'q' to quit, type 'calibrate' + Enter to recalibrate gyro")

    try:
        while True:
            loop_start = time.time()

            # --- Gyro Calibration Button ---
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                calibrate_gyro()
                time.sleep(0.5)

            # Check for terminal input (non-blocking)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                cmd = sys.stdin.readline().strip()
                if cmd.lower() == "calibrate":
                    calibrate_gyro()
                    print("Gyro calibrated via terminal command!")

            # --- Wiggling Detection ---
            with sensor_lock:
                gx = sensor_data['gyro_x'] - gyro_zero['x']
                gy = sensor_data['gyro_y'] - gyro_zero['y']
                gz = sensor_data['gyro_z'] - gyro_zero['z']

            # Calculate change in gyro
            delta_x = abs(gx - last_gyro['x'])
            delta_y = abs(gy - last_gyro['y'])
            delta_z = abs(gz - last_gyro['z'])
            last_gyro = {'x': gx, 'y': gy, 'z': gz}

            # If rapid change, record wiggle event
            if delta_x > WIGGLE_THRESHOLD or delta_y > WIGGLE_THRESHOLD or delta_z > WIGGLE_THRESHOLD:
                wiggle_events.append(loop_start)

            # Remove old wiggle events
            wiggle_events = [t for t in wiggle_events if loop_start - t < WIGGLE_WINDOW]

            # Camera activates on wiggle, deactivates if eyes detected and not drowsy
            if len(wiggle_events) >= WIGGLE_COUNT and not camera_active:
                print("Camera activated by wiggling")
                camera_active = True
                GPIO.output(LED_PIN, GPIO.HIGH)


            # --- Gas Sensor Detection (Future feature) ---
            if ENABLE_MQ_SENSOR:
                harmful_gas, gas_percent, raw_value = detect_gas()
                if harmful_gas:
                    GPIO.output(MQ_ALERT_PIN, GPIO.HIGH)
                    if hc12:
                        with sensor_lock:
                            data = f"GAS:{harmful_gas},PERCENT:{gas_percent},RAW:{raw_value},GPS:({sensor_data['lat']},{sensor_data['lon']})\n"
                        send_data_hc12(hc12, data)
                else:
                    GPIO.output(MQ_ALERT_PIN, GPIO.LOW)

            # --- Camera Processing Only When Active ---
            if camera_active:
                frame = None
                try:
                    if cap:
                        ret, frame = cap.read()
                        if not ret:
                            print("Camera read failed, reinitializing...")
                            cap.release()
                            time.sleep(1)
                            initialize_camera()
                            continue
                    elif picam2:
                        frame = picam2.capture_array()
                        if frame.shape[-1] == 4:
                            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                except Exception as e:
                    print(f"Camera error: {e}")
                    time.sleep(0.1)
                    continue

                if frame is None:
                    continue

                small_frame = imutils.resize(frame, width=160)
                gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)

                drowsy = 0
                face_found = False

                # Skip face detection on some frames for performance
                if face_detection_skip <= 0:
                    faces = detect(gray, 0)
                    if faces:
                        cached_faces = faces
                    face_detection_skip = FACE_DETECT_INTERVAL
                else:
                    faces = cached_faces
                    face_detection_skip -= 1

                # Process faces if detected
                if faces:
                    face_found = True
                    gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    for face in faces:
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
                        # Check for drowsiness
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

                # Alert logic
                if not face_found or drowsy:
                    COLLAPSE_ALERT = True
                    current_time = time.time()
                    if current_time - last_detection_time > 5:  # Send alert every 5 seconds
                        if hc12:
                            with sensor_lock:
                                alert_type = "DROWSY" if drowsy else "NO_FACE"
                                data = f"{alert_type},GPS:({sensor_data['lat']},{sensor_data['lon']}),TEMP:{sensor_data['temperature']:.1f},HUM:{sensor_data['humidity']:.1f}\n"
                            send_data_hc12(hc12, data)
                        last_detection_time = current_time
                else:
                    COLLAPSE_ALERT = False
                    # Eyes detected and not drowsy: deactivate camera until next wiggle
                    print("Camera deactivated: eyes detected and not drowsy")
                    camera_active = False
                    GPIO.output(LED_PIN, GPIO.LOW)

                cv2.imshow("Mining Helmet Safety System", frame)

            # Send regular sensor data (less frequently)
            if frame_count % 30 == 0 and hc12:  # Every 30 frames
                with sensor_lock:
                    data = f"SENSORS,TEMP:{sensor_data['temperature']:.1f},HUM:{sensor_data['humidity']:.1f},ACC:({sensor_data['accel_x']:.2f},{sensor_data['accel_y']:.2f},{sensor_data['accel_z']:.2f}),GYRO:({sensor_data['gyro_x']:.2f},{sensor_data['gyro_y']:.2f},{sensor_data['gyro_z']:.2f}),GPS:({sensor_data['lat']},{sensor_data['lon']})\n"
                send_data_hc12(hc12, data)

            frame_count += 1
            update_performance_stats()

            # Check for quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Maintain reasonable frame rate
            loop_time = time.time() - loop_start
            if loop_time < 0.066:  # ~15 FPS
                time.sleep(0.066 - loop_time)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        GPIO.cleanup()
        if cap:
            cap.release()
        if picam2:
            picam2.close()
        if hc12:
            hc12.close()
        cv2.destroyAllWindows()
        print("Cleanup complete")

if __name__ == "__main__":
    main()