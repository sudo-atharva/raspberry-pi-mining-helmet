from scipy.spatial import distance
from imutils import face_utils
import imutils
import dlib
import cv2
import platform
import importlib
import serial
import time
from collections import deque  # For FPS calculation
from datetime import datetime
import math
import os
import sys

# Optional imports with error handling
try:
    import Adafruit_DHT
except ImportError:
    Adafruit_DHT = None
    print("Warning: Adafruit_DHT not installed. DHT11 sensor will be disabled.")

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None
    print("Warning: smbus2 not installed. MPU6050 sensor will be disabled.")

# --- Drowsiness Detection Setup ---
def eye_aspect_ratio(eye):
    """Calculate the eye aspect ratio (EAR) for drowsiness detection"""
    A = distance.euclidean(eye[1], eye[5])
    B = distance.euclidean(eye[2], eye[4])
    C = distance.euclidean(eye[0], eye[3])
    ear = (A + B) / (2.0 * C)
    return ear

def try_open_uart(port):
    """Try to open UART port with error handling"""
    if os.path.exists(port):
        try:
            return serial.Serial(port, baudrate=9600, timeout=1)
        except Exception as e:
            print(f"Warning: Could not open UART port {port}: {e}")
    else:
        print(f"Warning: UART port {port} not found.")
    return None

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
EAR_THRESHOLD = 0.25  # Eye aspect ratio threshold - FIXED: was using undefined 'thresh'
frame_check = 20
flag = 0

# Initialize face detection
detect = dlib.get_frontal_face_detector()
predict = dlib.shape_predictor("models/shape_predictor_68_face_landmarks.dat")
(lStart, lEnd) = face_utils.FACIAL_LANDMARKS_68_IDXS["left_eye"]
(rStart, rEnd) = face_utils.FACIAL_LANDMARKS_68_IDXS["right_eye"]

# Initialize camera variables
picam2 = None
cap = None

# --- Robust Camera Selection: USB webcam or PiCamera2 ---
def get_fps(frame_times, n=30):
    """Calculate FPS from a deque of frame timestamps"""
    if len(frame_times) < 2:
        return 0
    return len(frame_times)/(frame_times[-1] - frame_times[0])

def initialize_camera():
    """Initialize camera with fallback options"""
    global cap, picam2
    
    # Try USB webcam first
    try:
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            # Set optimized camera parameters
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            # Test camera
            ret, frame = cap.read()
            if ret and frame is not None:
                print("Using USB webcam via OpenCV.")
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
                # Optimize PiCamera2 settings for performance
                picam2.configure(picam2.create_preview_configuration(
                    main={"format": 'RGB888',  # Direct RGB format
                          "size": (640, 480)},
                    controls={"FrameRate": 30,
                             "AwbEnable": 0,  # Disable auto white balance
                             "AeEnable": 1,   # Keep auto exposure
                             "NoiseReductionMode": 0}  # Minimal noise reduction
                ))
                picam2.start()
                print("Using PiCamera2.")
                return True
            else:
                print("PiCamera2 not available.")
        except Exception as e:
            print(f"Error initializing PiCamera2: {e}")
            picam2 = None
    
    print("No camera found: neither USB webcam nor PiCamera2 available.")
    return False

# Initialize camera
if not initialize_camera():
    print("Error: No camera available. Exiting.")
    sys.exit(1)

# --- DHT11 Setup ---
if Adafruit_DHT:
    DHT_SENSOR = Adafruit_DHT.DHT11
    DHT_PIN = 4  # GPIO pin
    print("DHT11 sensor initialized on GPIO pin 4")
else:
    DHT_SENSOR = None
    DHT_PIN = None

# --- MPU6050 Setup ---
bus = None
if SMBus:
    try:
        MPU6050_ADDR = 0x68
        bus = SMBus(1)
        bus.write_byte_data(MPU6050_ADDR, 0x6B, 0)  # Wake up MPU6050
        print("MPU6050 sensor initialized")
    except Exception as e:
        print(f"Error initializing MPU6050: {e}")
        bus = None

def read_mpu6050():
    """Read MPU6050 sensor data"""
    if bus is None:
        return -1, -1, -1, -1, -1, -1
    
    try:
        def read_word(reg):
            h = bus.read_byte_data(MPU6050_ADDR, reg)
            l = bus.read_byte_data(MPU6050_ADDR, reg+1)
            val = (h << 8) + l
            if val >= 0x8000:
                val = -((65535 - val) + 1)
            return val
        
        accel_x = read_word(0x3B) / 16384.0
        accel_y = read_word(0x3D) / 16384.0
        accel_z = read_word(0x3F) / 16384.0
        gyro_x = read_word(0x43) / 131.0
        gyro_y = read_word(0x45) / 131.0
        gyro_z = read_word(0x47) / 131.0
        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
    except Exception as e:
        print(f"Error reading MPU6050: {e}")
        return -1, -1, -1, -1, -1, -1

# --- GPS Setup (NEO-6M) ---
def try_open_gps(port):
    """Initialize GPS module"""
    if os.path.exists(port):
        try:
            gps_serial = serial.Serial(port, baudrate=9600, timeout=1)
            print(f"GPS module initialized on {port}")
            
            def read_gps():
                try:
                    line = gps_serial.readline().decode('ascii', errors='replace')
                    if line.startswith('$GPGGA'):
                        parts = line.split(',')
                        if len(parts) > 5 and parts[2] and parts[4]:
                            # Convert from DDMM.MMMM format to decimal degrees
                            lat_raw = float(parts[2])
                            lat = int(lat_raw/100) + (lat_raw % 100)/60
                            if parts[3] == 'S':
                                lat = -lat
                            
                            lon_raw = float(parts[4])
                            lon = int(lon_raw/100) + (lon_raw % 100)/60
                            if parts[5] == 'W':
                                lon = -lon
                            
                            return lat, lon
                except Exception as e:
                    print(f"GPS read error: {e}")
                return None, None
            
            return gps_serial, read_gps
        except Exception as e:
            print(f"Warning: Could not open GPS port {port}: {e}")
    else:
        print(f"Warning: GPS port {port} not found.")
    
    def read_gps():
        return None, None
    return None, read_gps

# Initialize GPS
gps_serial, read_gps = try_open_gps('/dev/ttyS0')

# --- UART Setup (HC-12) ---
uart = try_open_uart('/dev/ttyUSB0')
if uart:
    print("UART communication initialized")

print("Starting drowsiness detection system...")
print("Press 'q' to quit")

# Initialize performance monitoring
frame_times = deque(maxlen=30)  # Store last 30 frame timestamps
last_fps_print = time.time()
fps_print_interval = 5  # Print FPS every 5 seconds

# Initialize face detection ROI
roi_scale = 1.0  # Dynamic ROI scaling
min_roi_scale = 0.3  # Minimum ROI scale
max_roi_scale = 1.0  # Maximum ROI scale
target_fps = 15  # Target FPS

# Main loop
try:
    while True:
        loop_start = time.time()
        
        # Camera frame capture
        frame = None
        ret = False
        
        if cap is not None:
            ret, frame = cap.read()
        elif picam2 is not None:
            try:
                frame = picam2.capture_array()
                if frame is not None:
                    if frame.shape[-1] == 4:  # Only convert if 4 channels
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                    ret = True
            except Exception as e:
                print(f"PiCamera2 capture error: {e}")
                ret = False
        
        if not ret or frame is None:
            print("Failed to capture frame. Retrying...")
            time.sleep(0.1)
            continue
        
        # Dynamic frame resizing based on performance
        frame = imutils.resize(frame, width=int(640 * roi_scale))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Performance monitoring
        frame_times.append(loop_start)
        current_fps = get_fps(frame_times)
        
        # Adjust ROI scale based on FPS
        if current_fps < target_fps - 2 and roi_scale > min_roi_scale:
            roi_scale = max(roi_scale * 0.95, min_roi_scale)
        elif current_fps > target_fps + 2 and roi_scale < max_roi_scale:
            roi_scale = min(roi_scale * 1.05, max_roi_scale)
        
        # Print FPS periodically
        if time.time() - last_fps_print > fps_print_interval:
            print(f"FPS: {current_fps:.1f}, ROI Scale: {roi_scale:.2f}")
            last_fps_print = time.time()
        
        # Face detection with ROI optimization
        subjects = detect(gray, 0)  # 0 = Don't upsample image
        drowsy = 0
        
        for subject in subjects:
            shape = predict(gray, subject)
            shape = face_utils.shape_to_np(shape)
            
            # Extract and process eye regions
            leftEye = shape[lStart:lEnd]
            rightEye = shape[rStart:rEnd]
            
            # Calculate eye aspect ratios (EAR)
            leftEAR = eye_aspect_ratio(leftEye)
            rightEAR = eye_aspect_ratio(rightEye)
            ear = (leftEAR + rightEAR) / 2.0
            
            # Visualize eye detection
            leftEyeHull = cv2.convexHull(leftEye)
            rightEyeHull = cv2.convexHull(rightEye)
            cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
            cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)
            
            # Drowsiness detection based on EAR
            if ear < EAR_THRESHOLD:
                flag += 1
                if flag >= frame_check:
                    cv2.putText(frame, "****************ALERT!****************", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.putText(frame, "****************ALERT!****************", (10, 325),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    drowsy = 1
            else:
                flag = 0
            
            # Display metrics
            cv2.putText(frame, f"EAR: {ear:.2f}", (300, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"FPS: {current_fps:.1f}", (300, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Display frame (skip if we're falling behind)
        if current_fps < 5:  # Emergency frame skip if performance is very poor
            skip_display = frame_times[-1] % 2 == 0  # Skip every other frame
        else:
            skip_display = False
        
        if not skip_display:
            cv2.imshow("Drowsiness Detection", frame)
        
        # Read sensors
        # DHT11 Reading
        if Adafruit_DHT and DHT_SENSOR and DHT_PIN is not None:
            humidity, temperature = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)
            if humidity is None or temperature is None:
                humidity, temperature = -1, -1
        else:
            humidity, temperature = -1, -1
        
        # MPU6050 Reading
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = read_mpu6050()
        
        # GPS Reading
        lat, lon = read_gps()
        if lat is None or lon is None:
            lat, lon = -1, -1
        
        # Prepare data string
        data = f"DROWSY:{drowsy},TEMP:{temperature:.1f},HUM:{humidity:.1f},ACC:({accel_x:.2f},{accel_y:.2f},{accel_z:.2f}),GYRO:({gyro_x:.2f},{gyro_y:.2f},{gyro_z:.2f}),GPS:({lat},{lon})\n"
        print(data.strip())
        
        # Send data via UART (HC-12)
        if uart:
            try:
                uart.write(data.encode())
            except Exception as e:
                print(f"UART send error: {e}")
        
        # Check for exit key
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        
        # Small delay to prevent excessive CPU usage
        elapsed = time.time() - loop_start
        if elapsed < 1/30:  # Cap at 30 FPS maximum
            time.sleep(1/30 - elapsed)

except KeyboardInterrupt:
    print("\nProgram interrupted by user")
except Exception as e:
    print(f"Unexpected error: {e}")

finally:
    # --- Cleanup ---
    print("Cleaning up...")
    
    if 'gps_serial' in locals() and gps_serial:
        gps_serial.close()
    
    if bus is not None:
        bus.close()
    
    cv2.destroyAllWindows()
    
    if cap is not None:
        cap.release()
    
    if picam2 is not None:
        try:
            picam2.close()
        except Exception:
            pass
    
    if uart:
        uart.close()
    
    print("Cleanup complete. Goodbye!")