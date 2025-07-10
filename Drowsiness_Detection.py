from scipy.spatial import distance
from imutils import face_utils
import imutils
import dlib
import cv2
import platform
import importlib
import serial
import time
import math
try:
    import Adafruit_DHT
except ImportError:
    Adafruit_DHT = None
try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

# --- Drowsiness Detection Setup ---
def eye_aspect_ratio(eye):
    A = distance.euclidean(eye[1], eye[5])
    B = distance.euclidean(eye[2], eye[4])
    C = distance.euclidean(eye[0], eye[3])
    ear = (A + B) / (2.0 * C)
    return ear

# --- Robust UART Setup (HC-12) ---
uart = None
try:
    uart = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)
except Exception:
    uart = None
frame_check = 20
detect = dlib.get_frontal_face_detector()
predict = dlib.shape_predictor("models/shape_predictor_68_face_landmarks.dat")
(lStart, lEnd) = face_utils.FACIAL_LANDMARKS_68_IDXS["left_eye"]
(rStart, rEnd) = face_utils.FACIAL_LANDMARKS_68_IDXS["right_eye"]
picam2 = None
cap = None
# --- Robust Camera Selection: USB webcam or PiCamera2 ---
if platform.system() == 'Linux':
    try:
        # Try USB webcam first
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            cap = None
    except Exception:
        cap = None
    if cap is None:
        try:
            picamera2_mod = importlib.util.find_spec("picamera2")
            if picamera2_mod is not None:
                from picamera2 import Picamera2
                picam2 = Picamera2()
                picam2.start()
        except Exception:
            picam2 = None
else:
    try:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            cap = None
    except Exception:
        cap = None
flag = 0

# --- DHT11 Setup ---
if Adafruit_DHT:
    DHT_SENSOR = Adafruit_DHT.DHT11
    DHT_PIN = 4  # GPIO pin
else:
    DHT_SENSOR = None
    DHT_PIN = None

# --- MPU6050 Setup ---
if SMBus:
    MPU6050_ADDR = 0x68
    bus = SMBus(1)
    bus.write_byte_data(MPU6050_ADDR, 0x6B, 0)  # Wake up MPU6050
    def read_mpu6050():
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
else:
    def read_mpu6050():
        return -1, -1, -1, -1, -1, -1

# --- GPS Setup (NEO-6M) ---
try:
    gps_serial = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=1)
    def read_gps():
        try:
            line = gps_serial.readline().decode('ascii', errors='replace')
            if line.startswith('$GPGGA'):
                parts = line.split(',')
                if len(parts) > 5 and parts[2] and parts[4]:
                    lat = float(parts[2])
                    lon = float(parts[4])
                    return lat, lon
        except Exception:
            pass
        return None, None
except Exception:
    gps_serial = None
    def read_gps():
        return None, None

# --- UART Setup (HC-12) ---
try:
    uart = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)
except Exception:
    uart = None


while True:
    # Camera frame capture (USB webcam or PiCamera2)
    frame = None
    ret = False
    if cap is not None:
        ret, frame = cap.read()
    elif picam2 is not None:
        try:
            frame = picam2.capture_array()
            ret = frame is not None
        except Exception:
            ret = False
    if not ret or frame is None:
        print("No camera available or failed to capture frame.")
        break
    frame = imutils.resize(frame, width=450)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    subjects = detect(gray, 0)
    drowsy = 0
    for subject in subjects:
        shape = predict(gray, subject)
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
        if ear < thresh:
            flag += 1
            if flag >= frame_check:
                cv2.putText(frame, "****************ALERT!****************", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(frame, "****************ALERT!****************", (10,325),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                drowsy = 1
        else:
            flag = 0
    cv2.imshow("Frame", frame)


    # DHT11 Reading
    if Adafruit_DHT and DHT_SENSOR and DHT_PIN is not None:
        humidity, temperature = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)
        if humidity is None or temperature is None:
            humidity, temperature = -1, -1
    else:
        humidity, temperature = -1, -1

    # MPU6050 Reading
    try:
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = read_mpu6050()
    except Exception:
        accel_x = accel_y = accel_z = gyro_x = gyro_y = gyro_z = -1

    # GPS Reading
    lat, lon = read_gps()
    if lat is None or lon is None:
        lat, lon = -1, -1

    # Prepare data string
    data = f"DROWSY:{drowsy},TEMP:{temperature:.1f},HUM:{humidity:.1f},ACC:({accel_x:.2f},{accel_y:.2f},{accel_z:.2f}),GYRO:({gyro_x:.2f},{gyro_y:.2f},{gyro_z:.2f}),GPS:({lat},{lon})\n"
    print(data)

    # Send data via UART (HC-12)
    if uart:
        try:
            uart.write(data.encode())
        except Exception as e:
            print("UART send error:", e)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

# --- Cleanup ---
if 'gps_serial' in locals() and gps_serial:
    gps_serial.close()
if SMBus and 'bus' in locals():
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
