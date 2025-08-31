# 🛡️ Mining Helmet Safety System

A comprehensive IoT-based safety monitoring system for mining workers featuring real-time drowsiness detection, environmental monitoring, and wireless alert transmission.

## 🌟 Features

- **Real-time Drowsiness Detection**: Computer vision-based eye tracking
- **Environmental Monitoring**: Temperature, humidity, gas detection
- **Motion Detection**: Head movement using MPU6050
- **GPS Tracking**: Real-time location monitoring
- **Wireless Communication**: HC-12 radio for remote monitoring
- **Live Charts**: Temperature/humidity graphs in boss monitor
- **Alert System**: Visual, audio, and wireless notifications

## 🏗️ System Architecture

```
HELMET (Sender) → HC-12 Radio → BOSS MONITOR (Receiver)
     ↓                    ↓              ↓
  Camera + Sensors    Wireless Link   GUI + Charts
```

## 📋 Requirements

### Hardware
- **Helmet Side**: Raspberry Pi, Camera, MPU6050, DHT sensor, MQ gas sensor, GPS, HC-12
- **Monitor Side**: Computer with USB port, HC-12 module

### Software
```bash
pip install opencv-python dlib imutils numpy scipy pyserial psutil
pip install RPi.GPIO smbus2 adafruit-circuitpython-dht spidev
```

## 🚀 Installation

1. **Clone Repository**
```bash
git clone <repository-url>
cd raspberry-pi-mining-helmet-1
```

2. **Install Dependencies**
```bash
pip install -r requirements.txt
```

3. **Download Face Model**
```bash
wget http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2
bunzip2 shape_predictor_68_face_landmarks.dat.bz2
mv shape_predictor_68_face_landmarks.dat models/
```

## 🎯 Usage

### Start Helmet System
```bash
python3 main.py
```

### Start Boss Monitor
```bash
python3 boss_monitor_gui.py
```

## 📊 Data Format

HC-12 messages: `STATUS,GPS:(lat,lon),TEMP:xx.x,HUM:yy.y,TIME:HH:MM:SS`

Examples:
- `AWAKE,GPS:(12.345678,98.765432),TEMP:25.1,HUM:60.2,TIME:14:30:25`
- `DROWSY,GPS:(12.345678,98.765432),TEMP:25.1,HUM:60.2,TIME:14:30:26`

## 🔧 Troubleshooting

- **No Connection**: Check HC-12 channel/baud settings
- **Camera Issues**: Verify USB connections and permissions
- **Sensors**: Check I2C/GPIO connections and power

## 📄 License

MIT License - see LICENSE.txt for details.

---

**⚠️ Safety Notice**: This system is for safety monitoring and should not replace proper safety protocols.
