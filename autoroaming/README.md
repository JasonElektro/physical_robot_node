# BCR Bot Standalone - README

## Deskripsi

BCR Bot Standalone adalah sistem navigasi dan kontrol motor yang terintegrasi untuk robot otonom. Sistem ini menggunakan LiDAR A1M8 untuk deteksi obstacle dan navigasi real-world tanpa simulasi Gazebo.

## Fitur Utama

### 🚀 **Navigasi**
- **Primary Mode**: Navigasi otomatis dengan obstacle avoidance dasar
- **R-Mode**: Obstacle avoidance tingkat lanjut dengan logika yang lebih canggih
- **Charging Navigation**: Mode navigasi khusus untuk pencarian charging station
- **Manual Mode**: Kontrol manual menggunakan keyboard

### 🔍 **Sensor & Hardware**
- **LiDAR A1M8**: Sensor utama untuk deteksi obstacle 360°
- **PCA9685**: Motor controller untuk 4 motor mecanum wheel
- **Raspberry Pi**: Platform utama
- **MQTT**: Transmisi data mapping real-time

### 🎮 **Kontrol**
- Keyboard control (WASD + special keys)
- Emergency stop dengan spacebar
- Mode switching yang mudah
- Status monitoring real-time

## Requirements

### Hardware
- Raspberry Pi 4 (recommended)
- LiDAR A1M8 dengan kabel USB
- PCA9685 PWM driver
- 4x Motor DC dengan encoder
- Power supply 12V untuk motor
- Kabel jumper dan breadboard

### Software Dependencies
```bash
# Install Python packages
pip install adafruit-circuitpython-pca9685
pip install rplidar
pip install paho-mqtt
pip install RPi.GPIO
```

### System Requirements
- Python 3.7+
- Raspberry Pi OS
- USB port untuk LiDAR
- I2C enabled untuk PCA9685

## Installation

### 1. Clone Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies
```bash
# Update system
sudo apt update
sudo apt upgrade

# Install Python packages
pip install -r requirements.txt

# Enable I2C dan Serial
sudo raspi-config
# Navigate to Interfacing Options -> I2C -> Enable
# Navigate to Interfacing Options -> Serial -> Enable
```

### 3. Hardware Setup

#### LiDAR A1M8 Connection
```
LiDAR A1M8 -> Raspberry Pi
USB Cable -> USB Port (/dev/ttyUSB0)
```

#### PCA9685 Connection
```
PCA9685 -> Raspberry Pi
VCC -> 5V
GND -> GND
SDA -> GPIO2 (Pin 3)
SCL -> GPIO3 (Pin 5)
```

#### Motor Pin Mapping
```
Motor 1 (Depan Kiri):  PWM=15, DIR=14
Motor 2 (Depan Kanan): PWM=3,  DIR=2
Motor 3 (Belakang Kanan): PWM=11, DIR=10
Motor 4 (Belakang Kiri): PWM=7,  DIR=6
```

### 4. Permissions Setup
```bash
# Give permission to USB port
sudo chmod 666 /dev/ttyUSB0

# Add user to dialout group
sudo usermod -a -G dialout $USER
```

## Usage

### 1. Running the Robot
```bash
python3 robot_lokal.py
```

### 2. Keyboard Controls

#### Movement Controls
- **W**: Move Forward
- **S**: Move Backward  
- **A**: Turn Left
- **D**: Turn Right
- **SPACE**: Emergency Stop (Stop all motors)

#### Navigation Mode Controls
- **T**: Toggle Charging Navigation Mode
- **R**: Toggle R-Mode (Advanced obstacle avoidance)

#### Control Logic
- Manual mode aktif saat ada input keyboard
- Primary mode resume otomatis setelah 5 detik tanpa input
- Tekan Ctrl+C untuk keluar dengan aman

### 3. Navigation Modes

#### Primary Mode (Default)
- Navigasi otomatis sederhana
- Obstacle avoidance dasar
- Bergerak maju, belok kiri/kanan jika ada obstacle
- Cocok untuk environment yang tidak terlalu kompleks

#### R-Mode (Advanced)
- Obstacle avoidance tingkat lanjut
- Logika yang lebih canggih untuk situasi kompleks
- Dapat handling multiple obstacles
- Backing up dan turn around jika kedua sisi terblokir

#### Charging Navigation
- Mode khusus untuk pencarian charging station
- Gerakan yang lebih hati-hati dan lambat
- Prioritas keamanan tinggi
- Simulasi kondisi battery low

### 4. LiDAR Orientation
```
Orientasi LiDAR A1M8:
    Front (180°)
Left(90°)  Right(270°)
     Rear (0°)
```

## Configuration

### LiDAR Settings
```python
LIDAR_PORT = '/dev/ttyUSB0'        # Port LiDAR
LIDAR_BAUDRATE = 115200            # Baudrate
LIDAR_TIMEOUT = 1.0                # Timeout
```

### Obstacle Detection Thresholds
```python
obstacle_threshold = 1.0           # Deteksi obstacle (meter)
emergency_stop_threshold = 0.4     # Emergency stop (meter)
```

### Motor Speed Configuration
```python
speedCoef = 0.4                    # Primary mode speed
rmodeSpeedCoef = 0.5               # R-mode speed
rmodeBackwardSpeedCoef = 0.6       # R-mode backward speed
```

### MQTT Configuration
```python
MQTT_BROKER = "spin5.petra.ac.id"
MQTT_PORT = 1883
MQTT_TOPIC = "robot/lidar/real_world_data"
```

## Monitoring & Debugging

### Real-time Status Display
Program menampilkan status real-time yang mencakup:
- Timestamp
- Current action
- LiDAR distances (Front, Left, Right)
- MQTT connection status
- Motor status

### LiDAR Data Logging
```
[LIDAR A1M8] CORRECTED ORIENTATION:
[LIDAR A1M8]   Front (180°)=2.45m
[LIDAR A1M8]   Left  (90°)=1.23m
[LIDAR A1M8]   Right (270°)=3.67m
```

### MQTT Data Transmission
Data LiDAR dikirim ke MQTT broker untuk mapping:
- Topic: `robot/lidar/real_world_data`
- Format: JSON dengan ranges, timestamps, dan obstacle status
- Compatible dengan `mqtt_to_ros2_bridge.py`

## Troubleshooting

### LiDAR Issues

#### LiDAR Not Detected
```bash
# Check USB connections
ls /dev/ttyUSB*

# Check permissions
sudo chmod 666 /dev/ttyUSB0

# Check if device is recognized
dmesg | tail
```

#### Port Permission Denied
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Logout and login again
```

#### LiDAR Data Issues
- Pastikan LiDAR berputar
- Check cable connection
- Verify power supply
- Try different USB port

### Motor Issues

#### Motors Not Moving
- Check PCA9685 connection
- Verify I2C is enabled: `sudo i2cdetect -y 1`
- Check power supply to motors
- Verify pin connections

#### Erratic Movement
- Check motor encoder connections
- Verify motor directions
- Check PWM frequency setting
- Ensure stable power supply

### MQTT Issues

#### Connection Failed
- Check WiFi connection
- Verify broker address: `spin5.petra.ac.id`
- Check firewall settings
- Try manual connection: `mosquitto_pub -h spin5.petra.ac.id -t test -m "hello"`

### General Issues

#### Import Errors
```bash
# Install missing packages
pip install adafruit-circuitpython-pca9685
pip install rplidar
pip install paho-mqtt
```

#### Keyboard Input Not Working
- Check terminal settings
- Verify stdin permissions
- Try running with `sudo` (not recommended for production)

## Safety Features

### Emergency Stop
- **Spacebar**: Immediate stop semua motor
- **Ctrl+C**: Graceful shutdown dengan cleanup
- **Obstacle Detection**: Auto-stop jika obstacle terlalu dekat (<0.25m)

### Graceful Shutdown
Program melakukan cleanup otomatis saat exit:
- Stop semua motor
- Disconnect LiDAR
- Close MQTT connection
- Restore terminal settings
- Clear screen

## Development & Customization

### Adding New Navigation Modes
1. Buat method baru `handle_custom_mode()`
2. Tambahkan key handler di `handle_key()`
3. Integrate ke main control loop

### Modifying Obstacle Thresholds
Edit values di `__init__()`:
```python
self.obstacle_threshold = 1.0          # Adjust as needed
self.emergency_stop_threshold = 0.4    # Critical distance
```

### Custom Motor Behaviors
Modify movement methods:
- `move_forward()`
- `move_backward()`
- `turn_left()`
- `turn_right()`

## License

[Specify your license here]

## Contact & Support

[Add your contact information]

---

**⚠️ Safety Notice**: 
- Selalu test di environment yang aman
- Pastikan ada emergency stop yang mudah dijangkau
- Monitor robot saat beroperasi
- Jangan tinggalkan robot tanpa pengawasan

**🔧 Maintenance**:
- Regular cleaning LiDAR lens
- Check motor connections
- Verify power supply stability
- Update software dependencies 
