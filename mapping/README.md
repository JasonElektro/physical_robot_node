# LiDAR Test Scripts

This repository contains two Python scripts for testing and streaming data from RP-Lidar A1M8 sensor to MQTT broker for ROS2/Gazebo integration.

## Files Overview

### 📁 `test_lidar.py` (Production-Ready)
- **Size**: 857 lines
- **Purpose**: Comprehensive LiDAR testing and streaming solution
- **Features**: Multiple modes, robust error handling, real-time diagnostics
- **Target**: Production use, development, and debugging

### 📁 `test_lidar_simple.py` (Quick Testing)
- **Size**: 388 lines  
- **Purpose**: Simplified LiDAR testing with library bug workarounds
- **Features**: Mock data generation, basic streaming, fallback solutions
- **Target**: Quick testing, library troubleshooting, mock data generation

## 🔧 Prerequisites

### Hardware Requirements
- **RP-Lidar A1M8** sensor
- **USB connection** to development machine
- **Power supply** for LiDAR (if required)

### Software Dependencies
```bash
# Install required Python packages
pip install rplidar-robotics
pip install paho-mqtt
```

### System Setup (Linux)
```bash
# Add user to dialout group for USB access
sudo usermod -a -G dialout $USER

# Check USB port (usually /dev/ttyUSB0)
ls -la /dev/ttyUSB*

# Set permissions if needed
sudo chmod 666 /dev/ttyUSB0
```

## 🚀 Installation & Configuration

### 1. Clone or Download Files
```bash
# Download the scripts to your project directory
# Ensure both files are in the same directory
```

### 2. Configure LiDAR Settings
Both scripts use these default settings:
```python
LIDAR_PORT = '/dev/ttyUSB0'        # USB port for LiDAR
LIDAR_BAUDRATE = 115200           # Communication speed
LIDAR_TIMEOUT = 1.0               # Connection timeout
```

### 3. Configure MQTT Settings
```python
MQTT_BROKER = "spin5.petra.ac.id"           # MQTT broker address
MQTT_PORT = 1883                            # MQTT port
MQTT_TOPIC_LIDAR_DATA = "robot/lidar/real_world_data"    # Data topic
MQTT_TOPIC_LIDAR_STATUS = "robot/lidar/status"           # Status topic
```

## 📖 Usage Guide

### Using `test_lidar.py` (Recommended for Production)

#### Run the Script
```bash
python3 test_lidar.py
```

#### Available Modes:
```
1. Test koneksi dan scanning (default)
   - Tests LiDAR connection
   - Performs scanning diagnostics  
   - Shows data format analysis
   - Displays object detection results

2. Streaming mode - kirim data ke roam.py
   - Streams real LiDAR data to MQTT
   - For Gazebo integration
   - Continuous data transmission

3. RViz visualization mode - kirim data ke RViz  
   - Streams data for RViz visualization
   - Requires mqtt_to_ros2_bridge.py running
   - Real-time visualization support

4. Exit
```

#### Example Output:
```
[LIDAR] Connected to LiDAR: {'model': 'A1M8', 'firmware': (1, 29), 'hardware': 7}
[LIDAR] Health status: ('Good', 0)
[LIDAR] Scan 1 successful: 380 points
[LIDAR] ✓ Sent real data: 380 points -> 350 valid ranges
```

### Using `test_lidar_simple.py` (Troubleshooting & Mock Data)

#### Run the Script
```bash
python3 test_lidar_simple.py
```

#### Available Modes:
```
1. Mock data streaming (RECOMMENDED - works around library bug)
   - Generates realistic mock LiDAR data
   - Bypasses rplidar library issues
   - Creates obstacles at specific angles
   - Guaranteed to work for testing

2. Real LiDAR streaming (may fail due to library bug)
   - Attempts to use real LiDAR hardware
   - May encounter library compatibility issues
   - Fallback to mock mode if fails

3. Exit
```

#### Mock Data Features:
- **90° obstacle**: 80cm distance
- **180° obstacle**: 120cm distance  
- **270° obstacle**: 60cm distance
- **Other directions**: 3m (no obstacle)

## 🔍 Key Differences

| Feature | test_lidar.py | test_lidar_simple.py |
|---------|---------------|---------------------|
| **Complexity** | High (857 lines) | Low (388 lines) |
| **Error Handling** | Comprehensive retry logic | Basic with fallbacks |
| **Mock Data** | ❌ Not available | ✅ Built-in mock generation |
| **Test Modes** | 4 modes | 3 modes |
| **Classes** | 3 classes | 1 class |
| **Production Ready** | ✅ Yes | ❌ Testing only |
| **Library Bug Workaround** | ❌ Limited | ✅ Explicit support |
| **Diagnostics** | ✅ Comprehensive | ❌ Basic |

## 🛠️ Troubleshooting

### Common Issues

#### 1. "Import rplidar could not be resolved"
```bash
# Install the correct package
pip install rplidar-robotics

# Or try alternative
pip install rplidar
```

#### 2. "Permission denied" on /dev/ttyUSB0
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Logout and login again, or reboot
# Or temporary fix:
sudo chmod 666 /dev/ttyUSB0
```

#### 3. "iter_scans() library issue: too many values to unpack"
This is a known issue with some rplidar library versions.

**Solution**: Use `test_lidar_simple.py` with mock mode:
```
Choose mode: 1 (Mock data streaming)
```

#### 4. LiDAR not detected
```bash
# Check USB connection
lsusb | grep -i lidar

# Check available ports
ls -la /dev/ttyUSB*

# Try different ports
LIDAR_PORT = '/dev/ttyUSB1'  # or /dev/ttyACM0
```

#### 5. MQTT connection failed
```bash
# Check network connectivity
ping spin5.petra.ac.id

# Check if broker is running
telnet spin5.petra.ac.id 1883
```

### Error Messages & Solutions

| Error | Solution |
|-------|----------|
| `"too many values to unpack"` | Use `test_lidar_simple.py` mock mode |
| `"Connection timed out"` | Check USB cable and power supply |
| `"No module named 'rplidar'"` | Install: `pip install rplidar-robotics` |
| `"Permission denied"` | Add user to dialout group |
| `"MQTT connection failed"` | Check broker address and network |

## 📊 Data Format

Both scripts output LiDAR data in this format:
```python
{
    "timestamp": 1234567890.123,
    "ranges": [0.8, 1.2, 3.0, ...],  # 360 values in meters
    "angle_min": 0.0,
    "angle_max": 6.283185307179586,  # 2π radians
    "angle_increment": 0.017453292519943295,  # 1° in radians
    "range_min": 0.15,
    "range_max": 12.0,
    "source": "real_world_lidar" | "real_world_lidar_rviz_mock"
}
```

## 🎯 When to Use Which Script

### Use `test_lidar.py` when:
- ✅ You need production-ready solution
- ✅ You want comprehensive diagnostics
- ✅ You need multiple streaming modes
- ✅ You want robust error handling
- ✅ LiDAR hardware is working properly

### Use `test_lidar_simple.py` when:
- ✅ You're experiencing library compatibility issues
- ✅ You need quick testing with mock data
- ✅ You want to bypass hardware problems temporarily
- ✅ You're troubleshooting MQTT connectivity
- ✅ You need guaranteed working solution for testing

## 🔗 Integration

### With Gazebo (roam.py)
```bash
# 1. Run the LiDAR script
python3 test_lidar.py
# Choose mode: 2 (Streaming mode)

# 2. Run your Gazebo simulation
# The robot will receive real-world obstacle data
```

### With RViz Visualization
```bash
# 1. Start ROS2 bridge
python3 mqtt_to_ros2_bridge.py

# 2. Run LiDAR script  
python3 test_lidar.py
# Choose mode: 3 (RViz visualization)

# 3. Open RViz and subscribe to LaserScan topic
```

## 📝 Notes

- **Mock data in `test_lidar_simple.py`** is useful for testing MQTT connectivity without hardware
- **Library bugs** are common with rplidar packages - use simple version as fallback
- **MQTT topics** are configured for integration with existing ROS2 bridge
- **Data rate** is approximately 10 Hz for real-time applications
- **Range data** is converted to meters for ROS2 compatibility

## 🐛 Known Issues

1. **Duplicate method definitions** in `test_lidar_simple.py` (lines 127, 169, 211)
2. **Library unpacking errors** with certain rplidar versions
3. **Permission issues** on some Linux distributions
4. **USB disconnection** during long-running sessions

## 🔮 Future Improvements

- [ ] Fix duplicate method definitions
- [ ] Add configuration file support
- [ ] Implement data logging functionality
- [ ] Add web interface for monitoring
- [ ] Support for multiple LiDAR sensors
- [ ] Add data filtering and smoothing options

---

**Author**: Robot Development Team  
**Last Updated**: 2024  
**License**: MIT License 
