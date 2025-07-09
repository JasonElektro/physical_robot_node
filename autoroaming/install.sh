#!/bin/bash

# BCR Bot Standalone - Installation Script
# Run with: bash install.sh

set -e  # Exit on any error

echo "============================================"
echo "  BCR Bot Standalone Installation Script  "
echo "============================================"

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null; then
    echo "⚠️  Warning: This script is designed for Raspberry Pi"
    echo "   Some features may not work on other systems"
fi

# Update system
echo "📦 Updating system packages..."
sudo apt update
sudo apt upgrade -y

# Install system dependencies
echo "🔧 Installing system dependencies..."
sudo apt install -y \
    python3 \
    python3-pip \
    python3-venv \
    i2c-tools \
    git \
    mosquitto-clients

# Enable I2C
echo "🔌 Enabling I2C interface..."
sudo raspi-config nonint do_i2c 0

# Enable Serial
echo "📡 Enabling Serial interface..."
sudo raspi-config nonint do_serial 0

# Create virtual environment (recommended)
echo "🐍 Creating Python virtual environment..."
python3 -m venv venv
source venv/bin/activate

# Install Python dependencies
echo "📋 Installing Python dependencies..."
pip install --upgrade pip
pip install -r requirements.txt

# Set up USB permissions
echo "🔐 Setting up USB permissions..."
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0 2>/dev/null || echo "Note: /dev/ttyUSB0 not found (LiDAR may not be connected)"

# Create startup script
echo "🚀 Creating startup script..."
cat > start_robot.sh << 'EOF'
#!/bin/bash
# BCR Bot Standalone Startup Script

echo "Starting BCR Bot Standalone..."
echo "Press Ctrl+C to stop"

# Check if virtual environment exists
if [ -d "venv" ]; then
    echo "Activating virtual environment..."
    source venv/bin/activate
fi

# Check LiDAR connection
if [ -e "/dev/ttyUSB0" ]; then
    echo "✓ LiDAR detected at /dev/ttyUSB0"
else
    echo "⚠️  LiDAR not detected. Check USB connection."
    echo "   Available USB devices:"
    ls -la /dev/ttyUSB* 2>/dev/null || echo "   No USB devices found"
fi

# Check I2C connection
echo "Checking I2C devices..."
i2cdetect -y 1

# Start the robot
python3 robot_lokal.py
EOF

chmod +x start_robot.sh

# Create systemd service (optional)
echo "🔄 Creating systemd service..."
sudo tee /etc/systemd/system/bcr-bot.service > /dev/null << EOF
[Unit]
Description=BCR Bot Standalone
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=$(pwd)
ExecStart=$(pwd)/start_robot.sh
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# Test I2C connection
echo "🔍 Testing I2C connection..."
if command -v i2cdetect >/dev/null 2>&1; then
    echo "I2C devices found:"
    i2cdetect -y 1
else
    echo "i2cdetect not available, skipping I2C test"
fi

# Test LiDAR connection
echo "🔍 Testing LiDAR connection..."
if [ -e "/dev/ttyUSB0" ]; then
    echo "✓ LiDAR device found at /dev/ttyUSB0"
    ls -la /dev/ttyUSB0
else
    echo "⚠️  LiDAR not found. Common solutions:"
    echo "   1. Check USB cable connection"
    echo "   2. Try different USB port"
    echo "   3. Check with: lsusb"
    echo "   4. Check with: dmesg | tail"
fi

# Create quick test script
echo "🧪 Creating test script..."
cat > test_system.py << 'EOF'
#!/usr/bin/env python3
"""
BCR Bot System Test Script
Tests hardware connections and basic functionality
"""

import sys
import time

def test_imports():
    """Test if all required modules can be imported"""
    print("Testing imports...")
    try:
        import board
        import busio
        import adafruit_pca9685
        print("✓ PCA9685 libraries available")
    except ImportError as e:
        print(f"❌ PCA9685 import error: {e}")
        return False
    
    try:
        from rplidar import RPLidar
        print("✓ RPLidar library available")
    except ImportError as e:
        print(f"❌ RPLidar import error: {e}")
        return False
    
    try:
        import paho.mqtt.client as mqtt
        print("✓ MQTT library available")
    except ImportError as e:
        print(f"❌ MQTT import error: {e}")
        return False
    
    return True

def test_i2c():
    """Test I2C connection"""
    print("\nTesting I2C connection...")
    try:
        import board
        import busio
        i2c = busio.I2C(3, 2)  # SCL=GPIO3, SDA=GPIO2
        print("✓ I2C initialized successfully")
        return True
    except Exception as e:
        print(f"❌ I2C error: {e}")
        return False

def test_lidar():
    """Test LiDAR connection"""
    print("\nTesting LiDAR connection...")
    try:
        from rplidar import RPLidar
        lidar = RPLidar('/dev/ttyUSB0')
        info = lidar.get_info()
        print(f"✓ LiDAR connected: {info}")
        lidar.disconnect()
        return True
    except Exception as e:
        print(f"❌ LiDAR error: {e}")
        return False

def test_mqtt():
    """Test MQTT connection"""
    print("\nTesting MQTT connection...")
    try:
        import paho.mqtt.client as mqtt
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        client.connect("spin5.petra.ac.id", 1883, 60)
        print("✓ MQTT connection successful")
        client.disconnect()
        return True
    except Exception as e:
        print(f"❌ MQTT error: {e}")
        return False

if __name__ == "__main__":
    print("BCR Bot System Test")
    print("=" * 30)
    
    tests = [
        ("Imports", test_imports),
        ("I2C", test_i2c),
        ("LiDAR", test_lidar),
        ("MQTT", test_mqtt)
    ]
    
    results = []
    for name, test_func in tests:
        try:
            result = test_func()
            results.append((name, result))
        except Exception as e:
            print(f"❌ {name} test failed: {e}")
            results.append((name, False))
    
    print("\n" + "=" * 30)
    print("Test Results:")
    all_passed = True
    for name, result in results:
        status = "✓ PASS" if result else "❌ FAIL"
        print(f"{name}: {status}")
        if not result:
            all_passed = False
    
    print("=" * 30)
    if all_passed:
        print("🎉 All tests passed! System is ready.")
    else:
        print("⚠️  Some tests failed. Check hardware connections.")
EOF

chmod +x test_system.py

echo ""
echo "============================================"
echo "✅ Installation completed successfully!"
echo "============================================"
echo ""
echo "Next steps:"
echo "1. Reboot your Raspberry Pi: sudo reboot"
echo "2. Test the system: python3 test_system.py"
echo "3. Run the robot: ./start_robot.sh"
echo ""
echo "Optional:"
echo "- Enable auto-start: sudo systemctl enable bcr-bot"
echo "- Start service: sudo systemctl start bcr-bot"
echo "- Check service: sudo systemctl status bcr-bot"
echo ""
echo "Troubleshooting:"
echo "- Check hardware connections"
echo "- Verify permissions: groups $USER"
echo "- Test LiDAR: ls -la /dev/ttyUSB*"
echo "- Test I2C: i2cdetect -y 1"
echo ""
echo "Happy robot building! 🤖" 
