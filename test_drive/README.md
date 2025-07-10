# Robot Motor Testing Script

A Python script for testing 4-motor robot configurations using PCA9685 PWM controller and Raspberry Pi.

## Overview

This script provides comprehensive testing capabilities for a 4-wheeled robot with individual motor control. It uses the PCA9685 PWM controller to manage motor speed and direction, allowing for precise control of each motor independently.

## Hardware Requirements

- **Raspberry Pi** (any model with GPIO pins)
- **PCA9685 PWM Controller** (16-channel, 12-bit)
- **4 DC Motors** with motor drivers
- **I2C Connection** between Raspberry Pi and PCA9685
- **Power Supply** for motors

### Pin Mapping

#### PWM Pins (PCA9685 channels for speed control):
- Motor 1 (Front Left): Channel 15
- Motor 2 (Front Right): Channel 3
- Motor 3 (Back Right): Channel 11
- Motor 4 (Back Left): Channel 7

#### Direction Pins (PCA9685 channels for direction control):
- Motor 1 (Front Left): Channel 14
- Motor 2 (Front Right): Channel 2
- Motor 3 (Back Right): Channel 10
- Motor 4 (Back Left): Channel 6

### I2C Connection
- **SCL**: GPIO 3
- **SDA**: GPIO 2

## Software Dependencies

Install the required Python libraries:

```bash
pip3 install adafruit-circuitpython-pca9685
pip3 install adafruit-blinka
pip3 install RPi.GPIO
```

### Enable I2C on Raspberry Pi

1. Run `sudo raspi-config`
2. Navigate to **Interfacing Options** → **I2C**
3. Enable I2C
4. Reboot your Raspberry Pi

## Usage

### Basic Usage

```bash
python3 test_motor.py
```

### What the Script Does

1. **Initialization**: Sets up PCA9685 controller with 1500Hz frequency
2. **Individual Motor Testing**: Tests each motor forward and backward
3. **Movement Pattern Testing**: Tests robot movements (forward, backward, left turn, right turn)
4. **Safety Shutdown**: Automatically stops all motors when done or interrupted

### Test Sequence

1. **Individual Motor Tests**:
   - Each motor runs forward for 2 seconds
   - Stops for 1 second
   - Runs backward for 2 seconds
   - Stops for 1 second

2. **Movement Pattern Tests**:
   - Forward movement (3 seconds)
   - Backward movement (3 seconds)
   - Left turn (3 seconds)
   - Right turn (3 seconds)

## Configuration

### Speed Settings
- `fullSpeed`: 65535 (maximum PWM value)
- `halfSpeed`: 32767 (50% speed for testing)
- `speedCoef`: 0.3 (30% of full speed for movement tests)

### Direction Settings
- `arahMAJU`: 0xFFFF (forward direction)
- `arahMUNDUR`: 0x0000 (backward direction)

## Safety Features

- **Emergency Stop**: Press `Ctrl+C` to immediately stop all motors
- **Automatic Shutdown**: Motors automatically stop when script ends
- **Error Handling**: Comprehensive error handling for hardware failures

## Troubleshooting

### Common Issues

1. **Import Errors**:
   - Ensure all dependencies are installed
   - Check that you're running on Raspberry Pi with proper libraries

2. **I2C Not Found**:
   - Verify I2C is enabled in raspi-config
   - Check physical connections to PCA9685
   - Try `i2cdetect -y 1` to scan for devices

3. **Motors Not Responding**:
   - Check motor driver connections
   - Verify power supply to motors
   - Ensure PCA9685 is properly powered

4. **Inconsistent Motor Behavior**:
   - Check motor wiring polarity
   - Verify PWM and direction pin assignments
   - Ensure adequate power supply

### Debug Mode

The script includes detailed debug output showing:
- Motor initialization status
- Speed and direction changes
- Pin assignments
- Error messages

## Motor Layout

```
Front
[2] --- [1]
 |       |
 |       |
[4] --- [3]
Back
```

- Motor 1: Front Left
- Motor 2: Front Right  
- Motor 3: Back Right
- Motor 4: Back Left

## Movement Logic

### Forward Movement
- Motors 1 & 3: Forward direction
- Motors 2 & 4: Backward direction

### Backward Movement
- Motors 1 & 3: Backward direction
- Motors 2 & 4: Forward direction

### Left Turn
- Motors 1 & 3: Backward direction
- Motors 2 & 4: Forward direction

### Right Turn
- Motors 1 & 3: Forward direction
- Motors 2 & 4: Backward direction

## Important Notes

⚠️ **Safety Warning**: Always ensure motors are securely mounted and area is clear before testing.

⚠️ **Power Supply**: Ensure adequate power supply for all motors to prevent brownouts.

⚠️ **Wiring**: Double-check all connections before powering on.

## License

This script is provided as-is for educational and testing purposes.

## Contributing

Feel free to modify the script for your specific robot configuration. Common modifications include:
- Adjusting pin mappings
- Changing speed coefficients
- Adding new movement patterns
- Implementing sensor integration 
