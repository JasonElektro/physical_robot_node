#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import json
import time
from smbus2 import SMBus
from rplidar import RPLidar

# Constants untuk motor control
PCA_FREQ = 1500  # Updated to match your config
fullSpeed = 65535  # Max PWM value
halfSpeed = 32767
noSpeed = 0
speedCoef = 0.5  # Increased to match Gazebo speed

# Safety thresholds
MAX_CURRENT = 10.0  # Maximum current in amps
MIN_BATTERY = 10.0  # Minimum battery voltage
MAX_TEMP = 60.0    # Maximum motor temperature in Celsius

# Command timeout
COMMAND_TIMEOUT = 0.5  # 500ms timeout for commands

# PCA9685 registers
MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06
LED0_ON_H = 0x07
LED0_OFF_L = 0x08
LED0_OFF_H = 0x09

# Pin mapping untuk PCA9685 - Updated to match your config
pinPWM = {
    1: 15,  # Motor 1 PWM pin
    2: 3,   # Motor 2 PWM pin
    3: 11,  # Motor 3 PWM pin
    4: 7    # Motor 4 PWM pin
}
pinDIR = {
    1: 14,  # Motor 1 direction pin
    2: 2,   # Motor 2 direction pin
    3: 10,  # Motor 3 direction pin
    4: 6    # Motor 4 direction pin
}

# Arah putaran motor
arahMAJU = 1      # HIGH/ON
arahMUNDUR = 0    # LOW/OFF

class PCA9685:
    def __init__(self, bus, address=0x40):
        self.bus = bus
        self.address = address
        self.setup()

    def setup(self):
        """Initialize PCA9685"""
        try:
            # Set mode1 register to enable auto-increment
            self.bus.write_byte_data(self.address, MODE1, 0x20)
            print("[DEBUG] PCA9685 MODE1 set to 0x20")
            
            # Set mode2 register
            self.bus.write_byte_data(self.address, MODE2, 0x04)
            print("[DEBUG] PCA9685 MODE2 set to 0x04")
            
            # Set prescale for 50Hz
            prescale = int(25000000 / (4096 * PCA_FREQ) - 1)
            print(f"[DEBUG] PCA9685 prescale set to {prescale}")
            
            self.bus.write_byte_data(self.address, MODE1, 0x10)  # Sleep
            self.bus.write_byte_data(self.address, PRESCALE, prescale)
            self.bus.write_byte_data(self.address, MODE1, 0x20)  # Wake up
            
            # Verify initialization
            mode1 = self.bus.read_byte_data(self.address, MODE1)
            mode2 = self.bus.read_byte_data(self.address, MODE2)
            print(f"[DEBUG] PCA9685 MODE1 readback: 0x{mode1:02x}")
            print(f"[DEBUG] PCA9685 MODE2 readback: 0x{mode2:02x}")
            
            # Test all PWM channels
            print("[DEBUG] Testing all PWM channels...")
            for channel in range(16):
                self.set_pwm(channel, 0, 4095)  # Set to full on
                time.sleep(0.1)
                self.set_pwm(channel, 0, 0)     # Set to full off
                print(f"[DEBUG] Tested channel {channel}")
            
        except Exception as e:
            print(f"[ERROR] PCA9685 setup failed: {str(e)}")
            raise

    def set_pwm(self, channel, on, off):
        """Set PWM value for a channel"""
        try:
            self.bus.write_byte_data(self.address, LED0_ON_L + 4*channel, on & 0xFF)
            self.bus.write_byte_data(self.address, LED0_ON_H + 4*channel, on >> 8)
            self.bus.write_byte_data(self.address, LED0_OFF_L + 4*channel, off & 0xFF)
            self.bus.write_byte_data(self.address, LED0_OFF_H + 4*channel, off >> 8)
            print(f"[DEBUG] Set PWM channel {channel} to on={on}, off={off}")
        except Exception as e:
            print(f"[ERROR] Failed to set PWM channel {channel}: {str(e)}")

    def set_direction(self, channel, direction):
        """Set direction pin to HIGH or LOW"""
        try:
            if direction == 1:  # HIGH
                self.set_pwm(channel, 0, 4095)  # Full ON
            else:  # LOW
                self.set_pwm(channel, 0, 0)     # Full OFF
            print(f"[DEBUG] Set direction channel {channel} to {'HIGH' if direction == 1 else 'LOW'}")
        except Exception as e:
            print(f"[ERROR] Failed to set direction channel {channel}: {str(e)}")

class PhysicalRobotController:
    def __init__(self):
        # Setup PCA9685
        try:
            self.bus = SMBus(1)  # Use I2C bus 1
            self.pca = PCA9685(self.bus)
            print("[STATUS] Connected to PCA9685")
            
            # Initialize semua motor ke stop
            for roda in range(1, 5):
                self.pca.set_pwm(pinPWM[roda], 0, 0)
                self.pca.set_direction(pinDIR[roda], 0)
                
        except Exception as e:
            print(f"[ERROR] PCA9685 init failed: {str(e)}")
            self.pca = None

        # Setup RPLidar with proper configuration
        try:
            self.lidar = RPLidar('/dev/ttyUSB0', baudrate=115200, timeout=1)
            self.lidar.motor_speed = 200  # Set motor speed
            self.lidar.start_motor()
            time.sleep(1)  # Give time for motor to start
            print("[STATUS] Connected to RPLidar")
        except Exception as e:
            print(f"[ERROR] RPLidar init failed: {str(e)}")
            self.lidar = None

        # Setup MQTT
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        try:
            self.mqtt_client.connect("codex.petra.ac.id", 1883, 60)
            self.mqtt_client.loop_start()
            print("[STATUS] Connected to MQTT broker")
        except Exception as e:
            print(f"[ERROR] MQTT connection failed: {str(e)}")

        # Initialize command tracking
        self.last_command_time = time.time()
        self.last_command_source = None
        self.command_timeout = COMMAND_TIMEOUT

    def check_safety(self):
        """Check various safety conditions"""
        try:
            # Check for command timeout
            if time.time() - self.last_command_time > self.command_timeout:
                print("[WARNING] Command timeout - stopping motors")
                self.stop_all_motors()
                return False

            # Check battery voltage (if implemented)
            # if self.get_battery_voltage() < MIN_BATTERY:
            #     print("[WARNING] Low battery - stopping motors")
            #     self.stop_all_motors()
            #     return False

            # Check motor temperature (if implemented)
            # if self.get_motor_temperature() > MAX_TEMP:
            #     print("[WARNING] Motor temperature too high - stopping motors")
            #     self.stop_all_motors()
            #     return False

            return True
        except Exception as e:
            print(f"[ERROR] Safety check failed: {str(e)}")
            return False

    def ramp_speed(self, target_speed):
        """Gradually change speed to avoid sudden movements"""
        if target_speed > self.current_speed:
            self.current_speed = min(self.current_speed + self.speed_ramp_rate, target_speed)
        elif target_speed < self.current_speed:
            self.current_speed = max(self.current_speed - self.speed_ramp_rate, target_speed)
        return self.current_speed

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("[STATUS] MQTT Connected")
            # Subscribe to all command topics
            client.subscribe("robot/physical/cmd_vel")  # Manual control
            client.subscribe("robot/primary/cmd_vel")   # Primary mode
            client.subscribe("robot/secondary/cmd_vel") # Secondary mode
            client.subscribe("robot/rmode/cmd_vel")     # Rmode
            client.subscribe("robot/physical/status")   # Status topic
        else:
            print(f"[ERROR] MQTT Connection failed with code {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            cmd_data = json.loads(msg.payload.decode())
            print(f"[DEBUG] Received command from {msg.topic}: {cmd_data}")
            
            # Update command source and time
            self.last_command_source = msg.topic
            self.last_command_time = time.time()
            
            # Handle the command
            self.handle_movement_command(cmd_data)
            
        except Exception as e:
            print(f"[ERROR] Failed to process command: {str(e)}")

    def handle_movement_command(self, cmd_data):
        """Convert cmd_vel to motor commands"""
        if self.pca is None:
            print("[ERROR] PCA9685 not initialized")
            return

        try:
            # Extract linear and angular velocities
            linear_x = float(cmd_data.get('linear', {}).get('x', 0))
            angular_z = float(cmd_data.get('angular', {}).get('z', 0))
            
            print(f"[DEBUG] Processing command - linear_x: {linear_x}, angular_z: {angular_z}")

            # Stop semua motor dulu
            self.stop_all_motors()

            # Tentukan gerakan berdasarkan cmd_vel
            if abs(linear_x) > abs(angular_z):
                # Gerakan maju/mundur dominan
                if linear_x > 0:
                    print("[DEBUG] Command: Moving forward")
                    self.maju(min(abs(linear_x), 1.0))
                else:
                    print("[DEBUG] Command: Moving backward")
                    self.mundur(min(abs(linear_x), 1.0))
            else:
                # Gerakan putar dominan
                if angular_z > 0:
                    print("[DEBUG] Command: Turning left")
                    self.putar_kiri(min(abs(angular_z), 1.0))
                else:
                    print("[DEBUG] Command: Turning right")
                    self.putar_kanan(min(abs(angular_z), 1.0))

        except Exception as e:
            print(f"[ERROR] Error in handle_movement_command: {str(e)}")
            print(f"[DEBUG] Command data received: {cmd_data}")
            self.stop_all_motors()

    def stop_all_motors(self):
        """Stop semua motor"""
        try:
            print("[DEBUG] Stopping all motors")
            for roda in range(1, 5):
                self.pca.set_pwm(pinPWM[roda], 0, 0)
                print(f"[DEBUG] Motor {roda} PWM set to 0")
            self.current_speed = 0
            print("[DEBUG] All motors stopped")
        except Exception as e:
            print(f"[ERROR] Failed to stop motors: {str(e)}")

    def set_motor_speed(self, speed_factor):
        """Set kecepatan motor (0.0 - 1.0)"""
        try:
            speed = int(speed_factor * fullSpeed * speedCoef)  # Apply speed coefficient
            print(f"[DEBUG] Setting motor speed to {speed} (factor: {speed_factor}, fullSpeed: {fullSpeed}, speedCoef: {speedCoef})")
            for roda in range(1, 5):
                self.pca.set_pwm(pinPWM[roda], 0, speed)
                print(f"[DEBUG] Motor {roda} PWM set to {speed}")
        except Exception as e:
            print(f"[ERROR] Failed to set motor speed: {str(e)}")

    def mundur(self, speed_factor=1.0):
        """Move backward"""
        try:
            print("[DEBUG] Setting direction for backward movement")
            # All wheels backward
            print("[DEBUG] Motor 1: MUNDUR")
            self.pca.set_direction(pinDIR[1], arahMUNDUR)  # LOW
            print("[DEBUG] Motor 2: MUNDUR")
            self.pca.set_direction(pinDIR[2], arahMUNDUR)  # LOW
            print("[DEBUG] Motor 3: MUNDUR")
            self.pca.set_direction(pinDIR[3], arahMUNDUR)  # LOW
            print("[DEBUG] Motor 4: MUNDUR")
            self.pca.set_direction(pinDIR[4], arahMUNDUR)  # LOW
            
            # Set speed for all motors
            speed = int(fullSpeed * speed_factor * speedCoef)
            print(f"[DEBUG] Setting motor speed to {speed} (factor: {speed_factor}, fullSpeed: {fullSpeed}, speedCoef: {speedCoef})")
            
            self.pca.set_pwm(pinPWM[1], 0, speed)
            print(f"[DEBUG] Motor 1 PWM set to {speed}")
            self.pca.set_pwm(pinPWM[2], 0, speed)
            print(f"[DEBUG] Motor 2 PWM set to {speed}")
            self.pca.set_pwm(pinPWM[3], 0, speed)
            print(f"[DEBUG] Motor 3 PWM set to {speed}")
            self.pca.set_pwm(pinPWM[4], 0, speed)
            print(f"[DEBUG] Motor 4 PWM set to {speed}")
            
            print("[DEBUG] Backward movement command executed")
            
        except Exception as e:
            print(f"[ERROR] Error in mundur: {str(e)}")
            self.stop_all_motors()

    def maju(self, speed_factor=1.0):
        """Move forward"""
        try:
            print("[DEBUG] Setting direction for forward movement")
            # All wheels forward
            print("[DEBUG] Motor 1: MAJU")
            self.pca.set_direction(pinDIR[1], arahMAJU)  # HIGH
            print("[DEBUG] Motor 2: MAJU")
            self.pca.set_direction(pinDIR[2], arahMAJU)  # HIGH
            print("[DEBUG] Motor 3: MAJU")
            self.pca.set_direction(pinDIR[3], arahMAJU)  # HIGH
            print("[DEBUG] Motor 4: MAJU")
            self.pca.set_direction(pinDIR[4], arahMAJU)  # HIGH
            
            # Set speed for all motors
            speed = int(fullSpeed * speed_factor * speedCoef)
            print(f"[DEBUG] Setting motor speed to {speed} (factor: {speed_factor}, fullSpeed: {fullSpeed}, speedCoef: {speedCoef})")
            
            self.pca.set_pwm(pinPWM[1], 0, speed)
            print(f"[DEBUG] Motor 1 PWM set to {speed}")
            self.pca.set_pwm(pinPWM[2], 0, speed)
            print(f"[DEBUG] Motor 2 PWM set to {speed}")
            self.pca.set_pwm(pinPWM[3], 0, speed)
            print(f"[DEBUG] Motor 3 PWM set to {speed}")
            self.pca.set_pwm(pinPWM[4], 0, speed)
            print(f"[DEBUG] Motor 4 PWM set to {speed}")
            
            print("[DEBUG] Forward movement command executed")
            
        except Exception as e:
            print(f"[ERROR] Error in maju: {str(e)}")
            self.stop_all_motors()

    def putar_kiri(self, speed_factor=1.0):
        """Turn left"""
        try:
            print("[DEBUG] Setting direction for left turn")
            # Wheels 2,3 backward, Wheels 1,4 forward
            print("[DEBUG] Motor 1: MAJU")
            self.pca.set_direction(pinDIR[1], arahMAJU)  # HIGH
            print("[DEBUG] Motor 2: MUNDUR")
            self.pca.set_direction(pinDIR[2], arahMUNDUR)  # LOW
            print("[DEBUG] Motor 3: MUNDUR")
            self.pca.set_direction(pinDIR[3], arahMUNDUR)  # LOW
            print("[DEBUG] Motor 4: MAJU")
            self.pca.set_direction(pinDIR[4], arahMAJU)  # HIGH
            
            # Set speed for all motors
            speed = int(fullSpeed * speed_factor * speedCoef)
            print(f"[DEBUG] Setting motor speed to {speed} (factor: {speed_factor}, fullSpeed: {fullSpeed}, speedCoef: {speedCoef})")
            
            self.pca.set_pwm(pinPWM[1], 0, speed)
            print(f"[DEBUG] Motor 1 PWM set to {speed}")
            self.pca.set_pwm(pinPWM[2], 0, speed)
            print(f"[DEBUG] Motor 2 PWM set to {speed}")
            self.pca.set_pwm(pinPWM[3], 0, speed)
            print(f"[DEBUG] Motor 3 PWM set to {speed}")
            self.pca.set_pwm(pinPWM[4], 0, speed)
            print(f"[DEBUG] Motor 4 PWM set to {speed}")
            
            print("[DEBUG] Left turn command executed")
            
        except Exception as e:
            print(f"[ERROR] Error in putar_kiri: {str(e)}")
            self.stop_all_motors()

    def putar_kanan(self, speed_factor=1.0):
        """Turn right"""
        try:
            print("[DEBUG] Setting direction for right turn")
            # Wheels 2,3 forward, Wheels 1,4 backward
            print("[DEBUG] Motor 1: MUNDUR")
            self.pca.set_direction(pinDIR[1], arahMUNDUR)  # LOW
            print("[DEBUG] Motor 2: MAJU")
            self.pca.set_direction(pinDIR[2], arahMAJU)  # HIGH
            print("[DEBUG] Motor 3: MAJU")
            self.pca.set_direction(pinDIR[3], arahMAJU)  # HIGH
            print("[DEBUG] Motor 4: MUNDUR")
            self.pca.set_direction(pinDIR[4], arahMUNDUR)  # LOW
            
            # Set speed for all motors
            speed = int(fullSpeed * speed_factor * speedCoef)
            print(f"[DEBUG] Setting motor speed to {speed} (factor: {speed_factor}, fullSpeed: {fullSpeed}, speedCoef: {speedCoef})")
            
            self.pca.set_pwm(pinPWM[1], 0, speed)
            print(f"[DEBUG] Motor 1 PWM set to {speed}")
            self.pca.set_pwm(pinPWM[2], 0, speed)
            print(f"[DEBUG] Motor 2 PWM set to {speed}")
            self.pca.set_pwm(pinPWM[3], 0, speed)
            print(f"[DEBUG] Motor 3 PWM set to {speed}")
            self.pca.set_pwm(pinPWM[4], 0, speed)
            print(f"[DEBUG] Motor 4 PWM set to {speed}")
            
            print("[DEBUG] Right turn command executed")
            
        except Exception as e:
            print(f"[ERROR] Error in putar_kanan: {str(e)}")
            self.stop_all_motors()

    def publish_lidar_data(self):
        """Baca dan publish data LiDAR"""
        if self.lidar is None:
            return

        try:
            for scan in self.lidar.iter_scans(max_buf_meas=500):
                scan_data = {
                    'ranges': [],
                    'angles': []
                }
                for _, angle, distance in scan:
                    scan_data['ranges'].append(distance/1000.0)  # Convert to meters
                    scan_data['angles'].append(angle)

                self.mqtt_client.publish("robot/physical/scan", 
                                       json.dumps(scan_data))
                break  # Hanya publish 1 scan
        except Exception as e:
            print(f"[ERROR] Failed to read LiDAR: {str(e)}")
            # Try to reset LiDAR connection
            try:
                self.lidar.stop_motor()
                time.sleep(0.5)
                self.lidar.start_motor()
                time.sleep(0.5)
            except:
                pass

    def test_motors(self):
        """Test motor movement in sequence"""
        print("[TEST] Starting motor test sequence")
        
        # First test each motor individually
        print("\n[TEST] Testing individual motors...")
        for motor in range(1, 5):
            print(f"\n[TEST] Testing Motor {motor}")
            print(f"[TEST] Setting PWM pin {pinPWM[motor]} and DIR pin {pinDIR[motor]}")
            
            # Test forward
            print(f"[TEST] Motor {motor} - Forward")
            self.pca.set_direction(pinDIR[motor], arahMAJU)
            self.pca.set_pwm(pinPWM[motor], 0, 9830)
            time.sleep(2)
            
            # Stop
            print(f"[TEST] Motor {motor} - Stop")
            self.pca.set_pwm(pinPWM[motor], 0, 0)
            time.sleep(1)
            
            # Test backward
            print(f"[TEST] Motor {motor} - Backward")
            self.pca.set_direction(pinDIR[motor], arahMUNDUR)
            self.pca.set_pwm(pinPWM[motor], 0, 9830)
            time.sleep(2)
            
            # Stop
            print(f"[TEST] Motor {motor} - Stop")
            self.pca.set_pwm(pinPWM[motor], 0, 0)
            time.sleep(1)
        
        print("\n[TEST] Testing all motors together...")
        
        # Test forward
        print("[TEST] Testing forward movement")
        test_cmd = {
            'linear': {'x': 0.5, 'y': 0, 'z': 0},
            'angular': {'x': 0, 'y': 0, 'z': 0}
        }
        self.handle_movement_command(test_cmd)
        time.sleep(2)
        
        # Test backward
        print("[TEST] Testing backward movement")
        test_cmd = {
            'linear': {'x': -0.5, 'y': 0, 'z': 0},
            'angular': {'x': 0, 'y': 0, 'z': 0}
        }
        self.handle_movement_command(test_cmd)
        time.sleep(2)
        
        # Test left turn
        print("[TEST] Testing left turn")
        test_cmd = {
            'linear': {'x': 0, 'y': 0, 'z': 0},
            'angular': {'x': 0, 'y': 0, 'z': 0.5}
        }
        self.handle_movement_command(test_cmd)
        time.sleep(2)
        
        # Test right turn
        print("[TEST] Testing right turn")
        test_cmd = {
            'linear': {'x': 0, 'y': 0, 'z': 0},
            'angular': {'x': 0, 'y': 0, 'z': -0.5}
        }
        self.handle_movement_command(test_cmd)
        time.sleep(2)
        
        # Stop
        print("[TEST] Stopping motors")
        test_cmd = {
            'linear': {'x': 0, 'y': 0, 'z': 0},
            'angular': {'x': 0, 'y': 0, 'z': 0}
        }
        self.handle_movement_command(test_cmd)
        print("[TEST] Motor test sequence completed")

    def run(self):
        print("[STATUS] Robot controller running...")
        try:
            # Run motor test sequence
            self.test_motors()
            
            while True:
                self.publish_lidar_data()
                time.sleep(0.1)  # 10Hz update rate
        except KeyboardInterrupt:
            print("\n[STATUS] Shutting down...")
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup saat shutdown"""
        # Stop semua motor
        if self.pca is not None:
            self.stop_all_motors()

        # Stop LiDAR
        if self.lidar is not None:
            try:
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except:
                pass

        # Disconnect MQTT
        if self.mqtt_client is not None:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

if __name__ == "__main__":
    controller = PhysicalRobotController()
    controller.run()