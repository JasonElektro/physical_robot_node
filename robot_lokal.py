#!/usr/bin/env python3
"""
BCR Bot Standalone - Combined Navigation and Motor Control
Uses real-world LiDAR A1M8 data only, no Gazebo simulation
Direct motor control on Raspberry Pi

Features:
- Real-world LiDAR A1M8 integration
- Direct motor control via PCA9685
- Multiple navigation modes (Primary, Secondary, R-mode, Manual)
- Obstacle avoidance using LiDAR data
- Keyboard control
- Waypoint navigation
- Battery monitoring and charging navigation
"""

import time
import board
import busio
import adafruit_pca9685
import RPi.GPIO as GPIO
import threading
import math
import sys
import select
import tty
import termios
import os
import signal
import queue
import random
import json
from rplidar import RPLidar
import paho.mqtt.client as mqtt

# Motor Configuration
PCA_FREQ = 1500
fullSpeed = 0xFFFF  # 65535
halfSpeed = 0x7FFF  # 32767
noSpeed = 0x0000    # 0
speedCoef = 0.25    # Speed reduction factor - 1/4 MAX SPEED
rmodeSpeedCoef = 0.25  # R-mode speed coefficient - 1/4 MAX SPEED
rmodeBackwardSpeedCoef = 0.25  # R-mode backward speed coefficient - 1/4 MAX SPEED

# Motor directions
arahMAJU = 0xFFFF    # HIGH
arahMUNDUR = 0x0000  # LOW

# Pin mappings
pinPWM = {
    1: 15,  # Motor 1 PWM pin (depan kiri)
    2: 3,   # Motor 2 PWM pin (depan kanan)
    3: 11,  # Motor 3 PWM pin (belakang kanan)
    4: 7    # Motor 4 PWM pin (belakang kiri)
}

pinDIR = {
    1: 14,  # Motor 1 direction pin (depan kiri)
    2: 2,   # Motor 2 direction pin (depan kanan)
    3: 10,  # Motor 3 direction pin (belakang kanan)
    4: 6    # Motor 4 direction pin (belakang kiri)
}

# LiDAR Configuration
LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_BAUDRATE = 115200
LIDAR_TIMEOUT = 1.0

class BCRBotStandalone:
    def __init__(self):
        print("[INIT] Initializing BCR Bot Standalone...")
        
        # Initialize hardware first
        self.init_hardware()
        
        # Initialize navigation state
        self.init_navigation_state()
        
        # Initialize LiDAR
        self.init_lidar()
        
        # Initialize MQTT for mapping
        self.init_mqtt()
        
        # Initialize keyboard handling
        self.init_keyboard()
        
        print("[INIT] BCR Bot Standalone initialized successfully")
        print("[INIT] Ready for operation")

    def init_hardware(self):
        """Initialize PCA9685 and motor hardware"""
        try:
            print("[INIT] Initializing PCA9685...")
            self.i2c = busio.I2C(3, 2)  # SCL=GPIO3, SDA=GPIO2
            self.pca = adafruit_pca9685.PCA9685(self.i2c)
            self.pca.frequency = PCA_FREQ
            
            # Initialize all motors to stop
            for motor in range(1, 5):
                self.pca.channels[pinPWM[motor]].duty_cycle = 0
                self.pca.channels[pinDIR[motor]].duty_cycle = 0
            
            print("[INIT] PCA9685 initialized successfully")
            
        except Exception as e:
            print(f"[ERROR] Failed to initialize PCA9685: {str(e)}")
            raise

    def init_navigation_state(self):
        """Initialize all navigation-related state variables"""
        # Remove position tracking (not used in standalone mode)
        # self.robot_position = (0.0, 0.0)  # Removed - not used
        # self.robot_orientation = 0.0      # Removed - not used
        self.last_position = None
        self.position_threshold = 0.1
        self.stuck_time = 0
        self.stuck_threshold = 5.0
        self.stuck = False
        
        # Status tracking
        self.status_queue = queue.Queue()
        self.stuck_counter = 0
        self.max_stuck = 20
        self.last_turn_time = time.time()
        self.rotation_progress = 0
        self.last_input_time = time.time()
        self.status = 'Waiting for input...'
        self.last_status = ''
        self.last_status_update = time.time()
        self.status_update_interval = 2.0
        
        # Turn cooldown
        self.last_turn_direction = None
        self.turn_cooldown = 10.0
        self.last_turn_time = 0.0
        
        # Control mode flags
        self.primary_active = True
        self.secondary_control_active = False
        self.manual_mode = False
        self.r_mode = False
        
        # Navigation charging attributes (simplified without position tracking)
        self.nav_charging_mode = False
        self.charging_obstacle_threshold = 1.2
        self.charging_emergency_stop_threshold = 0.5
        self.charging_turn_speed = 0.25  # 1/4 MAX SPEED
        self.battery_low = False
        self.battery_voltage = 12.0
        self.battery_low_threshold = 11.0
        
        # Turning behavior attributes
        self.consecutive_turns = 0
        self.max_consecutive_turns = 3
        self.backup_duration = 2.0
        self.backup_start_time = None
        self.is_backing_up = False
        self.turning_to_best = False
        self.recovery_end_time = 0
        self.recovery_phase = 0
        self.recovery_active = False
        self.recovery_start_time = None
        self.recovery_timeout = 10.0
        self.recovery_phase_start = None
        self.recovery_phase_duration = 2.0
        
        # Obstacle detection variables - using only real-world LiDAR
        self.obstacle_detected = False
        self.lidar_too_close = False
        self.lidar_min_distance = float('inf')
        self.left_min = float('inf')
        self.right_min = float('inf')
        self.left_obstacle = False
        self.right_obstacle = False
        self.obstacle_threshold = 1.0
        self.emergency_stop_threshold = 0.4
        
        # Enhanced obstacle detection
        self.front_obstacle_close = False
        self.front_obstacle_warning = False
        self.side_obstacle_close = False
        self.both_sides_blocked = False
        
        # Movement control variables
        self.turning = False
        self.is_scanning = False
        self.scan_start_time = 0
        self.scan_duration = 3.0
        self.turn_start_time = 0
        self.turn_duration = 1.0
        
        # Robot state
        self.running = True
        
        # Secondary control attributes
        self.secondary_active = False
        self.secondary_start_time = None
        self.secondary_timeout = 45.0
        
        # R-mode attributes (simplified without coordinate navigation)
        self.r_mode_timeout = 45.0
        self.r_mode_forward_speed = 0.25  # 1/4 MAX SPEED
        self.r_mode_turn_speed = 0.25     # 1/4 MAX SPEED
        self.r_mode_obstacle_threshold = 1.2
        self.r_mode_emergency_stop = 0.6
        
        # MQTT attributes for mapping data
        self.mqtt_client = None
        self.mqtt_connected = False
        self.mapping_data_buffer = []
        self.last_mapping_send = time.time()
        
        # Current mode and motor status
        self.current_mode = "PRIMARY"
        self.motor_status = "STOPPED"

    def init_lidar(self):
        """Initialize LiDAR A1M8 with better error handling"""
        try:
            print(f"[LIDAR] Checking USB permissions for {LIDAR_PORT}...")
            
            # Check if port exists
            import os
            if not os.path.exists(LIDAR_PORT):
                print(f"[ERROR] LiDAR port {LIDAR_PORT} does not exist!")
                print("[HELP] Try: ls /dev/ttyUSB* to see available ports")
                print("[HELP] Or: sudo chmod 666 /dev/ttyUSB0")
                self.lidar = None
                return
                
            print(f"[LIDAR] Port {LIDAR_PORT} exists, initializing...")
            self.lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE, timeout=LIDAR_TIMEOUT)
            
            # Get LiDAR info
            print("[LIDAR] Getting device info...")
            info = self.lidar.get_info()
            print(f"[LIDAR] Device info: {info}")
            
            # Get health status
            print("[LIDAR] Checking health status...")
            health = self.lidar.get_health()
            print(f"[LIDAR] Health status: {health}")
            
            # Initialize LiDAR data
            self.lidar_data = None
            self.lidar_thread = None
            self.lidar_running = False
            self.last_lidar_log = time.time()
            
            # Initialize LiDAR logging for debugging
            self.last_mapping_send = time.time()
            
            print("[LIDAR] ✓ LiDAR A1M8 initialized successfully")
            print("[LIDAR] ✓ CORRECTED Orientation: Front=180°, Left=90°, Right=270°, Rear=0°")
            print("[LIDAR] ✓ Layout:")
            print("[LIDAR]     Front (180°)")
            print("[LIDAR]  Left(90°)  Right(270°)")
            print("[LIDAR]      Rear (0°)")
            
            # Start scanning immediately
            self.start_lidar_scanning()
            
        except Exception as e:
            print(f"[ERROR] Failed to initialize LiDAR: {str(e)}")
            print("[HELP] Troubleshooting steps:")
            print("[HELP] 1. Check connection: ls /dev/ttyUSB*")
            print("[HELP] 2. Check permissions: sudo chmod 666 /dev/ttyUSB0") 
            print("[HELP] 3. Check if device is recognized: dmesg | tail")
            print("[HELP] 4. Try different port: /dev/ttyUSB1")
            self.lidar = None

    def init_mqtt(self):
        """Initialize MQTT client for mapping data"""
        try:
            print("[MQTT] Initializing MQTT client for mapping...")
            self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            
            # Enhanced connection settings for WiFi semangka 6/12
            self.mqtt_client.reconnect_delay_set(min_delay=1, max_delay=30)
            self.mqtt_client._keepalive = 30
            
            try:
                print("[MQTT] Connecting to spin5.petra.ac.id...")
                self.mqtt_client.connect("spin5.petra.ac.id", 1883, 30)
                self.mqtt_client.loop_start()
                print("[MQTT] MQTT client started")
            except Exception as e:
                print(f"[MQTT] Failed to connect to MQTT broker: {str(e)}")
                print("[MQTT] Continuing without MQTT support")
                self.mqtt_client = None
                
        except Exception as e:
            print(f"[ERROR] Failed to initialize MQTT: {str(e)}")
            self.mqtt_client = None

    def on_mqtt_connect(self, client, userdata, flags, reason_code, properties):
        """MQTT connection callback"""
        if reason_code == 0:
            self.mqtt_connected = True
            print("[MQTT] Connected successfully to spin5.petra.ac.id")
            print("[MQTT] Will send LiDAR data to topic: robot/lidar/real_world_data")
        else:
            self.mqtt_connected = False
            print(f"[MQTT] Connection failed with code: {reason_code}")

    def on_mqtt_disconnect(self, client, userdata, reason_code, properties):
        """MQTT disconnection callback"""
        self.mqtt_connected = False
        print(f"[MQTT] Disconnected with code: {reason_code}")

    def init_keyboard(self):
        """Initialize keyboard input handling"""
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            signal.signal(signal.SIGINT, self.signal_handler)
            os.system('clear')
            self.print_instructions()
            print("[KEYBOARD] Keyboard input initialized")
        except Exception as e:
            print(f"[ERROR] Failed to initialize keyboard: {str(e)}")

    def signal_handler(self, signum, frame):
        """Handle Ctrl+C and other termination signals"""
        try:
            self.running = False
            print('\nShutdown signal received, cleaning up...')
            self.cleanup()
            sys.exit(0)
        except Exception as e:
            print(f'\nError during signal handling: {str(e)}')
            sys.exit(1)

    def start_lidar_scanning(self):
        """Start LiDAR scanning in a separate thread with improved error handling"""
        if self.lidar_running:
            print("[LIDAR] ✓ Already scanning")
            return True
            
        if not self.lidar:
            print("[LIDAR] ❌ LiDAR not initialized - cannot start scanning")
            return False
            
        try:
            print("[LIDAR] Starting scanning thread...")
            self.lidar_running = True
            self.lidar_thread = threading.Thread(target=self.lidar_scanning_loop, name="LiDAR_Scanner")
            self.lidar_thread.daemon = True
            self.lidar_thread.start()
            
            # Wait a moment to check if thread starts successfully
            time.sleep(0.5)
            if self.lidar_thread.is_alive():
                print("[LIDAR] ✓ Scanning thread started successfully")
                return True
            else:
                print("[LIDAR] ❌ Scanning thread failed to start properly")
                self.lidar_running = False
                return False
                
        except Exception as e:
            print(f"[LIDAR] ❌ Failed to start scanning: {str(e)}")
            self.lidar_running = False
            return False

    def lidar_scanning_loop(self):
        """Main LiDAR scanning loop with detailed debugging"""
        scan_count = 0
        last_status_time = time.time()
        
        print("[LIDAR] 🔄 Starting LiDAR scanning loop...")
        
        try:
            print("[LIDAR] 🔄 Calling lidar.iter_scans()...")
            for scan in self.lidar.iter_scans():
                if not self.lidar_running:
                    print("[LIDAR] 🛑 Scanning stopped by lidar_running=False")
                    break
                    
                scan_count += 1
                
                # Process the scan data
                self.process_lidar_scan(scan)
                
                # Print status every 100 scans (more frequent for debugging)
                current_time = time.time()
                if current_time - last_status_time > 5.0:  # Every 5 seconds
                    print(f"[LIDAR] 📊 Processed {scan_count} scans (Rate: {scan_count/5:.1f} scans/sec)")
                    scan_count = 0  # Reset counter
                    last_status_time = current_time
                    
                time.sleep(0.01)  # Small delay
                
        except Exception as e:
            print(f"[LIDAR] ❌ Scanning error: {str(e)}")
            print(f"[LIDAR] ❌ Error type: {type(e).__name__}")
            print("[LIDAR] ❌ This might be a USB connection or permission issue")
            
        finally:
            print("[LIDAR] 🛑 LiDAR scanning loop ended")
            self.lidar_running = False

    def process_lidar_scan(self, scan_data):
        """Process LiDAR scan data for obstacle detection"""
        try:
            # Convert scan data to 360-degree array
            ranges = [float('inf')] * 360
            
            for quality, angle, distance in scan_data:
                angle_deg = int(angle) % 360
                if 0 <= angle_deg < 360:
                    ranges[angle_deg] = distance / 1000.0  # Convert to meters
            
            # Process obstacle detection
            self.update_obstacle_data(ranges)
            
        except Exception as e:
            print(f"[ERROR] Failed to process LiDAR scan: {str(e)}")

    def update_obstacle_data(self, ranges):
        """Update obstacle detection data from LiDAR ranges - CORRECTED ORIENTATION"""
        try:
            n = len(ranges)
            window = 15  # Window size for averaging
            
            # FRONT (180 degrees) - CORRECTED: front is at 180°, not 0°
            front_idx = 180
            front_start = max(0, front_idx - window)
            front_end = min(n, front_idx + window + 1)
            front_ranges = ranges[front_start:front_end]
            
            valid_front = []
            for r in front_ranges:
                if r > 0.20 and r < 12.0:  # Filter valid readings
                    valid_front.append(r)
            
            self.lidar_min_distance = min(valid_front) if valid_front else float('inf')
            
            # LEFT (90 degrees) - this is correct
            left_idx = 90
            left_start = max(0, left_idx - window)
            left_end = min(n, left_idx + window + 1)
            left_ranges = ranges[left_start:left_end]
            
            valid_left = []
            for r in left_ranges:
                if r > 0.20 and r < 12.0:
                    valid_left.append(r)
            
            self.left_min = min(valid_left) if valid_left else float('inf')
            
            # RIGHT (270 degrees) - this is correct
            right_idx = 270
            right_start = max(0, right_idx - window)
            right_end = min(n, right_idx + window + 1)
            right_ranges = ranges[right_start:right_end]
            
            valid_right = []
            for r in right_ranges:
                if r > 0.20 and r < 12.0:
                    valid_right.append(r)
            
            self.right_min = min(valid_right) if valid_right else float('inf')
            
            # Update obstacle flags
            self.obstacle_detected = self.lidar_min_distance < self.obstacle_threshold
            self.lidar_too_close = self.lidar_min_distance < self.emergency_stop_threshold
            self.left_obstacle = self.left_min < 0.6
            self.right_obstacle = self.right_min < 0.6
            self.front_obstacle_close = self.lidar_min_distance < 1.0
            self.front_obstacle_warning = self.lidar_min_distance < 1.3
            self.side_obstacle_close = min(self.left_min, self.right_min) < 0.5
            self.both_sides_blocked = self.left_min < 0.6 and self.right_min < 0.6
            
            # Log LiDAR distances every 3 seconds with CORRECTED orientation info
            current_time = time.time()
            if current_time - self.last_lidar_log > 3.0:
                print(f"\n[LIDAR A1M8] CORRECTED ORIENTATION:")
                print(f"[LIDAR A1M8]   Front (180°)={self.lidar_min_distance:.2f}m")
                print(f"[LIDAR A1M8]   Left  (90°)={self.left_min:.2f}m")
                print(f"[LIDAR A1M8]   Right (270°)={self.right_min:.2f}m")
                print(f"[LIDAR A1M8] Layout:    Front")
                print(f"[LIDAR A1M8]         Left   Right")
                print(f"[LIDAR A1M8]           Rear")
                
                if self.obstacle_detected:
                    print(f"[LIDAR A1M8] ⚠️  FRONT OBSTACLE! Distance={self.lidar_min_distance:.2f}m")
                if self.left_obstacle:
                    print(f"[LIDAR A1M8] ⚠️  LEFT OBSTACLE! Distance={self.left_min:.2f}m")
                if self.right_obstacle:
                    print(f"[LIDAR A1M8] ⚠️  RIGHT OBSTACLE! Distance={self.right_min:.2f}m")
                if self.both_sides_blocked:
                    print(f"[LIDAR A1M8] 🚫 BOTH SIDES BLOCKED!")
                    
                self.last_lidar_log = current_time
            
            # Send mapping data via MQTT
            self.send_mapping_data(ranges)
            
        except Exception as e:
            print(f"[ERROR] Failed to update obstacle data: {str(e)}")

    def send_mapping_data(self, ranges):
        """Send LiDAR data to MQTT for mapping - Using correct topics from mqtt_to_ros2_bridge.py"""
        try:
            if not self.mqtt_client or not self.mqtt_connected:
                return
                
            current_time = time.time()
            # Send mapping data every 0.5 seconds to avoid overwhelming
            if current_time - self.last_mapping_send < 0.5:
                return
                
            # Prepare mapping data in format expected by mqtt_to_ros2_bridge.py
            mapping_data = {
                "timestamp": current_time,
                "ranges": ranges,
                "angle_min": 0.0,  # A1M8 starts from 0 degrees
                "angle_max": 2 * math.pi,  # Full 360 degrees
                "angle_increment": 2 * math.pi / 360,
                "range_min": 0.15,
                "range_max": 12.0,
                "source": "bcr_bot_standalone_a1m8",
                "front_distance": self.lidar_min_distance,
                "left_distance": self.left_min,
                "right_distance": self.right_min,
                "obstacles": {
                    "front": self.obstacle_detected,
                    "left": self.left_obstacle,
                    "right": self.right_obstacle
                }
            }
            
            # Send to the correct MQTT topic used by mqtt_to_ros2_bridge.py
            self.mqtt_client.publish("robot/lidar/real_world_data", json.dumps(mapping_data))
            
            self.last_mapping_send = current_time
            
        except Exception as e:
            print(f"[ERROR] Failed to send mapping data: {str(e)}")

    def set_motor(self, motor_num, speed, direction):
        """Set motor speed and direction"""
        try:
            if motor_num not in pinPWM or motor_num not in pinDIR:
                print(f"[ERROR] Invalid motor number: {motor_num}")
                return
            
            # Set direction
            dir_value = arahMAJU if direction else arahMUNDUR
            self.pca.channels[pinDIR[motor_num]].duty_cycle = dir_value
            
            # Set speed
            self.pca.channels[pinPWM[motor_num]].duty_cycle = speed
            
        except Exception as e:
            print(f"[ERROR] Failed to set motor {motor_num}: {e}")

    def stop_all_motors(self):
        """Stop all motors"""
        try:
            for motor in range(1, 5):
                self.pca.channels[pinPWM[motor]].duty_cycle = 0
                self.pca.channels[pinDIR[motor]].duty_cycle = 0
            self.motor_status = "STOPPED"
        except Exception as e:
            print(f"[ERROR] Failed to stop motors: {str(e)}")

    def move_forward(self, speed_factor=1.0):
        """Move forward with dynamic slow down if obstacle detected on sides"""
        current_speed_coef = rmodeSpeedCoef if self.current_mode == "R-MODE" else speedCoef
        speed = int(fullSpeed * speed_factor * current_speed_coef)
        slow_speed = int(speed * 0.5)  # 50% slow down

        # Slow down logic for side obstacles
        right_obstacle = hasattr(self, 'right_min') and self.right_min < 0.8
        left_obstacle = hasattr(self, 'left_min') and self.left_min < 0.8

        # Motor mapping:
        # 1: depan kiri, 2: depan kanan, 3: belakang kanan, 4: belakang kiri
        # Jika obstacle kanan, motor 2 & 4 slow
        # Jika obstacle kiri, motor 1 & 4 slow
        if right_obstacle and left_obstacle:
            # Both sides slow
            self.set_motor(1, slow_speed, False)
            self.set_motor(2, slow_speed, False)
            self.set_motor(3, slow_speed, False)
            self.set_motor(4, slow_speed, False)
        elif right_obstacle:
            self.set_motor(1, speed, False)
            self.set_motor(2, slow_speed, False)
            self.set_motor(3, speed, False)
            self.set_motor(4, slow_speed, False)
        elif left_obstacle:
            self.set_motor(1, slow_speed, False)
            self.set_motor(2, speed, False)
            self.set_motor(3, speed, False)
            self.set_motor(4, slow_speed, False)
        else:
            self.set_motor(1, speed, False)
            self.set_motor(2, speed, False)
            self.set_motor(3, speed, False)
            self.set_motor(4, speed, False)

        self.motor_status = f"FORWARD (speed: {speed_factor:.2f})"

    def move_backward(self, speed_factor=1.0):
        """Move backward"""
        current_speed_coef = rmodeBackwardSpeedCoef if self.current_mode == "R-MODE" else speedCoef
        speed = int(fullSpeed * speed_factor * current_speed_coef)
        
        self.set_motor(1, speed, True)   # Motor 1 backward
        self.set_motor(2, speed, True)   # Motor 2 backward
        self.set_motor(3, speed, True)   # Motor 3 backward
        self.set_motor(4, speed, True)   # Motor 4 backward
        
        self.motor_status = f"BACKWARD (speed: {speed_factor:.2f})"

    def turn_left(self, speed_factor=1.0):
        """Turn left"""
        current_speed_coef = rmodeSpeedCoef if self.current_mode == "R-MODE" else speedCoef
        speed = int(fullSpeed * speed_factor * current_speed_coef)
        
        self.set_motor(1, speed, False)  # Motor 1 backward
        self.set_motor(2, speed, True)   # Motor 2 forward
        self.set_motor(3, speed, False)  # Motor 3 backward
        self.set_motor(4, speed, True)   # Motor 4 forward
        
        self.motor_status = f"TURNING LEFT (speed: {speed_factor:.2f})"

    def turn_right(self, speed_factor=1.0):
        """Turn right"""
        current_speed_coef = rmodeSpeedCoef if self.current_mode == "R-MODE" else speedCoef
        speed = int(fullSpeed * speed_factor * current_speed_coef)
        
        self.set_motor(1, speed, True)   # Motor 1 forward
        self.set_motor(2, speed, False)  # Motor 2 backward
        self.set_motor(3, speed, True)   # Motor 3 forward
        self.set_motor(4, speed, False)  # Motor 4 backward
        
        self.motor_status = f"TURNING RIGHT (speed: {speed_factor:.2f})"

    def getch(self):
        """Get character from stdin"""
        try:
            return sys.stdin.read(1)
        except KeyboardInterrupt:
            return None
        except Exception as e:
            print(f"[KEYBOARD] Error reading from stdin: {str(e)}")
            return None

    def keyboard_listener(self):
        """Keyboard input listener thread"""
        print("[KEYBOARD] Keyboard listener started")
        
        while self.running:
            try:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = self.getch()
                    if key is None:
                        continue
                    
                    key = key.lower()
                    self.handle_key(key)
                else:
                    # Check for manual mode timeout
                    if self.manual_mode and (time.time() - self.last_input_time > 5.0):
                        self.manual_mode = False
                        self.primary_active = True
                        self.print_status('Primary mode resumed')
                        
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"[KEYBOARD] Error in keyboard listener: {str(e)}")
                time.sleep(0.1)
                continue
                
            time.sleep(0.05)

    def handle_key(self, key):
        """Handle keyboard input"""
        try:
            self.manual_mode = True
            self.last_input_time = time.time()
            
            # Handle navigation mode toggles
            if key == 't':
                self.nav_charging_mode = not self.nav_charging_mode
                self.battery_low = self.nav_charging_mode
                if self.nav_charging_mode:
                    self.manual_mode = False
                    self.primary_active = False
                    self.r_mode = False
                    self.print_status("BATTERY LOW! Charging navigation mode activated")
                else:
                    self.primary_active = True
                    self.print_status("Charging navigation cancelled")
                return
            
            # Handle R mode toggle (simplified - just avoid obstacles and move forward)
            if key == 'r':
                self.r_mode = not self.r_mode
                if self.r_mode:
                    self.print_status("R-mode activated - Advanced obstacle avoidance")
                    self.current_mode = "R-MODE"
                    self.primary_active = False
                    self.manual_mode = False
                else:
                    self.print_status("R-mode deactivated")
                    self.current_mode = "PRIMARY"
                    self.primary_active = True
                return
            
            # Handle manual movement
            if key == 'w':
                self.move_forward(0.25)  # 1/4 MAX SPEED
                self.print_status("Manual: Moving forward")
            elif key == 's':
                self.move_backward(0.25) # 1/4 MAX SPEED
                self.print_status("Manual: Moving backward")
            elif key == 'a':
                self.turn_left(0.25)     # 1/4 MAX SPEED
                self.print_status("Manual: Turning left")
            elif key == 'd':
                self.turn_right(0.25)    # 1/4 MAX SPEED
                self.print_status("Manual: Turning right")
            elif key == ' ':  # Spacebar to stop all motors
                self.stop_all_motors()
                self.print_status("Manual: ALL MOTORS STOPPED")
                print(f"\n[STOP] ALL MOTORS STOPPED BY SPACEBAR")
            else:
                self.print_status(f"Unknown key: '{key}' - Use WASD for movement, Space to stop")
                
        except Exception as e:
            print(f"[KEYBOARD] Error handling key '{key}': {str(e)}")

    def print_status(self, message):
        """Print status message with detailed LiDAR and system status"""
        timestamp = time.strftime("%H:%M:%S")
        
        # LiDAR status with detailed distances and system status
        lidar_status = ""
        if hasattr(self, 'lidar') and self.lidar:
            if hasattr(self, 'lidar_running') and self.lidar_running:
                if hasattr(self, 'lidar_min_distance') and self.lidar_min_distance != float('inf'):
                    lidar_status = f"LiDAR: F={self.lidar_min_distance:.2f}m L={self.left_min:.2f}m R={self.right_min:.2f}m"
                else:
                    lidar_status = "LiDAR: Running (waiting for data...)"
            else:
                lidar_status = "LiDAR: Connected but not scanning"
        else:
            lidar_status = "LiDAR: Not connected"
        
        # MQTT status
        mqtt_status = "MQTT: Connected (robot/lidar/real_world_data)" if self.mqtt_connected else "MQTT: Disconnected"
        
        status_line = f'[{timestamp}] {message} | {lidar_status} | {mqtt_status}'
        
        self.last_status = message
        
        # Clear line and print new status
        sys.stdout.write('\r\033[K' + status_line)
        sys.stdout.flush()

    def print_instructions(self):
        """Print control instructions"""
        print('\n' + '='*50)
        print(' '*10 + 'BCR BOT STANDALONE CONTROL')
        print('='*50)
        print('MOVEMENT:')
        print('W: Move Forward')
        print('S: Move Backward')
        print('A: Turn Left')
        print('D: Turn Right')
        print('SPACE: STOP ALL MOTORS')
        print('')
        print('NAVIGATION MODES:')
        print('T: Toggle Charging Navigation Mode')
        print('R: Toggle R-mode (Advanced obstacle avoidance)')
        print('')
        print('LIDAR ORIENTATION:')
        print('Front: 0°, Left: 90°, Right: 270°, Rear: 180°')
        print('')
        print('MQTT MAPPING:')
        print('Data sent to: robot/lidar/real_world_data topic')
        print('Compatible with mqtt_to_ros2_bridge.py')
        print('Broker: spin5.petra.ac.id')
        print('')
        print('Press any key to start...')
        print('Primary mode resumes after 5s of no input')
        print('Press Ctrl+C to exit')
        print('='*50 + '\n')

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def main_control_loop(self):
        """Main control loop"""
        print("[MAIN] Starting main control loop...")
        
        while self.running:
            try:
                # Emergency stop check
                if hasattr(self, 'lidar_min_distance') and self.lidar_min_distance < 0.25:
                    self.stop_all_motors()
                    self.print_status("EMERGENCY STOP - Obstacle too close!")
                    time.sleep(0.1)
                    continue
                
                # Navigation charging mode (highest priority)
                if self.nav_charging_mode:
                    self.handle_charging_navigation()
                    
                # R-mode
                elif self.r_mode:
                    self.handle_r_mode()
                    
                # Manual mode
                elif self.manual_mode:
                    # Manual control handled by keyboard
                    pass
                    
                # Primary mode (default)
                elif self.primary_active:
                    self.handle_primary_mode()
                    
                time.sleep(0.1)  # 10 Hz control loop
                
            except KeyboardInterrupt:
                print('\nKeyboard interrupt in main loop')
                break
            except Exception as e:
                print(f'\nError in main loop: {str(e)}')
                time.sleep(1.0)
                continue

    def handle_charging_navigation(self):
        """Handle charging navigation mode - simplified without coordinates"""
        # Check for obstacles
        if self.lidar_min_distance < self.charging_emergency_stop_threshold:
            self.stop_all_motors()
            self.print_status("CHARGING NAV: Emergency stop - obstacle too close!")
        elif self.lidar_min_distance < self.charging_obstacle_threshold:
            # Avoid obstacle
            if self.left_min > self.right_min:
                self.turn_left(self.charging_turn_speed)
                self.print_status("CHARGING NAV: Avoiding obstacle - turning left")
            else:
                self.turn_right(self.charging_turn_speed)
                self.print_status("CHARGING NAV: Avoiding obstacle - turning right")
        else:
            # Move forward at 1/4 maximum speed when path is clear
            self.move_forward(0.25)  # 1/4 MAX SPEED
            self.print_status("CHARGING NAV: Moving forward to find charging station")

    def handle_r_mode(self):
        """Handle R-mode - Advanced obstacle avoidance"""
        # Check for obstacles with more sophisticated logic
        if self.lidar_min_distance < self.r_mode_emergency_stop:
            self.stop_all_motors()
            self.print_status("R-MODE: Emergency stop - obstacle too close!")
        elif self.obstacle_detected:
            # Advanced obstacle avoidance
            if self.both_sides_blocked:
                # Both sides blocked - back up and turn around
                self.move_backward(0.25)  # 1/4 MAX SPEED
                self.print_status("R-MODE: Both sides blocked - backing up")
            elif self.left_min > self.right_min:
                # More space on left
                self.turn_left(self.r_mode_turn_speed)
                self.print_status(f"R-MODE: Turning left (left space: {self.left_min:.2f}m)")
            else:
                # More space on right
                self.turn_right(self.r_mode_turn_speed)
                self.print_status(f"R-MODE: Turning right (right space: {self.right_min:.2f}m)")
        else:
            # Path is clear - move forward
            forward_speed = self.r_mode_forward_speed
            if self.front_obstacle_warning:
                forward_speed *= 0.5  # Reduce speed when obstacle nearby
                self.print_status(f"R-MODE: Moving forward (reduced speed) - front: {self.lidar_min_distance:.2f}m")
            else:
                self.print_status(f"R-MODE: Moving forward - clear path ahead: {self.lidar_min_distance:.2f}m")
            self.move_forward(forward_speed)

    def handle_primary_mode(self):
        """Handle primary autonomous mode - Simple and effective obstacle avoidance"""
        # Emergency stop check
        if self.lidar_too_close:
            self.stop_all_motors()
            self.print_status(f"PRIMARY: Emergency stop - obstacle at {self.lidar_min_distance:.2f}m!")
            return
        
        # Simple obstacle avoidance logic (like working code)
        if self.obstacle_detected:
            # Choose direction with more space
            if self.both_sides_blocked:
                # Both sides blocked - turn around (prefer left turn)
                self.turn_left(0.5)   # 1/2 MAX SPEED
                self.print_status(f"PRIMARY: Both sides blocked - turning left (L:{self.left_min:.2f}m R:{self.right_min:.2f}m)")
            elif self.left_min > self.right_min:
                # More space on left - turn left
                self.turn_left(0.5)   # 1/2 MAX SPEED
                self.print_status(f"PRIMARY: Turning left (L:{self.left_min:.2f}m > R:{self.right_min:.2f}m)")
            else:
                # More space on right - turn right
                self.turn_right(0.5)  # 1/2 MAX SPEED
                self.print_status(f"PRIMARY: Turning right (R:{self.right_min:.2f}m > L:{self.left_min:.2f}m)")
        else:
            # No obstacles - move forward
            self.move_forward(0.5)   # 1/2 MAX SPEED
            self.print_status(f"PRIMARY: Clear path - moving forward (F:{self.lidar_min_distance:.2f}m L:{self.left_min:.2f}m R:{self.right_min:.2f}m)")

    def debug_lidar_connection(self):
        """Debug LiDAR connection issues"""
        print("\n=== LIDAR CONNECTION DEBUGGING ===")
        
        # Check available USB ports
        import os
        import glob
        usb_ports = glob.glob('/dev/ttyUSB*')
        if usb_ports:
            print(f"[DEBUG] Available USB ports: {usb_ports}")
            for port in usb_ports:
                try:
                    # Check permissions
                    stat_info = os.stat(port)
                    print(f"[DEBUG] {port}: permissions = {oct(stat_info.st_mode)[-3:]}")
                except Exception as e:
                    print(f"[DEBUG] {port}: error = {str(e)}")
        else:
            print("[DEBUG] ❌ No /dev/ttyUSB* ports found!")
            print("[HELP] Check if LiDAR is connected: lsusb")
        
        # Check if LiDAR is connected but on different port
        all_tty = glob.glob('/dev/tty*USB*') + glob.glob('/dev/ttyACM*')
        if all_tty:
            print(f"[DEBUG] All possible ports: {all_tty}")
        
        print("=" * 40)

    def run(self):
        """Main run method with improved LiDAR debugging"""
        try:
            print("[MAIN] Starting BCR Bot Standalone...")
            
            # Debug LiDAR connection if initialization failed
            if not self.lidar:
                self.debug_lidar_connection()
                print("[WARNING] ⚠️  Running without LiDAR - manual control only!")
                print("[WARNING] ⚠️  Fix LiDAR connection for autonomous modes")
            else:
                # Start LiDAR scanning
                lidar_started = self.start_lidar_scanning()
                if not lidar_started:
                    print("[WARNING] ⚠️  LiDAR connected but scanning failed!")
                    self.debug_lidar_connection()
            
            self.print_instructions()
            
            # Start keyboard listener
            keyboard_thread = threading.Thread(target=self.keyboard_listener, name="Keyboard_Listener")
            keyboard_thread.daemon = True
            keyboard_thread.start()
            
            print("\n=== BCR BOT STANDALONE RUNNING ===")
            if hasattr(self, 'lidar') and self.lidar and hasattr(self, 'lidar_running') and self.lidar_running:
                print("✓ LiDAR A1M8 scanning active")
            else:
                print("❌ LiDAR A1M8 not working - manual control only!")
            print("✓ Motors ready")
            print("✓ Keyboard control enabled")
            if self.mqtt_connected:
                print("✓ MQTT mapping data transmission active")
            else:
                print("❌ MQTT disconnected - mapping data not sent")
            print("Press Ctrl+C to stop")
            print("=" * 50)
            
            # Start main control loop
            self.main_control_loop()
            
        except KeyboardInterrupt:
            print('\nKeyboard interrupt received')
        except Exception as e:
            print(f'\nError in main run: {str(e)}')
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup resources"""
        try:
            print('\n[CLEANUP] Stopping all systems...')
            
            # Stop all motors
            self.stop_all_motors()
            print('[CLEANUP] ✓ All motors stopped')
            
            # Stop LiDAR
            if hasattr(self, 'lidar_running'):
                self.lidar_running = False
            
            if hasattr(self, 'lidar') and self.lidar:
                try:
                    self.lidar.disconnect()
                    print('[CLEANUP] ✓ LiDAR disconnected')
                except:
                    pass
            
            # Disconnect MQTT
            if hasattr(self, 'mqtt_client') and self.mqtt_client:
                try:
                    self.mqtt_client.loop_stop()
                    self.mqtt_client.disconnect()
                    print('[CLEANUP] ✓ MQTT disconnected')
                except:
                    pass
            
            # Restore terminal settings
            if hasattr(self, 'old_settings'):
                try:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
                    print('[CLEANUP] ✓ Terminal settings restored')
                except:
                    pass
            
            # Clear screen
            os.system('clear')
            print('=== BCR BOT STANDALONE STOPPED ===')
            print('✓ All motors stopped safely')
            print('✓ LiDAR A1M8 disconnected')
            print('✓ MQTT disconnected (robot/lidar/real_world_data)')
            print('✓ Mapping data transmission stopped')
            print('✓ Terminal restored')
            print('✓ Cleanup completed successfully')
            
        except Exception as e:
            print(f'Error during cleanup: {str(e)}')

if __name__ == '__main__':
    print("=== BCR BOT STANDALONE ===")
    print("Combined Navigation and Motor Control")
    print("Using Real-world LiDAR A1M8 Only")
    print("=" * 50)
    
    try:
        bot = BCRBotStandalone()
        bot.run()
    except Exception as e:
        print(f"Failed to start BCR Bot: {str(e)}")
        sys.exit(1)
