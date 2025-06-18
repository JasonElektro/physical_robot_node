#!/usr/bin/env python3
import time
import board
import busio
import adafruit_pca9685
import RPi.GPIO as GPIO


# Constants
PCA_FREQ = 1500
fullSpeed = 0xFFFF  # 65535
halfSpeed = 0x7FFF  # 32767
noSpeed = 0x0000    # 0
speedCoef = 0.3     # Speed reduction factor


# Arah putaran motor
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


class RobotTester:
   def __init__(self):
       print("[INIT] Initializing PCA9685...")
       try:
           # Initialize I2C bus
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


   def set_motor(self, motor_num, speed, direction):
       """Set motor speed and direction"""
       try:
           if motor_num not in pinPWM or motor_num not in pinDIR:
               print(f"[ERROR] Invalid motor number: {motor_num}")
               return
          
           # Set direction
           dir_value = arahMAJU if direction else arahMUNDUR
           self.pca.channels[pinDIR[motor_num]].duty_cycle = dir_value
           print(f"[DEBUG] Motor {motor_num} direction set to {'forward' if direction else 'backward'}")
          
           # Set speed
           self.pca.channels[pinPWM[motor_num]].duty_cycle = speed
           print(f"[DEBUG] Motor {motor_num} speed set to {speed}")
          
       except Exception as e:
           print(f"[ERROR] Failed to set motor {motor_num}: {e}")


   def stop_all_motors(self):
       """Stop all motors"""
       print("[DEBUG] Stopping all motors")
       for motor in range(1, 5):
           self.pca.channels[pinPWM[motor]].duty_cycle = 0
           self.pca.channels[pinDIR[motor]].duty_cycle = 0
       print("[DEBUG] All motors stopped")


   def test_individual_motors(self):
       """Test each motor individually"""
       print("\n[TEST] Testing individual motors...")
       for motor in range(1, 5):
           print(f"\n[TEST] Testing Motor {motor}")
           print(f"[TEST] Setting PWM pin {pinPWM[motor]} and DIR pin {pinDIR[motor]}")
          
           # Test forward
           print(f"[TEST] Motor {motor} - Forward")
           self.set_motor(motor, halfSpeed, True)
           time.sleep(2)
          
           # Stop
           print(f"[TEST] Motor {motor} - Stop")
           self.set_motor(motor, 0, True)
           time.sleep(1)
          
           # Test backward
           print(f"[TEST] Motor {motor} - Backward")
           self.set_motor(motor, halfSpeed, False)
           time.sleep(2)
          
           # Stop
           print(f"[TEST] Motor {motor} - Stop")
           self.set_motor(motor, 0, False)
           time.sleep(1)


   def test_movement(self):
       """Test all movement patterns"""
       print("\n[TEST] Testing movement patterns...")
       speed = int(fullSpeed * speedCoef)
      
       # Test forward
       print("\n[TEST] Testing forward movement")
       self.set_motor(1, speed, True)   # Motor 1 forward
       self.set_motor(2, speed, False)  # Motor 2 backward
       self.set_motor(3, speed, True)   # Motor 3 forward
       self.set_motor(4, speed, False)  # Motor 4 backward
       time.sleep(3)
       self.stop_all_motors()
       time.sleep(1)
      
       # Test backward
       print("\n[TEST] Testing backward movement")
       self.set_motor(1, speed, False)  # Motor 1 backward
       self.set_motor(2, speed, True)   # Motor 2 forward
       self.set_motor(3, speed, False)  # Motor 3 backward
       self.set_motor(4, speed, True)   # Motor 4 forward
       time.sleep(3)
       self.stop_all_motors()
       time.sleep(1)
      
       # Test left turn
       print("\n[TEST] Testing left turn")
       self.set_motor(1, speed, False)  # Motor 1 backward
       self.set_motor(2, speed, True)   # Motor 2 forward
       self.set_motor(3, speed, False)  # Motor 3 backward
       self.set_motor(4, speed, True)   # Motor 4 forward
       time.sleep(3)
       self.stop_all_motors()
       time.sleep(1)
      
       # Test right turn
       print("\n[TEST] Testing right turn")
       self.set_motor(1, speed, True)   # Motor 1 forward
       self.set_motor(2, speed, False)  # Motor 2 backward
       self.set_motor(3, speed, True)   # Motor 3 forward
       self.set_motor(4, speed, False)  # Motor 4 backward
       time.sleep(3)
       self.stop_all_motors()
       time.sleep(1)


   def run_tests(self):
       """Run all tests"""
       try:
           print("[TEST] Starting robot tests...")
          
           # Test individual motors
           self.test_individual_motors()
          
           # Test movement patterns
           self.test_movement()
          
           print("\n[TEST] All tests completed")
          
       except KeyboardInterrupt:
           print("\n[TEST] Tests interrupted by user")
       finally:
           self.stop_all_motors()
           print("[TEST] Robot stopped")


if __name__ == "__main__":
   tester = RobotTester()
   tester.run_tests()
