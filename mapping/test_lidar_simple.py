#!/usr/bin/env python3
"""
Simple LiDAR test script that works around rplidar library bugs
Uses alternative approach to avoid iter_scans() issues
"""

import time
import math
import json
import threading
from rplidar import RPLidar
import paho.mqtt.client as mqtt

# Konfigurasi LiDAR
LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_BAUDRATE = 115200
LIDAR_TIMEOUT = 1.0

# MQTT Configuration
MQTT_BROKER = "spin5.petra.ac.id"
MQTT_PORT = 1883
MQTT_TOPIC_LIDAR_DATA = "robot/lidar/real_world_data"
MQTT_TOPIC_LIDAR_STATUS = "robot/lidar/status"

class SimpleLiDARSender:
    """Simplified LiDAR sender that works around library bugs"""
    
    def __init__(self):
        self.mqtt_client = None
        self.lidar = None
        self.is_running = False
        self.init_mqtt()
    
    def init_mqtt(self):
        """Initialize MQTT connection"""
        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_publish = self.on_mqtt_publish
            
            print(f"[MQTT] Connecting to {MQTT_BROKER}:{MQTT_PORT}...")
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            print("[MQTT] Connected successfully")
            
        except Exception as e:
            print(f"[ERROR] MQTT connection failed: {str(e)}")
            self.mqtt_client = None
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("[MQTT] Connected to broker for RViz visualization")
        else:
            print(f"[MQTT] Connection failed with code {rc}")
    
    def on_mqtt_publish(self, client, userdata, mid):
        print(f"[MQTT] ✓ Message {mid} published successfully")
    
    def connect_lidar(self):
        """Connect to LiDAR with error handling"""
        try:
            print(f"[LIDAR] Connecting to LiDAR on {LIDAR_PORT}...")
            self.lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE, timeout=LIDAR_TIMEOUT)
            
            # Get info
            info = self.lidar.get_info()
            print(f"[LIDAR] Connected: {info}")
            
            # Try to get health (with error handling)
            try:
                health = self.lidar.get_health()
                print(f"[LIDAR] Health: {health}")
            except:
                print("[LIDAR] Health check skipped (library issue)")
            
            return True
            
        except Exception as e:
            print(f"[ERROR] LiDAR connection failed: {e}")
            return False
    
    def send_mock_data(self):
        """Send mock LiDAR data for testing"""
        print("[LIDAR] Sending mock data for testing...")
        
        # Create mock 360-degree scan data
        ranges = []
        for angle_deg in range(360):
            # Create some realistic mock data
            if 80 <= angle_deg <= 100:  # Obstacle at 90 degrees
                distance = 0.8  # 80cm
            elif 170 <= angle_deg <= 190:  # Obstacle at 180 degrees  
                distance = 1.2  # 120cm
            elif 260 <= angle_deg <= 280:  # Obstacle at 270 degrees
                distance = 0.6  # 60cm
            else:
                distance = 3.0  # 3 meters (no obstacle)
            
            ranges.append(distance)
        
        # Create data packet
        lidar_data = {
            "timestamp": time.time(),
            "ranges": ranges,
            "angle_min": 0.0,
            "angle_max": 2 * math.pi,
            "angle_increment": math.radians(1),
            "range_min": 0.15,
            "range_max": 12.0,
            "source": "real_world_lidar_rviz_mock"
        }
        
        # Send via MQTT
        if self.mqtt_client:
            json_data = json.dumps(lidar_data)
            result = self.mqtt_client.publish(MQTT_TOPIC_LIDAR_DATA, json_data)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                print(f"[LIDAR] ✓ Sent mock data: 360 points")
                print(f"[MQTT] ✓ Data size: {len(json_data)} bytes")
                return True
            else:
                print(f"[ERROR] MQTT publish failed: {result.rc}")
                return False
        return False
    
    def send_real_lidar_data(self, scan_data):
        """Send real LiDAR data to MQTT bridge"""
        if self.mqtt_client is None:
            return False
        
        try:
            # Convert scan data to 360-degree ranges array
            ranges = [float('inf')] * 360
            
            for quality, angle, distance in scan_data:
                angle_deg = int(angle) % 360
                if 0 <= angle_deg < 360:
                    ranges[angle_deg] = distance / 1000.0  # Convert to meters
            
            # Create data packet for RViz visualization
            lidar_data = {
                "timestamp": time.time(),
                "ranges": ranges,
                "angle_min": 0.0,
                "angle_max": 2 * math.pi,
                "angle_increment": math.radians(1),
                "range_min": 0.15,
                "range_max": 12.0,
                "source": "real_world_lidar_rviz"
            }
            
            # Send via MQTT
            json_data = json.dumps(lidar_data)
            result = self.mqtt_client.publish(MQTT_TOPIC_LIDAR_DATA, json_data)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                valid_ranges = sum(1 for r in ranges if r != float('inf'))
                print(f"[LIDAR] ✓ Sent real data: {len(scan_data)} points -> {valid_ranges} valid ranges")
                return True
            else:
                print(f"[ERROR] MQTT publish failed: {result.rc}")
                return False
                
        except Exception as e:
            print(f"[ERROR] Failed to send real LiDAR data: {e}")
            return False
    
    def send_real_lidar_data(self, scan_data):
        """Send real LiDAR data to MQTT bridge"""
        if self.mqtt_client is None:
            return False
        
        try:
            # Convert scan data to 360-degree ranges array
            ranges = [float('inf')] * 360
            
            for quality, angle, distance in scan_data:
                angle_deg = int(angle) % 360
                if 0 <= angle_deg < 360:
                    ranges[angle_deg] = distance / 1000.0  # Convert to meters
            
            # Create data packet for RViz visualization
            lidar_data = {
                "timestamp": time.time(),
                "ranges": ranges,
                "angle_min": 0.0,
                "angle_max": 2 * math.pi,
                "angle_increment": math.radians(1),
                "range_min": 0.15,
                "range_max": 12.0,
                "source": "real_world_lidar_rviz"
            }
            
            # Send via MQTT
            json_data = json.dumps(lidar_data)
            result = self.mqtt_client.publish(MQTT_TOPIC_LIDAR_DATA, json_data)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                valid_ranges = sum(1 for r in ranges if r != float('inf'))
                print(f"[LIDAR] ✓ Sent real data: {len(scan_data)} points -> {valid_ranges} valid ranges")
                return True
            else:
                print(f"[ERROR] MQTT publish failed: {result.rc}")
                return False
                
        except Exception as e:
            print(f"[ERROR] Failed to send real LiDAR data: {e}")
            return False
    
    def send_real_lidar_data(self, scan_data):
        """Send real LiDAR data to MQTT bridge"""
        if self.mqtt_client is None:
            return False
        
        try:
            # Convert scan data to 360-degree ranges array
            ranges = [float('inf')] * 360
            
            for quality, angle, distance in scan_data:
                angle_deg = int(angle) % 360
                if 0 <= angle_deg < 360:
                    ranges[angle_deg] = distance / 1000.0  # Convert to meters
            
            # Create data packet for RViz visualization
            lidar_data = {
                "timestamp": time.time(),
                "ranges": ranges,
                "angle_min": 0.0,
                "angle_max": 2 * math.pi,
                "angle_increment": math.radians(1),
                "range_min": 0.15,
                "range_max": 12.0,
                "source": "real_world_lidar_rviz"
            }
            
            # Send via MQTT
            json_data = json.dumps(lidar_data)
            result = self.mqtt_client.publish(MQTT_TOPIC_LIDAR_DATA, json_data)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                valid_ranges = sum(1 for r in ranges if r != float('inf'))
                print(f"[LIDAR] ✓ Sent real data: {len(scan_data)} points -> {valid_ranges} valid ranges")
                return True
            else:
                print(f"[ERROR] MQTT publish failed: {result.rc}")
                return False
                
        except Exception as e:
            print(f"[ERROR] Failed to send real LiDAR data: {e}")
            return False
    
    def start_mock_streaming(self):
        """Start mock data streaming for testing"""
        print("=== MOCK LIDAR STREAMING MODE ===")
        print("Sending mock LiDAR data to test MQTT bridge")
        print("This bypasses the rplidar library bug")
        print("=" * 50)
        
        self.is_running = True
        scan_count = 0
        
        try:
            while self.is_running:
                if self.send_mock_data():
                    scan_count += 1
                    print(f"[LIDAR] Sent mock scan {scan_count}")
                    
                    if scan_count % 10 == 0:
                        print(f"[LIDAR] Total mock scans sent: {scan_count}")
                
                time.sleep(1.0)  # Send data every second
                
        except KeyboardInterrupt:
            print("\n[LIDAR] Stopping mock streaming...")
        finally:
            self.stop_streaming()
    
    def start_real_streaming(self):
        """Attempt real LiDAR streaming (may fail due to library bug)"""
        print("=== REAL LIDAR STREAMING MODE ===")
        print("Attempting to use real LiDAR data")
        print("This may fail due to rplidar library bugs")
        print("=" * 50)
        
        if not self.connect_lidar():
            print("[ERROR] Could not connect to LiDAR")
            return
        
        print("[LIDAR] LiDAR connected, streaming should work!")
        print("[LIDAR] Press Ctrl+C to stop")
        
        # Try to start motor
        try:
            self.lidar.start_motor()
            print("[LIDAR] Motor started")
            time.sleep(2)
        except Exception as e:
            print(f"[ERROR] Motor start failed: {e}")
        
        # Attempt continuous scanning
        try:
            print("[LIDAR] Starting continuous scanning...")
            self.is_running = True
            scan_count = 0
            
            for scan in self.lidar.iter_scans():
                if not self.is_running:
                    break
                    
                scan_count += 1
                print(f"[LIDAR] Processing scan {scan_count} with {len(scan)} points")
                
                # Convert scan data and send to MQTT
                self.send_real_lidar_data(scan)
                
                # Print status every 10 scans
                if scan_count % 10 == 0:
                    print(f"[LIDAR] Sent {scan_count} scans to MQTT bridge")
                
                # Small delay
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n[LIDAR] Stopping real streaming...")
        except Exception as e:
            print(f"[ERROR] Scanning failed: {e}")
            print("[ERROR] This is the expected library bug")
            print("[ERROR] Use mock mode instead")
        finally:
            self.stop_streaming()
    
    def stop_streaming(self):
        """Stop streaming and cleanup"""
        self.is_running = False
        
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except:
                pass
            self.lidar = None
        
        if self.mqtt_client:
            try:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except:
                pass
        
        print("[LIDAR] Cleanup completed")

def main():
    """Main function"""
    print("Simple LiDAR Test Script (Library Bug Workaround)")
    print("=" * 60)
    print("Pilih mode:")
    print("1. Mock data streaming (RECOMMENDED - works around library bug)")
    print("2. Real LiDAR streaming (may fail due to library bug)")
    print("3. Exit")
    
    try:
        choice = input("\nPilih mode (1-3): ").strip()
        
        sender = SimpleLiDARSender()
        
        if choice == "1":
            # Mock mode - guaranteed to work
            sender.start_mock_streaming()
        elif choice == "2":
            # Real mode - will likely fail due to library bug
            sender.start_real_streaming()
        elif choice == "3":
            print("Exiting...")
            return
        else:
            print("Invalid choice, using mock mode...")
            sender.start_mock_streaming()
            
    except KeyboardInterrupt:
        print("\n\nScript dihentikan oleh user")
    except Exception as e:
        print(f"\nError: {str(e)}")

if __name__ == "__main__":
    main()
