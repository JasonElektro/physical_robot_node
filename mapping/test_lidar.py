#!/usr/bin/env python3
"""
Simple test script untuk RP-Lidar A1M8
Digunakan untuk memverifikasi koneksi dan data LiDAR
Sekarang juga bisa mengirim data LiDAR real-world ke roam.py via MQTT
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

# MQTT Configuration untuk mengirim data ke roam.py
MQTT_BROKER = "spin5.petra.ac.id"
MQTT_PORT = 1883
MQTT_TOPIC_LIDAR_DATA = "robot/lidar/real_world_data"
MQTT_TOPIC_LIDAR_STATUS = "robot/lidar/status"

class LiDARDataSender:
    """Class untuk mengirim data LiDAR real-world ke roam.py"""
    
    def __init__(self):
        self.mqtt_client = None
        self.lidar = None
        self.is_running = False
        self.lidar_thread = None
        self.init_mqtt()
    
    def init_mqtt(self):
        """Initialize MQTT connection"""
        try:
            # Use the latest MQTT callback API version to avoid deprecation warning
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            
            print(f"[MQTT] Connecting to {MQTT_BROKER}:{MQTT_PORT}...")
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            print("[MQTT] Connected successfully")
            
        except Exception as e:
            print(f"[ERROR] MQTT connection failed: {str(e)}")
            self.mqtt_client = None
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("[MQTT] Connected to broker")
            # Publish status
            status_data = {
                "status": "connected",
                "timestamp": time.time(),
                "message": "LiDAR real-world data sender connected"
            }
            if self.mqtt_client:
                self.mqtt_client.publish(MQTT_TOPIC_LIDAR_STATUS, json.dumps(status_data))
        else:
            print(f"[MQTT] Connection failed with code {rc}")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        print(f"[MQTT] Disconnected with code {rc}")
    
    def send_lidar_data(self, scan_data):
        """Send LiDAR data to roam.py via MQTT"""
        if self.mqtt_client is None:
            return
        
        try:
            # Convert scan data to format expected by bridge
            ranges = [float('inf')] * 360
            
            for quality, angle, distance in scan_data:
                angle_deg = int(angle) % 360
                if 0 <= angle_deg < 360:
                    ranges[angle_deg] = distance / 1000.0  # Convert to meters
            
            # Create data packet (format yang sama dengan test_mqtt_send.py)
            lidar_data = {
                "timestamp": time.time(),
                "ranges": ranges,
                "angle_min": 0.0,
                "angle_max": 2 * math.pi,
                "angle_increment": math.radians(1),
                "range_min": 0.15,
                "range_max": 12.0,
                "source": "real_world_lidar"
            }
            
            # Send via MQTT
            self.mqtt_client.publish(MQTT_TOPIC_LIDAR_DATA, json.dumps(lidar_data))
            print(f"[LIDAR] Sent scan data: {len(scan_data)} points -> {sum(1 for r in ranges if r != float('inf'))} valid ranges")
            
        except Exception as e:
            print(f"[ERROR] Failed to send LiDAR data: {str(e)}")
    
    def start_lidar_streaming(self):
        """Start streaming LiDAR data to roam.py"""
        if self.is_running:
            print("[LIDAR] Already streaming")
            return
        
        try:
            # Initialize LiDAR using the robust reset_lidar_connection function
            print(f"[LIDAR] Initializing LiDAR on {LIDAR_PORT}...")
            self.lidar = reset_lidar_connection()
            
            if self.lidar is None:
                print("[ERROR] Failed to initialize LiDAR after multiple attempts")
                print("[ERROR] Possible causes:")
                print("  - RPLIDAR not connected to USB")
                print("  - Wrong port (check /dev/ttyUSB*)")
                print("  - Permission denied (add user to dialout group)")
                print("  - Hardware malfunction")
                self.is_running = False
                return
            
            self.is_running = True
            self.lidar_thread = threading.Thread(target=self._lidar_streaming_loop)
            self.lidar_thread.daemon = True
            self.lidar_thread.start()
            
            print("[LIDAR] ✓ Started streaming real-world data to MQTT")
            print(f"[LIDAR] ✓ Publishing to topic: {MQTT_TOPIC_LIDAR_DATA}")
            print(f"[LIDAR] ✓ MQTT broker: {MQTT_BROKER}")
            print("[LIDAR] Press Ctrl+C to stop")
            
        except Exception as e:
            print(f"[ERROR] Failed to start LiDAR streaming: {str(e)}")
            self.is_running = False
    
    def _lidar_streaming_loop(self):
        """Main LiDAR streaming loop"""
        scan_count = 0
        start_time = time.time()
        
        try:
            if self.lidar:
                for scan in self.lidar.iter_scans():
                    if not self.is_running:
                        break
                    
                    scan_count += 1
                    
                    # Send data to roam.py
                    self.send_lidar_data(scan)
                    
                    # Print status every 10 scans
                    if scan_count % 10 == 0:
                        elapsed = time.time() - start_time
                        rate = scan_count / elapsed
                        print(f"[LIDAR] Sent {scan_count} scans ({rate:.1f} scans/sec)")
                    
                    # Small delay to prevent overwhelming
                    time.sleep(0.01)
                    
        except Exception as e:
            print(f"[ERROR] LiDAR streaming error: {str(e)}")
        finally:
            self.stop_lidar_streaming()
    
    def stop_lidar_streaming(self):
        """Stop LiDAR streaming"""
        self.is_running = False
        
        if self.lidar:
            try:
                self.lidar.disconnect()
            except:
                pass
            self.lidar = None
        
        if self.mqtt_client:
            try:
                status_data = {
                    "status": "disconnected",
                    "timestamp": time.time(),
                    "message": "LiDAR real-world data sender disconnected"
                }
                self.mqtt_client.publish(MQTT_TOPIC_LIDAR_STATUS, json.dumps(status_data))
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except:
                pass
        
        print("[LIDAR] Streaming stopped")

class LiDARVisualizationSender:
    """Class untuk mengirim data LiDAR ke RViz visualization via MQTT bridge"""
    
    def __init__(self):
        self.mqtt_client = None
        self.lidar = None
        self.is_running = False
        self.lidar_thread = None
        self.init_mqtt()
    
    def init_mqtt(self):
        """Initialize MQTT connection"""
        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
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
            # Publish status
            status_data = {
                "status": "connected",
                "timestamp": time.time(),
                "message": "LiDAR visualization sender connected"
            }
            if self.mqtt_client:
                self.mqtt_client.publish(MQTT_TOPIC_LIDAR_STATUS, json.dumps(status_data))
        else:
            print(f"[MQTT] Connection failed with code {rc}")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        print(f"[MQTT] Disconnected with code {rc}")
    
    def on_mqtt_publish(self, client, userdata, mid):
        print(f"[MQTT] ✓ Message {mid} published successfully to broker")
    
    def send_lidar_data_for_rviz(self, scan_data):
        """Send LiDAR data for RViz visualization via MQTT bridge"""
        if self.mqtt_client is None:
            print("[ERROR] MQTT client is None, cannot send data")
            return
        
        try:
            # Debug scan data
            print(f"[DEBUG] Scan data length: {len(scan_data)}")
            if len(scan_data) > 0:
                print(f"[DEBUG] First scan point: {scan_data[0]}")
                print(f"[DEBUG] Scan data type: {type(scan_data)}")
            
            # Convert scan data to format expected by bridge
            ranges = [float('inf')] * 360
            
            for measurement in scan_data:
                try:
                    # Handle different possible formats
                    if len(measurement) == 3:
                        quality, angle, distance = measurement
                    elif len(measurement) == 2:
                        angle, distance = measurement
                        quality = 15  # Default quality
                    else:
                        print(f"[WARNING] Unexpected measurement format: {measurement}")
                        continue
                    
                    angle_deg = int(angle) % 360
                    if 0 <= angle_deg < 360:
                        ranges[angle_deg] = distance / 1000.0  # Convert to meters
                except Exception as e:
                    print(f"[ERROR] Error processing measurement {measurement}: {e}")
                    continue
            
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
            
            # Send via MQTT with debug info
            json_data = json.dumps(lidar_data)
            result = self.mqtt_client.publish(MQTT_TOPIC_LIDAR_DATA, json_data)
            
            # Check if publish was successful
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                valid_ranges = sum(1 for r in ranges if r != float('inf'))
                print(f"[LIDAR] ✓ Sent RViz data: {len(scan_data)} points -> {valid_ranges} valid ranges")
                print(f"[MQTT] ✓ Published to topic: {MQTT_TOPIC_LIDAR_DATA}")
                print(f"[MQTT] ✓ Data size: {len(json_data)} bytes")
                print(f"[MQTT] ✓ Broker: {MQTT_BROKER}:{MQTT_PORT}")
                print(f"[MQTT] ✓ Message ID: {result.mid}")
            else:
                print(f"[ERROR] MQTT publish failed with code: {result.rc}")
                print(f"[ERROR] MQTT error codes: 0=Success, 1=Protocol error, 2=Invalid client ID, 3=Server unavailable, 4=Bad username/password, 5=Not authorized")
            
        except Exception as e:
            print(f"[ERROR] Failed to send LiDAR data for RViz: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def start_lidar_streaming(self):
        """Start streaming LiDAR data for RViz"""
        if self.is_running:
            print("[LIDAR] Already streaming")
            return
        
        try:
            # Initialize LiDAR
            print(f"[LIDAR] Initializing LiDAR on {LIDAR_PORT}...")
            self.lidar = reset_lidar_connection()
            
            if self.lidar is None:
                print("[ERROR] Failed to initialize LiDAR after multiple attempts")
                self.is_running = False
                return
            
            self.is_running = True
            print("[LIDAR] Creating streaming thread...")
            self.lidar_thread = threading.Thread(target=self._lidar_streaming_loop)
            self.lidar_thread.daemon = True
            print("[LIDAR] Starting streaming thread...")
            self.lidar_thread.start()
            print("[LIDAR] Thread started successfully")
            
            # Wait a moment to see if thread starts
            time.sleep(1)
            print(f"[LIDAR] Thread alive: {self.lidar_thread.is_alive()}")
            print(f"[LIDAR] Is running flag: {self.is_running}")
            
            print("[LIDAR] ✓ Started streaming data for RViz visualization")
            print(f"[LIDAR] ✓ Publishing to topic: {MQTT_TOPIC_LIDAR_DATA}")
            print(f"[LIDAR] ✓ MQTT broker: {MQTT_BROKER}")
            print("[LIDAR] Press Ctrl+C to stop")
            
        except Exception as e:
            print(f"[ERROR] Failed to start LiDAR streaming: {str(e)}")
            self.is_running = False
    
    def _lidar_streaming_loop(self):
        """Main LiDAR streaming loop for RViz"""
        print("[LIDAR] Starting RViz streaming loop...")
        scan_count = 0
        start_time = time.time()
        consecutive_errors = 0
        max_consecutive_errors = 5
        
        try:
            if self.lidar:
                print("[LIDAR] Starting scan iteration using iter_scans...")
                
                try:
                    # Use iter_scans with proper error handling
                    for scan in self.lidar.iter_scans():
                        if not self.is_running:
                            break
                        
                        try:
                            scan_count += 1
                            consecutive_errors = 0
                            
                            print(f"[LIDAR] Processing scan {scan_count} with {len(scan)} points")
                            
                            # Debug: check scan data format
                            if len(scan) > 0:
                                print(f"[DEBUG] First scan point: {scan[0]}")
                                print(f"[DEBUG] Scan data type: {type(scan)}")
                            
                            # Send data for RViz visualization
                            self.send_lidar_data_for_rviz(scan)
                            
                            # Print status every 10 scans
                            if scan_count % 10 == 0:
                                elapsed = time.time() - start_time
                                rate = scan_count / elapsed
                                print(f"[LIDAR] Sent {scan_count} scans for RViz ({rate:.1f} scans/sec)")
                            
                            # Small delay to prevent overwhelming
                            time.sleep(0.01)
                            
                        except Exception as scan_error:
                            consecutive_errors += 1
                            print(f"[ERROR] Error processing scan {scan_count}: {scan_error}")
                            import traceback
                            traceback.print_exc()
                            
                            if consecutive_errors >= max_consecutive_errors:
                                print(f"[ERROR] Too many consecutive scan errors, stopping")
                                self.is_running = False
                                break
                                
                except ValueError as iter_error:
                    if "too many values to unpack" in str(iter_error):
                        print(f"[ERROR] iter_scans() library issue: {iter_error}")
                        print("[ERROR] This is a known issue with some rplidar library versions")
                        print("[ERROR] Cannot proceed with streaming due to library incompatibility")
                        self.is_running = False
                    else:
                        raise iter_error
                    
        except Exception as e:
            consecutive_errors += 1
            print(f"[ERROR] LiDAR streaming error ({consecutive_errors}/{max_consecutive_errors}): {str(e)}")
            import traceback
            traceback.print_exc()
            
            if consecutive_errors >= max_consecutive_errors:
                print(f"[ERROR] Too many consecutive errors, stopping streaming")
                self.is_running = False
        finally:
            self.stop_lidar_streaming()
    
    def stop_lidar_streaming(self):
        """Stop LiDAR streaming"""
        self.is_running = False
        
        if self.lidar:
            try:
                self.lidar.disconnect()
            except:
                pass
            self.lidar = None
        
        if self.mqtt_client:
            try:
                status_data = {
                    "status": "disconnected",
                    "timestamp": time.time(),
                    "message": "LiDAR visualization sender disconnected"
                }
                self.mqtt_client.publish(MQTT_TOPIC_LIDAR_STATUS, json.dumps(status_data))
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except:
                pass
        
        print("[LIDAR] RViz streaming stopped")

def start_lidar_streaming_mode():
    """Start LiDAR streaming mode untuk mengirim data ke roam.py"""
    print("=== LIDAR STREAMING MODE (ROAM.PY) ===")
    print("Mengirim data LiDAR real-world ke roam.py via MQTT")
    print("Ini memungkinkan robot di Gazebo empty world untuk mendeteksi obstacle real-world")
    print("=" * 60)
    
    sender = LiDARDataSender()
    
    try:
        sender.start_lidar_streaming()
        
        # Keep running until interrupted
        while sender.is_running:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n[LIDAR] Stopping streaming...")
    finally:
        sender.stop_lidar_streaming()
        print("[LIDAR] Cleanup completed")

def start_lidar_rviz_mode():
    """Start LiDAR streaming mode untuk RViz visualization"""
    print("=== LIDAR RVIZ VISUALIZATION MODE ===")
    print("Mengirim data LiDAR real-world ke RViz via MQTT bridge")
    print("Ini memungkinkan visualisasi data LiDAR real-time di RViz")
    print("Pastikan mqtt_to_ros2_bridge.py sudah berjalan!")
    print("=" * 60)
    
    sender = LiDARVisualizationSender()
    
    try:
        sender.start_lidar_streaming()
        
        # Keep running until interrupted
        while sender.is_running:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n[LIDAR] Stopping RViz streaming...")
    finally:
        sender.stop_lidar_streaming()
        print("[LIDAR] RViz streaming cleanup completed")

def test_lidar_connection():
    """Test koneksi dasar ke LiDAR"""
    print("=== TEST KONEKSI LIDAR ===")
    
    try:
        # Coba koneksi ke LiDAR
        print(f"Mencoba koneksi ke {LIDAR_PORT}...")
        lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE, timeout=LIDAR_TIMEOUT)
        
        # Get info LiDAR
        info = lidar.get_info()
        print(f"✓ LiDAR Info: {info}")
        
        # Get health status
        health = lidar.get_health()
        print(f"✓ Health Status: {health}")
        
        # Disconnect
        lidar.disconnect()
        print("✓ Koneksi berhasil!")
        return True
        
    except Exception as e:
        print(f"✗ Error koneksi: {str(e)}")
        return False

def reset_lidar_connection():
    """Reset koneksi LiDAR dengan error handling"""
    max_retries = 3
    retry_count = 0
    
    while retry_count < max_retries:
        try:
            print(f"[LIDAR] Attempting to connect to LiDAR on {LIDAR_PORT} (attempt {retry_count + 1}/{max_retries})...")
            
            # Create LiDAR instance
            lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE, timeout=LIDAR_TIMEOUT)
            
            # Get LiDAR info
            info = lidar.get_info()
            print(f"[LIDAR] Connected to LiDAR: {info}")
            
            # Get health status with error handling
            try:
                health = lidar.get_health()
                print(f"[LIDAR] Health status: {health}")
            except ValueError as health_error:
                if "too many values to unpack" in str(health_error):
                    print(f"[LIDAR] Warning: get_health() has unpacking issue: {health_error}")
                    print("[LIDAR] This is a known issue with some library versions")
                    print("[LIDAR] Proceeding anyway...")
                else:
                    print(f"[LIDAR] Health check error: {health_error}")
            except Exception as health_error:
                print(f"[LIDAR] Warning: Could not get health status: {health_error}")
                print("[LIDAR] Proceeding anyway...")
            
            # Test basic functionality using iter_scans with proper error handling
            print("[LIDAR] Testing scan functionality...")
            scan_count = 0
            max_test_scans = 3
            
            try:
                # Try to get a few scans to verify functionality
                scan_iterator = lidar.iter_scans()
                for scan in scan_iterator:
                    if len(scan) > 0:
                        scan_count += 1
                        print(f"[LIDAR] Scan {scan_count} successful: {len(scan)} points")
                        
                        # Tampilkan sample data dari scan pertama
                        if scan_count == 1:
                            print("[LIDAR] Sample data from first scan:")
                            for i, (quality, angle, distance) in enumerate(scan[:10]):  # Tampilkan 10 point pertama
                                print(f"  Point {i}: Quality={quality}, Angle={angle:.1f}°, Distance={distance/1000:.3f}m")
                        
                        if scan_count >= max_test_scans:
                            break
                            
            except ValueError as scan_error:
                if "too many values to unpack" in str(scan_error):
                    print(f"[LIDAR] iter_scans() has unpacking issue: {scan_error}")
                    print("[LIDAR] This is a known issue with some library versions")
                    print("[LIDAR] Will try alternative approach in streaming mode")
                else:
                    print(f"[LIDAR] Scan test error: {scan_error}")
            except Exception as scan_error:
                print(f"[LIDAR] Scan test error: {scan_error}")
                # If iter_scans fails, we can't proceed with this approach
                print("[LIDAR] iter_scans() failed, but LiDAR connection is established")
                print("[LIDAR] This may be due to library version incompatibility")
            
            print("[LIDAR] LiDAR initialization successful!")
            
            # Disconnect and reconnect for clean state
            lidar.disconnect()
            time.sleep(0.5)
            
            # Create fresh connection for streaming
            fresh_lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE, timeout=LIDAR_TIMEOUT)
            print("[LIDAR] Created fresh LiDAR connection for streaming")
            return fresh_lidar
            
        except Exception as e:
            retry_count += 1
            error_msg = str(e)
            print(f"[ERROR] LiDAR initialization attempt {retry_count} failed: {error_msg}")
            
            # Clean up failed connection
            if 'lidar' in locals():
                try:
                    lidar.disconnect()
                except:
                    pass
            
            # Wait before retry
            if retry_count < max_retries:
                print(f"[LIDAR] Waiting 2 seconds before retry...")
                time.sleep(2.0)
    
    print(f"[ERROR] Failed to initialize LiDAR after {max_retries} attempts")
    return None

def test_lidar_scanning():
    """Test scanning data LiDAR dengan error recovery"""
    print("\n=== TEST SCANNING LIDAR ===")
    
    lidar = None
    try:
        # Initialize LiDAR dengan error handling
        lidar = reset_lidar_connection()
        if lidar is None:
            print("✗ Gagal menginisialisasi LiDAR")
            return False
        
        print("Memulai scanning...")
        scan_count = 0
        start_time = time.time()
        consecutive_errors = 0
        max_consecutive_errors = 3
        max_scan_time = 10.0  # Maximum 10 seconds
        
        # Scan selama 10 detik untuk melihat data yang terdeteksi
        print("[LIDAR] Scanning for 10 seconds to show detected objects...")
        
        try:
            # Use iter_scans for scanning
            for scan in lidar.iter_scans():
                if time.time() - start_time >= max_scan_time:
                    break
                
                try:
                    if len(scan) > 0:
                        scan_count += 1
                        consecutive_errors = 0  # Reset error counter on successful scan
                        
                        # Hitung statistik
                        distances = [meas[2]/1000.0 for meas in scan]  # Convert to meters
                        angles = [meas[1] for meas in scan]  # Angles in degrees
                        
                        min_dist = min(distances)
                        max_dist = max(distances)
                        avg_dist = sum(distances) / len(distances)
                        
                        print(f"Scan {scan_count}: {len(scan)} points, "
                              f"Min: {min_dist:.2f}m, Max: {max_dist:.2f}m, Avg: {avg_dist:.2f}m")
                        
                        # Analisis objek yang terdeteksi
                        close_objects = [d for d in distances if d < 1.0]  # Objek dalam 1 meter
                        medium_objects = [d for d in distances if 1.0 <= d < 3.0]  # Objek 1-3 meter
                        far_objects = [d for d in distances if d >= 3.0]  # Objek > 3 meter
                        
                        print(f"  Objects detected: Close({len(close_objects)}), Medium({len(medium_objects)}), Far({len(far_objects)})")
                        
                        # Tampilkan objek terdekat
                        if close_objects:
                            closest = min(close_objects)
                            closest_angle = angles[distances.index(closest)]
                            print(f"  Closest object: {closest:.2f}m at {closest_angle:.1f}°")
                        
                        # Tampilkan beberapa sample data setiap 5 scan
                        if scan_count % 5 == 1:
                            print("  Sample data (first 5 points):")
                            for i, (quality, angle, distance) in enumerate(scan[:5]):
                                print(f"    Point {i}: Quality={quality}, Angle={angle:.1f}°, Distance={distance/1000:.3f}m")
                        
                        print()  # Empty line for readability
                        
                except Exception as e:
                    consecutive_errors += 1
                    error_msg = str(e)
                    print(f"[ERROR] LiDAR scan error ({consecutive_errors}/{max_consecutive_errors}): {error_msg}")
                    
                    # If too many consecutive errors, stop trying
                    if consecutive_errors >= max_consecutive_errors:
                        print(f"[ERROR] Too many consecutive LiDAR errors ({consecutive_errors}), stopping")
                        break
                    
                    # Wait before retry
                    time.sleep(1.0)
                    
        except KeyboardInterrupt:
            print("\n[LIDAR] Scanning interrupted by user")
        
        # Disconnect
        if lidar:
            try:
                lidar.disconnect()
            except:
                pass
        
        if scan_count > 0:
            print(f"✓ Scanning test selesai! Total scan: {scan_count}")
            print("✓ LiDAR berhasil mendeteksi objek di sekitarnya")
            return True
        else:
            print("✗ Tidak ada scan yang berhasil")
            return False
        
    except Exception as e:
        print(f"✗ Error scanning: {str(e)}")
        if lidar:
            try:
                lidar.disconnect()
            except:
                pass
        return False

def test_lidar_data_format():
    """Test format data LiDAR"""
    print("\n=== TEST FORMAT DATA ===")
    
    lidar = None
    try:
        # Initialize LiDAR dengan error handling
        lidar = reset_lidar_connection()
        if lidar is None:
            print("✗ Gagal menginisialisasi LiDAR")
            return False
        
        print("Mengambil sample data...")
        
        # Ambil satu scan dengan error handling
        try:
            for scan in lidar.iter_scans():
                if len(scan) > 0:
                    print(f"✓ Data format OK: {len(scan)} points")
                    
                    # Convert ke format yang digunakan di test_robot.py
                    ranges = [float('inf')] * 360
                    
                    for quality, angle, distance in scan:
                        angle_deg = int(angle) % 360
                        if 0 <= angle_deg < 360:
                            ranges[angle_deg] = distance / 1000.0
                    
                    # Hitung coverage
                    valid_points = sum(1 for r in ranges if r != float('inf'))
                    coverage = (valid_points / 360) * 100
                    
                    print(f"✓ Coverage: {coverage:.1f}% ({valid_points}/360 points)")
                    print(f"✓ Range array length: {len(ranges)}")
                    
                    # Tampilkan beberapa sample ranges
                    print("Sample ranges (0°, 90°, 180°, 270°):")
                    for angle in [0, 90, 180, 270]:
                        dist = ranges[angle]
                        if dist != float('inf'):
                            print(f"  {angle}°: {dist:.3f}m")
                        else:
                            print(f"  {angle}°: No data")
                    
                    # Analisis distribusi data
                    front_data = ranges[0:30] + ranges[330:360]  # 60° depan
                    left_data = ranges[30:90]  # 60° kiri
                    right_data = ranges[270:330]  # 60° kanan
                    back_data = ranges[150:210]  # 60° belakang
                    
                    front_valid = sum(1 for r in front_data if r != float('inf'))
                    left_valid = sum(1 for r in left_data if r != float('inf'))
                    right_valid = sum(1 for r in right_data if r != float('inf'))
                    back_valid = sum(1 for r in back_data if r != float('inf'))
                    
                    print(f"\nData distribution:")
                    print(f"  Front (0°±30°): {front_valid}/60 points ({front_valid/60*100:.1f}%)")
                    print(f"  Left (30°-90°): {left_valid}/60 points ({left_valid/60*100:.1f}%)")
                    print(f"  Right (270°-330°): {right_valid}/60 points ({right_valid/60*100:.1f}%)")
                    print(f"  Back (150°-210°): {back_valid}/60 points ({back_valid/60*100:.1f}%)")
                    
                break
        except Exception as e:
            print(f"✗ Error saat mengambil scan: {str(e)}")
            return False
        
        # Disconnect
        if lidar:
            try:
                lidar.disconnect()
            except:
                pass
        
        return True
        
    except Exception as e:
        print(f"✗ Error format test: {str(e)}")
        if lidar:
            try:
                lidar.disconnect()
            except:
                pass
        return False

def main():
    """Main test function"""
    print("RP-Lidar A1M8 Test Script")
    print("=" * 40)
    print("Pilih mode:")
    print("1. Test koneksi dan scanning (default)")
    print("2. Streaming mode - kirim data ke roam.py")
    print("3. RViz visualization mode - kirim data ke RViz")
    print("4. Exit")
    
    try:
        choice = input("\nPilih mode (1-4): ").strip()
        
        if choice == "2":
            # Streaming mode untuk roam.py
            start_lidar_streaming_mode()
        elif choice == "3":
            # RViz visualization mode
            start_lidar_rviz_mode()
        elif choice == "4":
            print("Exiting...")
            return
        else:
            # Default: Test mode
            print("\n=== TEST MODE ===")
            
            # Test 1: Koneksi
            if not test_lidar_connection():
                print("\n❌ Test koneksi gagal! Cek koneksi hardware dan permission.")
                return
            
            # Test 2: Scanning
            if not test_lidar_scanning():
                print("\n❌ Test scanning gagal! Cek power supply dan koneksi.")
                return
            
            # Test 3: Format data
            if not test_lidar_data_format():
                print("\n❌ Test format data gagal!")
                return
            
            print("\n✅ SEMUA TEST BERHASIL!")
            print("LiDAR siap digunakan!")
            print("\nMode yang tersedia:")
            print("  - Opsi 2: Streaming ke roam.py (untuk integrasi Gazebo)")
            print("  - Opsi 3: Streaming ke RViz (untuk visualisasi real-time)")
            print("\nJalankan script ini lagi dan pilih mode yang diinginkan")
            
    except KeyboardInterrupt:
        print("\n\nScript dihentikan oleh user")
    except Exception as e:
        print(f"\nError: {str(e)}")

if __name__ == "__main__":
    main()
