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
MQTT_BROKER = "codex.petra.ac.id"
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
           self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
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
           # Convert scan data to format expected by roam.py
           ranges = [float('inf')] * 360
          
           for quality, angle, distance in scan_data:
               angle_deg = int(angle) % 360
               if 0 <= angle_deg < 360:
                   ranges[angle_deg] = distance / 1000.0  # Convert to meters
          
           # Create data packet
           lidar_data = {
               "timestamp": time.time(),
               "ranges": ranges,
               "angle_min": -math.pi,
               "angle_max": math.pi,
               "angle_increment": 2 * math.pi / 360,
               "range_min": 0.15,
               "range_max": 12.0,
               "source": "real_world_lidar"
           }
          
           # Send via MQTT
           self.mqtt_client.publish(MQTT_TOPIC_LIDAR_DATA, json.dumps(lidar_data))
          
       except Exception as e:
           print(f"[ERROR] Failed to send LiDAR data: {str(e)}")
  
   def start_lidar_streaming(self):
       """Start streaming LiDAR data to roam.py"""
       if self.is_running:
           print("[LIDAR] Already streaming")
           return
      
       try:
           # Initialize LiDAR
           print(f"[LIDAR] Initializing LiDAR on {LIDAR_PORT}...")
           self.lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE, timeout=LIDAR_TIMEOUT)
          
           # Get LiDAR info
           info = self.lidar.get_info()
           print(f"[LIDAR] Connected: {info}")
          
           # Get health status
           health = self.lidar.get_health()
           print(f"[LIDAR] Health: {health}")
          
           self.is_running = True
           self.lidar_thread = threading.Thread(target=self._lidar_streaming_loop)
           self.lidar_thread.daemon = True
           self.lidar_thread.start()
          
           print("[LIDAR] Started streaming real-world data to roam.py")
           print("[LIDAR] Press Ctrl+C to stop")
          
       except Exception as e:
           print(f"[ERROR] Failed to start LiDAR streaming: {str(e)}")
           self.is_running = False
  
   def _lidar_streaming_loop(self):
       """Main LiDAR streaming loop"""
       scan_count = 0
       start_time = time.time()
      
       try:
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


def start_lidar_streaming_mode():
   """Start LiDAR streaming mode untuk mengirim data ke roam.py"""
   print("=== LIDAR STREAMING MODE ===")
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
          
           # Get health status
           health = lidar.get_health()
           print(f"[LIDAR] Health status: {health}")
          
           # Test basic functionality - langsung coba scan tanpa start()
           print("[LIDAR] Testing scan functionality...")
           scan_count = 0
           max_test_scans = 3
          
           for scan in lidar.iter_scans():
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
          
           print("[LIDAR] LiDAR initialization successful!")
           return lidar
          
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
           # Use timeout for scanning
           scan_iterator = lidar.iter_scans()
           while time.time() - start_time < max_scan_time:
               try:
                   # Get next scan with timeout
                   scan = next(scan_iterator)
                   scan_count += 1
                   consecutive_errors = 0  # Reset error counter on successful scan
                  
                   # Process scan data
                   if len(scan) > 0:
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
                  
               except StopIteration:
                   print("[LIDAR] Scan iterator finished")
                   break
               except Exception as e:
                   consecutive_errors += 1
                   error_msg = str(e)
                   print(f"[ERROR] LiDAR scan error ({consecutive_errors}/{max_consecutive_errors}): {error_msg}")
                  
                   # Handle specific errors
                   if "Wrong body size" in error_msg:
                       print("[LIDAR] Wrong body size error detected, attempting to reset LiDAR...")
                       try:
                           # Reset LiDAR connection
                           lidar.disconnect()
                           time.sleep(1.0)
                          
                           # Reconnect
                           lidar = reset_lidar_connection()
                           if lidar is None:
                               print("[ERROR] Failed to reset LiDAR")
                               break
                          
                           print("[LIDAR] LiDAR reset and restarted successfully")
                           scan_iterator = lidar.iter_scans()
                           continue
                       except Exception as reset_error:
                           print(f"[ERROR] Failed to reset LiDAR: {str(reset_error)}")
                  
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
   print("3. Exit")
  
   try:
       choice = input("\nPilih mode (1-3): ").strip()
      
       if choice == "2":
           # Streaming mode
           start_lidar_streaming_mode()
       elif choice == "3":
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
           print("LiDAR siap digunakan dengan test_robot.py")
           print("\nUntuk streaming mode, jalankan script ini lagi dan pilih opsi 2")
          
   except KeyboardInterrupt:
       print("\n\nScript dihentikan oleh user")
   except Exception as e:
       print(f"\nError: {str(e)}")


if __name__ == "__main__":
   main()
