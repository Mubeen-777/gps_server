#!/usr/bin/env python3
"""
ADAS Accident Detection System for Termux
Optimized for lowest latency & highest accuracy
Author: ADAS Project
"""

import subprocess
import json
import time
import threading
import numpy as np
from datetime import datetime
from collections import deque
import signal
import sys

class KalmanFilter:
    """Simple 1D Kalman filter for sensor fusion"""
    def __init__(self, process_variance=0.1, measurement_variance=5.0):
        self.process_variance = process_variance  # Q
        self.measurement_variance = measurement_variance  # R
        self.estimate = 0.0
        self.error_covariance = 1.0  # P
    
    def update(self, measurement, control_input=0.0, dt=0.2):
        """Update filter with new measurement"""
        # Prediction step
        self.estimate = self.estimate + control_input * dt
        self.error_covariance += self.process_variance
        
        # Update step
        kalman_gain = self.error_covariance / (self.error_covariance + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.error_covariance = (1 - kalman_gain) * self.error_covariance
        
        return self.estimate

class ADASSystem:
    """Main ADAS system with sensor fusion and accident detection"""
    
    # Detection thresholds
    HARD_BRAKE_THRESHOLD = -3.5  # m/s²
    RAPID_ACCEL_THRESHOLD = 3.0  # m/s²
    CRASH_THRESHOLD = 78.4  # 8g in m/s²
    JERK_THRESHOLD = 50.0  # m/s³
    
    def __init__(self, log_file="adas_log.csv"):
        # Sensor data storage
        self.accel_data = {'x': 0, 'y': 0, 'z': 0, 'magnitude': 0, 'timestamp': 0}
        self.gyro_data = {'x': 0, 'y': 0, 'z': 0, 'timestamp': 0}
        self.gps_data = {'lat': 0, 'lon': 0, 'speed': 0, 'accuracy': 0, 'timestamp': 0}
        
        # Kalman filter for speed fusion
        self.kalman_speed = KalmanFilter(process_variance=0.1, measurement_variance=5.0)
        
        # History for jerk calculation
        self.accel_history = deque(maxlen=10)
        self.prev_accel = 0
        self.prev_time = 0
        
        # Event detection
        self.events = deque(maxlen=100)
        
        # Control flags
        self.running = False
        self.threads = []
        
        # Logging
        self.log_file = log_file
        self.init_logging()
        
        # Statistics
        self.start_time = time.time()
        self.gps_updates = 0
        self.accel_updates = 0
    
    def init_logging(self):
        """Initialize CSV logging"""
        try:
            with open(self.log_file, 'w') as f:
                f.write("timestamp,lat,lon,gps_speed,kalman_speed,accel_x,accel_y,accel_z,"
                       "accel_mag,gyro_x,gyro_y,gyro_z,jerk,event\n")
            print(f"[✓] Logging to: {self.log_file}")
        except Exception as e:
            print(f"[✗] Failed to initialize logging: {e}")
    
    def log_data(self, event=""):
        """Write data to CSV"""
        try:
            timestamp = int(time.time() * 1000)
            jerk = self.calculate_jerk()
            
            with open(self.log_file, 'a') as f:
                f.write(f"{timestamp},{self.gps_data['lat']},{self.gps_data['lon']},"
                       f"{self.gps_data['speed']},{self.kalman_speed.estimate},"
                       f"{self.accel_data['x']},{self.accel_data['y']},{self.accel_data['z']},"
                       f"{self.accel_data['magnitude']},"
                       f"{self.gyro_data['x']},{self.gyro_data['y']},{self.gyro_data['z']},"
                       f"{jerk},{event}\n")
        except Exception as e:
            print(f"[✗] Logging error: {e}")
    
    def read_gps(self):
        """Read GPS data using termux-location (FusedLocationProvider)"""
        while self.running:
            try:
                # Call termux-location with high accuracy
                result = subprocess.run(
                    ['termux-location', '-p', 'gps', '-r', 'last'],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                
                if result.returncode == 0:
                    data = json.loads(result.stdout)
                    self.gps_data = {
                        'lat': data.get('latitude', 0),
                        'lon': data.get('longitude', 0),
                        'speed': data.get('speed', 0),  # m/s
                        'accuracy': data.get('accuracy', 0),
                        'timestamp': time.time()
                    }
                    self.gps_updates += 1
                    
                    # Update Kalman filter
                    accel_mag = self.accel_data['magnitude']
                    fused_speed = self.kalman_speed.update(
                        self.gps_data['speed'], 
                        accel_mag, 
                        dt=0.2
                    )
                    
                    # Log data
                    self.log_data()
                    
            except Exception as e:
                print(f"[!] GPS error: {e}")
            
            time.sleep(0.2)  # 5 Hz update rate
    
    def read_accelerometer(self):
        """Read accelerometer data (LINEAR_ACCELERATION equivalent)"""
        while self.running:
            try:
                # Read accelerometer
                result = subprocess.run(
                    ['termux-sensor', '-s', 'BMI160 Accelerometer', '-n', '1', '-d', '10'],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                
                if result.returncode == 0:
                    data = json.loads(result.stdout)
                    sensor_data = data.get('BMI160 Accelerometer', {}).get('values', [0, 0, 0])
                    
                    # Remove gravity (approximate - proper would use sensor fusion)
                    # Note: termux-sensor gives total acceleration, we need linear
                    self.accel_data = {
                        'x': sensor_data[0],
                        'y': sensor_data[1],
                        'z': sensor_data[2] - 9.8,  # Remove gravity from Z
                        'magnitude': 0,
                        'timestamp': time.time()
                    }
                    
                    # Calculate magnitude
                    self.accel_data['magnitude'] = np.sqrt(
                        self.accel_data['x']**2 + 
                        self.accel_data['y']**2 + 
                        self.accel_data['z']**2
                    )
                    
                    self.accel_updates += 1
                    
                    # Add to history for jerk calculation
                    self.accel_history.append({
                        'value': self.accel_data['magnitude'],
                        'time': self.accel_data['timestamp']
                    })
                    
                    # Detect events
                    self.detect_events()
                    
            except Exception as e:
                print(f"[!] Accelerometer error: {e}")
            
            time.sleep(0.01)  # ~100 Hz attempt (termux limitation ~10-50 Hz realistic)
    
    def read_gyroscope(self):
        """Read gyroscope data"""
        while self.running:
            try:
                result = subprocess.run(
                    ['termux-sensor', '-s', 'BMI160 Gyroscope Uncalibrated', '-n', '1', '-d', '10'],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                
                if result.returncode == 0:
                    data = json.loads(result.stdout)
                    sensor_data = data.get('BMI160 Gyroscope Uncalibrated', {}).get('values', [0, 0, 0])
                    
                    self.gyro_data = {
                        'x': sensor_data[0],
                        'y': sensor_data[1],
                        'z': sensor_data[2],
                        'timestamp': time.time()
                    }
                    
            except Exception as e:
                print(f"[!] Gyroscope error: {e}")
            
            time.sleep(0.01)
    
    def calculate_jerk(self):
        """Calculate jerk (rate of change of acceleration)"""
        if len(self.accel_history) < 2:
            return 0
        
        recent = self.accel_history[-1]
        previous = self.accel_history[-2]
        
        dt = recent['time'] - previous['time']
        if dt <= 0:
            return 0
        
        jerk = (recent['value'] - previous['value']) / dt
        return jerk
    
    def detect_events(self):
        """Detect driving events and accidents"""
        accel = self.accel_data['magnitude']
        accel_y = self.accel_data['y']  # Longitudinal
        jerk = self.calculate_jerk()
        
        # Hard braking
        if accel_y < self.HARD_BRAKE_THRESHOLD:
            self.on_hard_brake(accel_y)
        
        # Rapid acceleration
        if accel_y > self.RAPID_ACCEL_THRESHOLD:
            self.on_rapid_acceleration(accel_y)
        
        # Crash detection (high G-force)
        if accel > self.CRASH_THRESHOLD:
            self.on_crash(accel)
        
        # Impact detection via jerk (most reliable)
        if abs(jerk) > self.JERK_THRESHOLD:
            self.on_impact(jerk)
    
    def on_hard_brake(self, accel):
        """Handle hard braking event"""
        event = f"HARD BRAKE: {accel:.2f} m/s²"
        self.add_event(event, severity="WARNING")
        self.log_data(event)
    
    def on_rapid_acceleration(self, accel):
        """Handle rapid acceleration"""
        event = f"RAPID ACCEL: {accel:.2f} m/s²"
        self.add_event(event, severity="INFO")
    
    def on_crash(self, accel):
        """Handle crash detection"""
        g_force = accel / 9.8
        event = f"⚠️  CRASH DETECTED: {g_force:.1f}g"
        self.add_event(event, severity="CRITICAL")
        self.log_data(event)
        
        # TODO: Send emergency alert
        print(f"\n{'='*60}")
        print(f"[CRITICAL] {event}")
        print(f"Location: {self.gps_data['lat']}, {self.gps_data['lon']}")
        print(f"{'='*60}\n")
    
    def on_impact(self, jerk):
        """Handle impact detection via jerk"""
        event = f"⚠️  IMPACT (Jerk): {jerk:.1f} m/s³"
        self.add_event(event, severity="CRITICAL")
        self.log_data(event)
    
    def add_event(self, event, severity="INFO"):
        """Add event to log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] [{severity}] {event}"
        self.events.append(log_entry)
        print(log_entry)
    
    def print_status(self):
        """Print current status (called periodically)"""
        while self.running:
            runtime = time.time() - self.start_time
            speed_kmh = self.kalman_speed.estimate * 3.6
            
            print(f"\n{'='*70}")
            print(f"ADAS Status - Runtime: {runtime:.1f}s")
            print(f"{'='*70}")
            print(f"Speed:        {speed_kmh:6.1f} km/h (GPS: {self.gps_data['speed']*3.6:.1f} km/h)")
            print(f"Acceleration: {self.accel_data['magnitude']:6.2f} m/s²")
            print(f"Location:     {self.gps_data['lat']:.6f}, {self.gps_data['lon']:.6f}")
            print(f"GPS Updates:  {self.gps_updates} ({self.gps_updates/runtime:.1f} Hz)")
            print(f"Accel Updates: {self.accel_updates} ({self.accel_updates/runtime:.1f} Hz)")
            print(f"{'='*70}")
            
            # Show recent events
            if self.events:
                print("Recent Events:")
                for event in list(self.events)[-5:]:
                    print(f"  {event}")
            
            time.sleep(2)  # Update every 2 seconds
    
    def start(self):
        """Start ADAS system"""
        print("\n" + "="*70)
        print("  ADAS Accident Detection System")
        print("  Optimized for Termux on Redmi Note 12")
        print("="*70 + "\n")
        
        # Check termux-api availability
        if not self.check_termux_api():
            return
        
        self.running = True
        self.start_time = time.time()
        
        # Start sensor threads
        gps_thread = threading.Thread(target=self.read_gps, daemon=True)
        accel_thread = threading.Thread(target=self.read_accelerometer, daemon=True)
        gyro_thread = threading.Thread(target=self.read_gyroscope, daemon=True)
        status_thread = threading.Thread(target=self.print_status, daemon=True)
        
        self.threads = [gps_thread, accel_thread, gyro_thread, status_thread]
        
        for thread in self.threads:
            thread.start()
        
        print("[✓] ADAS system started")
        print("[✓] Press Ctrl+C to stop\n")
        
        # Keep main thread alive
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            self.stop()
    
    def check_termux_api(self):
        """Check if termux-api is installed"""
        try:
            result = subprocess.run(['which', 'termux-location'], capture_output=True)
            if result.returncode != 0:
                print("[✗] termux-api not found!")
                print("    Install with: pkg install termux-api")
                print("    Also install Termux:API app from F-Droid or Google Play")
                return False
            print("[✓] termux-api detected")
            return True
        except Exception as e:
            print(f"[✗] Error checking termux-api: {e}")
            return False
    
    def stop(self):
        """Stop ADAS system"""
        print("\n[!] Stopping ADAS system...")
        self.running = False
        time.sleep(1)
        
        print(f"[✓] Total GPS updates: {self.gps_updates}")
        print(f"[✓] Total Accel updates: {self.accel_updates}")
        print(f"[✓] Log saved to: {self.log_file}")
        print("[✓] ADAS system stopped\n")

def main():
    """Main entry point"""
    adas = ADASSystem(log_file=f"adas_log_{int(time.time())}.csv")
    
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        adas.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Start system
    adas.start()

if __name__ == "__main__":
    main()
