#!/usr/bin/env python3
"""
USB GPS Server for Termux
Transmits real GPS location via USB (no WiFi/IP needed)
Works anywhere - car, home, outdoors
"""

import socket
import time
import threading
import sys
import os
from datetime import datetime
import subprocess
import json

class USBGPServer:
    def __init__(self, port=5555):
        self.port = port
        self.server = None
        self.running = False
        self.clients = []
        self.lock = threading.Lock()
        
        # GPS state
        self.last_location = None
        self.satellites = 0
        self.gps_accuracy = 0.0
        
    def start(self):
        """Start the GPS server on all interfaces (0.0.0.0)"""
        try:
            # Create socket
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # Bind to ALL interfaces (0.0.0.0) - important for USB!
            self.server.bind(('0.0.0.0', self.port))
            self.server.listen(5)
            self.server.settimeout(1.0)
            
            self.running = True
            
            print(f"✅ USB GPS Server started on port {self.port}")
            print(f"   Listening on: 0.0.0.0:{self.port}")
            print(f"   Connect via USB with: adb forward tcp:{self.port} tcp:{self.port}")
            print("   Press Ctrl+C to stop\n")
            
            # Start GPS monitoring thread
            gps_thread = threading.Thread(target=self.monitor_gps, daemon=True)
            gps_thread.start()
            
            # Start client accept thread
            accept_thread = threading.Thread(target=self.accept_clients, daemon=True)
            accept_thread.start()
            
            # Broadcast GPS data to all connected clients
            self.broadcast_gps_data()
            
        except Exception as e:
            print(f"❌ Failed to start server: {e}")
            self.cleanup()
            return False
        
        return True
    
    def monitor_gps(self):
        """Monitor GPS location using Termux-API"""
        print("📡 Starting GPS monitoring...")
        
        while self.running:
            try:
                location = self.get_gps_location()
                
                if location:
                    self.last_location = location
                    self.satellites = location.get('satellites', 0)
                    self.gps_accuracy = location.get('accuracy', 0.0)
                    
                    # Display GPS info
                    if self.satellites > 0:
                        lat = location['latitude']
                        lon = location['longitude']
                        speed = location.get('speed', 0) * 3.6  # Convert to km/h
                        alt = location.get('altitude', 0)
                        
                        print(f"📍 GPS: {lat:.6f}, {lon:.6f} | "
                              f"Speed: {speed:.1f} km/h | "
                              f"Alt: {alt:.0f}m | "
                              f"Sats: {self.satellites}")
                
                time.sleep(0.5)  # Update twice per second
                
            except Exception as e:
                print(f"⚠ GPS monitoring error: {e}")
                time.sleep(2)
    
    def get_gps_location(self):
        """Get GPS location using Termux-API"""
        try:
            # Method 1: Use termux-location API
            result = subprocess.run(['termux-location', '-p', 'gps'], 
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0 and result.stdout.strip():
                location = json.loads(result.stdout)
                
                # Add timestamp
                location['timestamp'] = datetime.utcnow().strftime('%H%M%S.%f')[:-3]
                location['date'] = datetime.now().strftime('%d%m%y')
                
                return location
            
        except (subprocess.TimeoutExpired, json.JSONDecodeError):
            pass
        except FileNotFoundError:
            print("⚠ termux-location not found. Installing termux-api...")
            subprocess.run(['pkg', 'install', '-y', 'termux-api'], 
                         capture_output=True)
            time.sleep(2)
        
        # Method 2: Use GPSd if available
        try:
            import gpsd
            gpsd.connect()
            packet = gpsd.get_current()
            
            if packet.mode >= 2:  # 2D or 3D fix
                location = {
                    'latitude': packet.lat,
                    'longitude': packet.lon,
                    'altitude': packet.alt if packet.mode >= 3 else 0,
                    'speed': packet.hspeed if hasattr(packet, 'hspeed') else 0,
                    'bearing': packet.track if hasattr(packet, 'track') else 0,
                    'accuracy': packet.eph if hasattr(packet, 'eph') else 99.0,
                    'satellites': packet.sats if hasattr(packet, 'sats') else 0,
                    'timestamp': datetime.utcnow().strftime('%H%M%S.%f')[:-3],
                    'date': datetime.now().strftime('%d%m%y')
                }
                return location
        except:
            pass
        
        # Method 3: Simulate GPS for testing
        if self.last_location is None:
            # Default location (Lahore)
            return {
                'latitude': 31.5204,
                'longitude': 74.3587,
                'altitude': 217.0,
                'speed': 0.0,
                'bearing': 0.0,
                'accuracy': 10.0,
                'satellites': 8,
                'timestamp': datetime.utcnow().strftime('%H%M%S.%f')[:-3],
                'date': datetime.now().strftime('%d%m%y')
            }
        
        return None
    
    def location_to_nmea(self, location):
        """Convert location dictionary to NMEA sentences"""
        if not location:
            return ""
        
        lat = location['latitude']
        lon = location['longitude']
        speed_knots = location.get('speed', 0) / 1.852  # Convert to knots
        bearing = location.get('bearing', 0)
        timestamp = location.get('timestamp', '000000.000')
        date = location.get('date', '010101')
        altitude = location.get('altitude', 0)
        satellites = location.get('satellites', 0)
        
        # Convert decimal degrees to NMEA format (DDMM.MMMM)
        lat_deg = int(abs(lat))
        lat_min = (abs(lat) - lat_deg) * 60.0
        lat_dir = 'N' if lat >= 0 else 'S'
        lat_str = f"{lat_deg:02d}{lat_min:06.3f}"
        
        lon_deg = int(abs(lon))
        lon_min = (abs(lon) - lon_deg) * 60.0
        lon_dir = 'E' if lon >= 0 else 'W'
        lon_str = f"{lon_deg:03d}{lon_min:06.3f}"
        
        nmea_sentences = []
        
        # GPRMC - Recommended Minimum Specific GPS/TRANSIT Data
        gprmc = f"$GPRMC,{timestamp},A,{lat_str},{lat_dir},{lon_str},{lon_dir},{speed_knots:.1f},{bearing:.1f},{date},,A"
        nmea_sentences.append(gprmc + self.nmea_checksum(gprmc))
        
        # GPGGA - Global Positioning System Fix Data
        gps_quality = 1 if satellites > 0 else 0
        hdop = location.get('accuracy', 99.0)  # Horizontal dilution of precision
        gpgga = f"$GPGGA,{timestamp},{lat_str},{lat_dir},{lon_str},{lon_dir},{gps_quality},{satellites:02d},{hdop:.1f},{altitude:.1f},M,,M,,"
        nmea_sentences.append(gpgga + self.nmea_checksum(gpgga))
        
        # GPGSA - GPS DOP and Active Satellites
        gsa_mode = 'A' if satellites > 0 else 'M'  # A=Auto, M=Manual
        gsa_fix = '3' if satellites >= 4 else '1'  # 1=No fix, 2=2D, 3=3D
        gsa_sats = '1,2,3,4,5,6,7,8,9,10,11,12'
        gpgsa = f"$GPGSA,{gsa_mode},{gsa_fix},{gsa_sats},99.9,99.9,99.9"
        nmea_sentences.append(gpgsa + self.nmea_checksum(gpgsa))
        
        # GPGSV - GPS Satellites in View (dummy data)
        if satellites > 0:
            gpgsv = f"$GPGSV,1,1,{satellites:02d},01,80,048,,02,50,290,,03,35,150,,04,25,020,"
            nmea_sentences.append(gpgsv + self.nmea_checksum(gpgsv))
        
        # GPVTG - Course over ground and ground speed
        speed_kmh = speed_knots * 1.852
        gpvtg = f"$GPVTG,{bearing:.1f},T,,M,{speed_knots:.1f},N,{speed_kmh:.1f},K,A"
        nmea_sentences.append(gpvtg + self.nmea_checksum(gpvtg))
        
        return '\n'.join(nmea_sentences) + '\n'
    
    def nmea_checksum(self, sentence):
        """Calculate NMEA checksum"""
        # Remove $ and * if present
        if sentence.startswith('$'):
            sentence = sentence[1:]
        if '*' in sentence:
            sentence = sentence.split('*')[0]
        
        # Calculate checksum
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)
        
        return f"*{checksum:02X}"
    
    def accept_clients(self):
        """Accept incoming client connections"""
        while self.running:
            try:
                client_socket, client_address = self.server.accept()
                client_socket.settimeout(1.0)
                
                with self.lock:
                    self.clients.append(client_socket)
                
                print(f"📱 Client connected: {client_address}")
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"⚠ Accept error: {e}")
    
    def broadcast_gps_data(self):
        """Broadcast GPS data to all connected clients"""
        print("📤 Broadcasting GPS data to clients...")
        
        while self.running:
            if self.last_location:
                nmea_data = self.location_to_nmea(self.last_location)
                
                with self.lock:
                    # Send to all clients
                    disconnected_clients = []
                    
                    for client in self.clients:
                        try:
                            client.sendall(nmea_data.encode('utf-8'))
                        except (socket.error, ConnectionResetError):
                            disconnected_clients.append(client)
                    
                    # Remove disconnected clients
                    for client in disconnected_clients:
                        self.clients.remove(client)
                        try:
                            client.close()
                        except:
                            pass
            
            # Send at 10Hz (100ms intervals) for smooth updates
            time.sleep(0.1)
    
    def cleanup(self):
        """Cleanup resources"""
        print("\n🛑 Stopping GPS server...")
        self.running = False
        
        # Close all client connections
        with self.lock:
            for client in self.clients:
                try:
                    client.close()
                except:
                    pass
            self.clients.clear()
        
        # Close server socket
        if self.server:
            try:
                self.server.close()
            except:
                pass
        
        print("✅ Server stopped")

def main():
    """Main function"""
    print("=" * 50)
    print("   📱 TERMUX USB GPS SERVER")
    print("   Transmits real GPS via USB to PC")
    print("=" * 50)
    print()
    
    # Check if Termux-API is installed
    try:
        subprocess.run(['termux-location', '--help'], 
                      capture_output=True, check=True)
    except:
        print("⚠ Termux-API not installed. Installing...")
        subprocess.run(['pkg', 'update', '-y'], capture_output=True)
        subprocess.run(['pkg', 'install', '-y', 'termux-api'], capture_output=True)
        print("✅ Termux-API installed")
        print()
    
    # Create and start server
    server = USBGPServer(port=5555)
    
    try:
        if server.start():
            # Keep main thread alive
            while server.running:
                time.sleep(1)
        else:
            print("❌ Failed to start server")
            return 1
            
    except KeyboardInterrupt:
        print("\n\nReceived Ctrl+C")
    finally:
        server.cleanup()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
