import sys
import socket
import threading
import time
import requests
import math
from concurrent.futures import ThreadPoolExecutor, as_completed
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout, QLineEdit, QPushButton
from PySide6.QtCore import QTimer, Signal, QObject, Qt, QPoint, QPointF
from PySide6.QtGui import QPainter, QPixmap, QFont, QWheelEvent, QMouseEvent, QPen, QBrush, QColor, QPolygonF, QPainterPath, QIcon
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink
import tempfile
import os

class TelemetryReceiver(QObject):
    """Handles UDP reception and MAVLink parsing"""
    
    # Signals to update UI
    telemetry_updated = Signal(dict)
    connection_changed = Signal(bool)
    
    def __init__(self, port=14551):
        super().__init__()
        self.port = port
        self.socket = None
        self.running = False
        self.last_heartbeat = 0
        
        # Current telemetry data
        self.telemetry_data = {
            'connected': False,
            'flight_mode': 'Unknown',
            'armed': False,
            'gps_lat': 0.0,
            'gps_lon': 0.0,
            'altitude': 0.0,
            'ground_speed': 0.0,
            'heading': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'battery_voltage': 0.0,
            'gps_satellites': 0
        }
        
        # Armed status stability variables
        self.armed_history = []
        self.max_history = 5
        self.last_stable_armed = False

        self.initial_battery = 24.9
        self.battery_start_time = time.time()
        self.battery_drain_rate = 1.0  # 1 volt per minute (much faster)
        
    def start_receiving(self):
        """Start UDP reception in separate thread"""
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop)
        self.thread.daemon = True
        self.thread.start()
        
        # Start connection monitor timer
        self.connection_timer = QTimer()
        self.connection_timer.timeout.connect(self._check_connection)
        self.connection_timer.start(1000)
        
    def stop_receiving(self):
        """Stop UDP reception"""
        self.running = False
        if self.socket:
            self.socket.close()
            
    def _receive_loop(self):
        """Main UDP reception loop"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind(('127.0.0.1', self.port))
            self.socket.settimeout(1.0)
            print(f"UDP listener started on port {self.port}")
            
            while self.running:
                try:
                    data, addr = self.socket.recvfrom(1024)
                    current_time = time.time()
                    self.last_heartbeat = current_time
                    if not self.telemetry_data['connected']:
                        print("Data received - setting connected")
                        self.telemetry_data['connected'] = True
                        self.connection_changed.emit(True)
                    
                    self._parse_mavlink(data)
                except socket.timeout:
                    current_time = time.time()
                    if current_time - self.last_heartbeat > 3:
                        if self.telemetry_data['connected']:
                            print("No data timeout - setting disconnected")
                            self.telemetry_data['connected'] = False
                            self.connection_changed.emit(False)
                    continue
                except Exception as e:
                    print(f"UDP receive error: {e}")
                    
        except Exception as e:
            print(f"Socket setup error: {e}")
            
    def _parse_mavlink(self, data):
        """Parse MAVLink messages"""
        try:
            print(f"Received {len(data)} bytes: {data[:20].hex()}")
            
            if not hasattr(self, 'mav_parser'):
                self.mav_parser = mavutil.mavlink.MAVLink(None)
            
            for byte in data:
                msg = self.mav_parser.parse_char(bytes([byte]))
                if msg:
                    print(f"Parsed message: {msg.get_type()}")
                    self._process_message(msg)
                    
        except Exception as e:
            print(f"MAVLink parse error: {e}")
            import traceback
            traceback.print_exc()
            
    def _process_message(self, msg):
        """Process individual MAVLink messages"""
        current_time = time.time()
        
        self.last_heartbeat = current_time
        self.telemetry_data['connected'] = True
        
        if msg.get_type() == 'HEARTBEAT':
            self.telemetry_data['flight_mode'] = "MISSION"
            
            # Get new armed state
            new_armed_state = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            
            # Add to history
            self.armed_history.append(new_armed_state)
            if len(self.armed_history) > self.max_history:
                self.armed_history.pop(0)
            
            # Only update if we have enough readings and they're consistent
            if len(self.armed_history) >= 2:
                # Check if last 2 readings are the same
                if all(x == new_armed_state for x in self.armed_history[-2:]):
                    if self.last_stable_armed != new_armed_state:
                        self.telemetry_data['armed'] = new_armed_state
                        self.last_stable_armed = new_armed_state
                        print(f"Armed status changed to: {new_armed_state}")
                    
        elif msg.get_type() == 'GLOBAL_POSITION_INT':
            raw_lat = msg.lat
            raw_lon = msg.lon
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0
            
            # Debug the raw values
            print(f"RAW GPS: lat_raw={raw_lat}, lon_raw={raw_lon}")
            print(f"CONVERTED GPS: lat={lat}, lon={lon}, alt={alt}")
            
            self.telemetry_data['gps_lat'] = lat
            self.telemetry_data['gps_lon'] = lon
            self.telemetry_data['altitude'] = alt
            
        elif msg.get_type() == 'VFR_HUD':
            self.telemetry_data['ground_speed'] = msg.groundspeed
            self.telemetry_data['heading'] = msg.heading
            
        elif msg.get_type() == 'ATTITUDE':
            self.telemetry_data['roll'] = msg.roll * 57.2958
            self.telemetry_data['pitch'] = msg.pitch * 57.2958
            self.telemetry_data['yaw'] = msg.yaw * 57.2958
            
        elif msg.get_type() == 'SYS_STATUS':
            # Simulate battery drain over time
            elapsed_minutes = (time.time() - self.battery_start_time) / 60.0
            simulated_voltage = max(18.0, self.initial_battery - (self.battery_drain_rate * elapsed_minutes))
            
            self.telemetry_data['battery_voltage'] = simulated_voltage
            
        elif msg.get_type() == 'GPS_RAW_INT':
            self.telemetry_data['gps_satellites'] = msg.satellites_visible
            
        self.telemetry_updated.emit(self.telemetry_data.copy())
                
    def _get_flight_mode(self, custom_mode):
        """Convert custom mode to flight mode string - ArduPilot mapping"""
        mode_map = {
            0: 'STABILIZE',
            1: 'ACRO', 
            2: 'ALT_HOLD',
            3: 'AUTO',
            4: 'GUIDED',
            5: 'LOITER',
            6: 'RTL',
            7: 'CIRCLE',
            9: 'LAND',
            11: 'DRIFT',
            13: 'SPORT',
            14: 'FLIP',
            15: 'AUTOTUNE',
            16: 'POSHOLD',
            17: 'BRAKE',
            18: 'THROW',
            19: 'AVOID_ADSB',
            20: 'GUIDED_NOGPS',
            21: 'SMART_RTL',
            22: 'FLOWHOLD',
            23: 'FOLLOW',
            24: 'ZIGZAG',
            25: 'SYSTEMID',
            26: 'AUTOROTATE'
        }
        return mode_map.get(custom_mode, f'MODE_{custom_mode}')
        
    def _check_connection(self):
        """Check if connection is still active"""
        pass


class TileDownloader(QObject):
    """Async tile downloader with multiple servers"""
    tile_downloaded = Signal(int, int, int, QPixmap)
    
    def __init__(self):
        super().__init__()
        self.executor = ThreadPoolExecutor(max_workers=16)
        
        self.cache_dir = os.path.expanduser("~/.drone_map_cache")
        os.makedirs(self.cache_dir, exist_ok=True)
        print(f"Persistent tile cache: {self.cache_dir}")
        
        self.tile_servers = [
            "https://tile.openstreetmap.org",
            "https://a.tile.openstreetmap.org", 
            "https://b.tile.openstreetmap.org",
            "https://c.tile.openstreetmap.org"
        ]
        self.server_index = 0
        
    def download_tiles_batch(self, tile_requests):
        """Download multiple tiles concurrently"""
        futures = []
        for zoom, x, y in tile_requests:
            future = self.executor.submit(self._download_single_tile, zoom, x, y)
            futures.append(future)
        
        for future in as_completed(futures):
            result = future.result()
            if result:
                zoom, x, y, pixmap = result
                self.tile_downloaded.emit(zoom, x, y, pixmap)
    
    def _get_next_server(self):
        """Get next server using round-robin"""
        server = self.tile_servers[self.server_index]
        self.server_index = (self.server_index + 1) % len(self.tile_servers)
        return server
    
    def _download_single_tile(self, zoom, x, y):
        """Download a single tile with fallback servers"""
        tile_path = os.path.join(self.cache_dir, f"{zoom}_{x}_{y}.png")
        if os.path.exists(tile_path):
            try:
                pixmap = QPixmap(tile_path)
                if not pixmap.isNull():
                    return (zoom, x, y, pixmap)
            except Exception:
                try:
                    os.remove(tile_path)
                except:
                    pass
        
        for attempt in range(len(self.tile_servers)):
            server = self._get_next_server()
            try:
                url = f"{server}/{zoom}/{x}/{y}.png"
                headers = {
                    'User-Agent': 'Drone Monitor 1.0',
                    'Accept': 'image/png,image/*,*/*;q=0.8',
                    'Accept-Encoding': 'gzip, deflate',
                    'Connection': 'keep-alive'
                }
                
                response = requests.get(url, headers=headers, timeout=10)
                if response.status_code == 200:
                    try:
                        with open(tile_path, 'wb') as f:
                            f.write(response.content)
                        
                        pixmap = QPixmap(tile_path)
                        if not pixmap.isNull():
                            print(f"Downloaded tile {zoom}/{x}/{y} from {server}")
                            return (zoom, x, y, pixmap)
                    except Exception as e:
                        print(f"Error saving tile {zoom}/{x}/{y}: {e}")
                        continue
                        
                elif response.status_code == 429:
                    print(f"Rate limited on {server}, trying next server...")
                    continue
                else:
                    print(f"HTTP {response.status_code} from {server}")
                    continue
                    
            except requests.exceptions.Timeout:
                print(f"Timeout from {server}, trying next server...")
                continue
            except Exception as e:
                print(f"Error downloading from {server}: {e}")
                continue
        
        print(f"Failed to download tile {zoom}/{x}/{y} from all servers")
        return None


class MapWidget(QWidget):
    """Real OpenStreetMap widget with mouse controls and fixed drone arrow"""
    
    def __init__(self):
        super().__init__()
        self.setMinimumSize(600, 400)
        
        # Map parameters
        self.center_lat = 41.0082  # Istanbul
        self.center_lon = 28.9784
        self.zoom = 13
        self.tile_size = 256
        
        # Mouse interaction
        self.last_mouse_pos = QPoint()
        self.is_dragging = False
        
        # Drone position and heading
        self.drone_lat = None
        self.drone_lon = None
        self.drone_heading = 0.0
        
        # Tile cache and downloader
        self.tiles = {}
        self.downloader = TileDownloader()
        self.downloader.tile_downloaded.connect(self.on_tile_downloaded)
        
        # Load initial tiles
        self.load_tiles_around_center(radius=2)
        
    def deg2num(self, lat_deg, lon_deg, zoom):
        """Convert lat/lon to tile numbers"""
        lat_rad = math.radians(lat_deg)
        n = 2.0 ** zoom
        x = int((lon_deg + 180.0) / 360.0 * n)
        y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
        return (x, y)
    
    def pixels_to_degrees(self, pixel_x, pixel_y):
        """Convert pixel movement to lat/lon degrees"""
        meters_per_pixel = 156543.03392 * math.cos(math.radians(self.center_lat)) / (2 ** self.zoom)
        degrees_per_meter_lat = 1.0 / 111320.0
        degrees_per_meter_lon = 1.0 / (111320.0 * math.cos(math.radians(self.center_lat)))
        
        lat_change = -pixel_y * meters_per_pixel * degrees_per_meter_lat
        lon_change = pixel_x * meters_per_pixel * degrees_per_meter_lon
        
        return lat_change, lon_change
    
    def load_tiles_around_center(self, radius=1):
        """Load tiles in a grid around center with smart preloading"""
        center_x, center_y = self.deg2num(self.center_lat, self.center_lon, self.zoom)
        
        tile_requests = []
        priorities = []
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                tile_x = center_x + dx
                tile_y = center_y + dy
                
                max_tile = 2 ** self.zoom
                if 0 <= tile_x < max_tile and 0 <= tile_y < max_tile:
                    if (self.zoom, tile_x, tile_y) not in self.tiles:
                        distance = (dx * dx + dy * dy) ** 0.5
                        priorities.append((distance, (self.zoom, tile_x, tile_y)))
        
        priorities.sort(key=lambda x: x[0])
        tile_requests = [tile for _, tile in priorities]
        
        if tile_requests:
            print(f"Requesting {len(tile_requests)} tiles (priority order)...")
            threading.Thread(
                target=self.downloader.download_tiles_batch, 
                args=(tile_requests,),
                daemon=True
            ).start()
    
    def on_tile_downloaded(self, zoom, x, y, pixmap):
        """Handle downloaded tile"""
        if zoom == self.zoom:
            self.tiles[(zoom, x, y)] = pixmap
            self.update()
            
            if len(self.tiles) > 100:
                self._cleanup_old_tiles()
    
    def _cleanup_old_tiles(self):
        """Remove oldest tiles from memory cache"""
        center_x, center_y = self.deg2num(self.center_lat, self.center_lon, self.zoom)
        
        tiles_to_remove = []
        for (zoom, tile_x, tile_y) in self.tiles.keys():
            if zoom == self.zoom:
                dx = abs(tile_x - center_x)
                dy = abs(tile_y - center_y)
                distance = (dx * dx + dy * dy) ** 0.5
                
                if distance > 4:
                    tiles_to_remove.append((zoom, tile_x, tile_y))
        
        for tile_key in tiles_to_remove:
            del self.tiles[tile_key]
        
        if tiles_to_remove:
            print(f"Cleaned up {len(tiles_to_remove)} old tiles from memory")
    
    def mousePressEvent(self, event: QMouseEvent):
        """Handle mouse press for dragging"""
        if event.button() == Qt.LeftButton:
            self.is_dragging = True
            self.last_mouse_pos = event.position().toPoint()
            self.setCursor(Qt.ClosedHandCursor)
    
    def mouseMoveEvent(self, event: QMouseEvent):
        """Handle mouse move for panning"""
        if self.is_dragging:
            current_pos = event.position().toPoint()
            dx = current_pos.x() - self.last_mouse_pos.x()
            dy = current_pos.y() - self.last_mouse_pos.y()
            
            lat_change, lon_change = self.pixels_to_degrees(dx, dy)
            
            self.center_lat -= lat_change
            self.center_lon -= lon_change
            
            self.center_lat = max(-85, min(85, self.center_lat))
            self.center_lon = max(-180, min(180, self.center_lon))
            
            self.last_mouse_pos = current_pos
            self.update()
    
    def mouseReleaseEvent(self, event: QMouseEvent):
        """Handle mouse release"""
        if event.button() == Qt.LeftButton:
            self.is_dragging = False
            self.setCursor(Qt.ArrowCursor)
            
            QTimer.singleShot(300, lambda: self.load_tiles_around_center(radius=2))
    
    def wheelEvent(self, event: QWheelEvent):
        """Handle mouse wheel for zooming"""
        mouse_x = event.position().x() - self.width() / 2
        mouse_y = event.position().y() - self.height() / 2
        
        lat_change, lon_change = self.pixels_to_degrees(mouse_x, mouse_y)
        mouse_lat = self.center_lat + lat_change
        mouse_lon = self.center_lon + lon_change
        
        if event.angleDelta().y() > 0:
            if self.zoom < 19:
                self.zoom += 1
        else:
            if self.zoom > 1:
                self.zoom -= 1
        
        lat_change_new, lon_change_new = self.pixels_to_degrees(mouse_x, mouse_y)
        self.center_lat = mouse_lat - lat_change_new
        self.center_lon = mouse_lon - lon_change_new
        
        self.center_lat = max(-85, min(85, self.center_lat))
        self.center_lon = max(-180, min(180, self.center_lon))
        
        self.tiles.clear()
        self.load_tiles_around_center(radius=3)
        
        print(f"Zoomed to level {self.zoom}, center: {self.center_lat:.6f}, {self.center_lon:.6f}")
    
    def paintEvent(self, event):
        """Paint the map"""
        painter = QPainter(self)
        
        painter.fillRect(self.rect(), Qt.lightGray)
        
        if len(self.tiles) == 0:
            painter.setPen(Qt.black)
            painter.setFont(QFont("Arial", 14))
            painter.drawText(20, 30, "Loading OpenStreetMap tiles...")
            painter.drawText(20, 50, f"Center: Istanbul ({self.center_lat:.4f}, {self.center_lon:.4f})")
            painter.drawText(20, 70, "Drag to pan, scroll to zoom")
            
            if self.drone_lat and self.drone_lon:
                self._draw_drone_marker(painter)
            return
        
        center_x, center_y = self.deg2num(self.center_lat, self.center_lon, self.zoom)
        
        widget_center_x = self.width() // 2
        widget_center_y = self.height() // 2
        
        for (zoom, tile_x, tile_y), pixmap in self.tiles.items():
            if zoom == self.zoom:
                dx = tile_x - center_x
                dy = tile_y - center_y
                
                draw_x = widget_center_x + (dx * self.tile_size) - self.tile_size // 2
                draw_y = widget_center_y + (dy * self.tile_size) - self.tile_size // 2
                
                painter.drawPixmap(draw_x, draw_y, pixmap)
        
        if self.drone_lat and self.drone_lon:
            self._draw_drone_marker(painter)
        
        painter.setPen(Qt.blue)
        painter.drawLine(widget_center_x - 10, widget_center_y, widget_center_x + 10, widget_center_y)
        painter.drawLine(widget_center_x, widget_center_y - 10, widget_center_x, widget_center_y + 10)
        
        painter.setPen(Qt.black)
        painter.setFont(QFont("Arial", 10))
        painter.drawText(10, self.height() - 60, f"Lat: {self.center_lat:.8f}, Lon: {self.center_lon:.8f}")
        painter.drawText(10, self.height() - 45, f"Zoom: {self.zoom}, Tiles: {len(self.tiles)}")
        painter.drawText(10, self.height() - 30, "Drag to pan, scroll to zoom")
        
        if self.drone_lat and self.drone_lon:
            painter.setPen(Qt.darkGreen)
            painter.setFont(QFont("Arial", 10, QFont.Bold))
            painter.drawText(10, self.height() - 15, f"🛰️ GPS LOCK: {self.drone_lat:.8f}, {self.drone_lon:.8f}")
        else:
            painter.setPen(Qt.red)
            painter.setFont(QFont("Arial", 10, QFont.Bold))
            painter.drawText(10, self.height() - 15, "📡 Waiting for GPS lock...")
    
    def _draw_drone_marker(self, painter):
        """Draw drone position marker with heading arrow"""
        lat_diff = self.drone_lat - self.center_lat
        lon_diff = self.drone_lon - self.center_lon
        
        lat_change_per_pixel, lon_change_per_pixel = self.pixels_to_degrees(1, 1)
        
        if lat_change_per_pixel != 0 and lon_change_per_pixel != 0:
            scale_y = 1.0 / lat_change_per_pixel
            scale_x = 1.0 / lon_change_per_pixel
            
            drone_x = self.width() // 2 + int(lon_diff * scale_x)
            drone_y = self.height() // 2 - int(lat_diff * scale_y)
            
            if -100 <= drone_x <= self.width() + 100 and -100 <= drone_y <= self.height() + 100:
                self._draw_drone_arrow(painter, drone_x, drone_y, self.drone_heading)
        else:
            print("Scale calculation failed, drawing drone at center")
            self._draw_drone_arrow(painter, self.width() // 2, self.height() // 2, self.drone_heading)
    
    def _draw_drone_arrow(self, painter, x, y, heading_deg):
        """Draw a directional arrow showing drone position and heading"""
        painter.save()
        
        painter.translate(x, y)
        
        qt_rotation = heading_deg - 90
        painter.rotate(qt_rotation)
        
        arrow_size = 25
        arrow = QPolygonF([
            QPointF(0, -arrow_size),
            QPointF(-arrow_size//3, arrow_size//3),
            QPointF(0, arrow_size//6),
            QPointF(arrow_size//3, arrow_size//3),
        ])
        
        painter.setPen(QPen(QColor(0, 0, 0), 3))
        painter.setBrush(QBrush(QColor(0, 0, 0, 100)))
        shadow_arrow = QPolygonF()
        for point in arrow:
            shadow_arrow.append(QPointF(point.x() + 2, point.y() + 2))
        painter.drawPolygon(shadow_arrow)
        
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.setBrush(QBrush(QColor(255, 50, 50)))
        painter.drawPolygon(arrow)
        
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        painter.drawEllipse(-4, -4, 8, 8)
        
        painter.restore()
        
        painter.setPen(QColor(0, 0, 0))
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        
        text = f"{heading_deg:.0f}°"
        text_width = painter.fontMetrics().horizontalAdvance(text)
        text_height = painter.fontMetrics().height()
        
        text_rect = QPointF(x - text_width//2, y + arrow_size + text_height + 5)
        painter.fillRect(int(text_rect.x() - 2), int(text_rect.y() - text_height), 
                        text_width + 4, text_height + 2, QColor(255, 255, 255, 200))
        
        painter.setPen(QColor(0, 0, 0))
        painter.drawText(int(text_rect.x()), int(text_rect.y()), text)
        
        label_y = y - arrow_size - 10
        drone_text = "DRONE"
        drone_width = painter.fontMetrics().horizontalAdvance(drone_text)
        painter.fillRect(x - drone_width//2 - 2, label_y - text_height, 
                        drone_width + 4, text_height + 2, QColor(255, 255, 255, 200))
        painter.setPen(QColor(255, 0, 0))
        painter.drawText(x - drone_width//2, label_y, drone_text)
    
    def update_drone_position(self, lat, lon):
        """Update drone position and refresh display"""
        if lat and lon and lat != 0 and lon != 0 and abs(lat) > 0.001 and abs(lon) > 0.001:
            first_gps_lock = (self.drone_lat is None or self.drone_lon is None)
            
            self.drone_lat = lat
            self.drone_lon = lon
            
            if first_gps_lock:
                print(f"🎯 First GPS lock acquired! Auto-zooming to drone position...")
                print(f"High precision coordinates: {lat:.8f}, {lon:.8f}")
                self.center_lat = lat
                self.center_lon = lon
                self.zoom = 18
                self.tiles.clear()
                self.load_tiles_around_center(radius=3)
                print(f"Map centered and zoomed to: {lat:.8f}, {lon:.8f} at zoom {self.zoom}")
            else:
                distance = ((lat - self.center_lat) ** 2 + (lon - self.center_lon) ** 2) ** 0.5
                if distance > 0.01:
                    self.center_lat = lat
                    self.center_lon = lon
                    self.tiles.clear()
                    self.load_tiles_around_center(radius=2)
            
            self.update()
        else:
            if self.drone_lat is not None:
                print("❌ GPS lock lost")
            self.drone_lat = None
            self.drone_lon = None
            self.update()
    
    def update_drone_heading(self, heading):
        """Update drone heading for arrow direction"""
        self.drone_heading = heading
        self.update()


class DroneMonitorUI(QMainWindow):
    """Main UI Class with Image Support"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Telemetry Monitor")
        self.setGeometry(100, 100, 1200, 800)
        
        # Initialize image icons
        self.images = {}
        self._load_images()

        # Set window icon (ADD THIS LINE)
        if 'logo' in self.images and self.images['logo']:
            self.setWindowIcon(QIcon(self.images['logo']))
        
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        # Main layout - horizontal (left side + right side)
        main_layout = QHBoxLayout(central_widget)

        # Left side - vertical layout for telemetry + map
        left_layout = QVBoxLayout()

        # Top: telemetry panel
        telemetry_panel = self._create_telemetry_panel()
        left_layout.addWidget(telemetry_panel)

        # Bottom: map
        self.map_widget = MapWidget()
        left_layout.addWidget(self.map_widget, 1)  # Map takes most space

        # Add left layout to main
        left_widget = QWidget()
        left_widget.setLayout(left_layout)
        main_layout.addWidget(left_widget, 6)  # Left side takes 2/3

        # Right panel
        right_panel = self._create_right_panel()
        main_layout.addWidget(right_panel, 1)  # Right side takes 1/3
        
        # Initialize telemetry receiver
        self.telemetry_receiver = TelemetryReceiver(port=14551)
        self.telemetry_receiver.telemetry_updated.connect(self._update_telemetry_display)
        self.telemetry_receiver.connection_changed.connect(self._update_connection_status)
        
        # Start receiving telemetry
        # self.telemetry_receiver.start_receiving()
    
    def _load_images(self):
        """Load all UI images - PyInstaller compatible"""
        # Handle PyInstaller bundled resources
        if hasattr(sys, '_MEIPASS'):
            base_path = sys._MEIPASS
        else:
            base_path = os.path.dirname(__file__)
        
        image_files = {
            'connection_on': os.path.join(base_path, 'images', 'connection_on.png'),
            'connection_off': os.path.join(base_path, 'images', 'connection_off.png'),
            'drone': os.path.join(base_path, 'images', 'drone.png'),
            'battery': os.path.join(base_path, 'images', 'battery.png'),
            'satellite': os.path.join(base_path, 'images', 'satellite.png'),
            'shield_green': os.path.join(base_path, 'images', 'shield_green.png'),
            'shield_red': os.path.join(base_path, 'images', 'shield_red.png'),
            'compass': os.path.join(base_path, 'images', 'compass.png'),
            'altitude': os.path.join(base_path, 'images', 'altitude.png'),
            'speed': os.path.join(base_path, 'images', 'speed.png'),
            'logo': os.path.join(base_path, 'images', 'logo.png')
        }
        
        for name, filepath in image_files.items():
            try:
                if os.path.exists(filepath):
                    pixmap = QPixmap(filepath)
                    if not pixmap.isNull():
                        scaled_pixmap = pixmap.scaled(24, 24, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                        self.images[name] = scaled_pixmap
                        print(f"✅ Loaded image: {name}")
                    else:
                        print(f"❌ Failed to load image: {filepath} (invalid format)")
                        self.images[name] = None
                else:
                    print(f"⚠️ Image not found: {filepath}")
                    self.images[name] = None
            except Exception as e:
                print(f"❌ Error loading {filepath}: {e}")
                self.images[name] = None
    
    def _create_image_label(self, image_name):
        """Create a QLabel with an image"""
        label = QLabel()
        
        if image_name in self.images and self.images[image_name]:
            label.setPixmap(self.images[image_name])
        else:
            # Fallback: show text if image not available
            label.setText("●")
            label.setStyleSheet("color: gray;")
        
        label.setFixedSize(24, 24)
        label.setScaledContents(True)
        return label
        
    def _create_telemetry_panel(self):
        """Create the telemetry data panel with images - horizontal grid layout"""
        panel = QWidget()
        panel.setMaximumHeight(120)  # Limit height to keep it compact
        
        # Use grid layout - horizontal arrangement
        grid_layout = QGridLayout(panel)
        
        # Row 0: Icons
        # Row 1: Labels  
        # Row 2: Values
        
        col = 0
        
        # Connection
        self.connection_icon = self._create_image_label('connection_off')
        grid_layout.addWidget(self.connection_icon, 0, col, Qt.AlignCenter)
        self.label_connection_status = QLabel("Disconnected")
        self.label_connection_status.setStyleSheet("font-weight: bold; color: red; font-size: 10px;")
        grid_layout.addWidget(self.label_connection_status, 1, col, Qt.AlignCenter)
        col += 1
        
        # Flight Mode
        flight_mode_icon = self._create_image_label('drone')
        grid_layout.addWidget(flight_mode_icon, 0, col, Qt.AlignCenter)
        grid_layout.addWidget(QLabel("Flight Mode"), 1, col, Qt.AlignCenter)
        self.label_flight_mode = QLabel("Unknown")
        grid_layout.addWidget(self.label_flight_mode, 2, col, Qt.AlignCenter)
        col += 1
        
        # Armed Status  
        self.armed_icon = self._create_image_label('shield_green')
        grid_layout.addWidget(self.armed_icon, 0, col, Qt.AlignCenter)
        grid_layout.addWidget(QLabel("Armed"), 1, col, Qt.AlignCenter)
        self.label_armed_status = QLabel("Disarmed")
        grid_layout.addWidget(self.label_armed_status, 2, col, Qt.AlignCenter)
        col += 1
        
        # GPS
        gps_icon = self._create_image_label('satellite')
        grid_layout.addWidget(gps_icon, 0, col, Qt.AlignCenter)
        grid_layout.addWidget(QLabel("GPS"), 1, col, Qt.AlignCenter)
        self.label_gps_coordinates = QLabel("0.000000, 0.000000")
        grid_layout.addWidget(self.label_gps_coordinates, 2, col, Qt.AlignCenter)
        col += 1
        
        # Altitude
        alt_icon = self._create_image_label('altitude')
        grid_layout.addWidget(alt_icon, 0, col, Qt.AlignCenter)
        grid_layout.addWidget(QLabel("Altitude"), 1, col, Qt.AlignCenter)
        self.label_altitude = QLabel("0.0")
        grid_layout.addWidget(self.label_altitude, 2, col, Qt.AlignCenter)
        col += 1
        
        # Speed
        speed_icon = self._create_image_label('speed')
        grid_layout.addWidget(speed_icon, 0, col, Qt.AlignCenter)
        grid_layout.addWidget(QLabel("Speed"), 1, col, Qt.AlignCenter)
        self.label_ground_speed = QLabel("0.0")
        grid_layout.addWidget(self.label_ground_speed, 2, col, Qt.AlignCenter)
        col += 1
        
        # Heading
        compass_icon = self._create_image_label('compass')
        grid_layout.addWidget(compass_icon, 0, col, Qt.AlignCenter)
        grid_layout.addWidget(QLabel("Heading"), 1, col, Qt.AlignCenter)
        self.label_heading = QLabel("0")
        grid_layout.addWidget(self.label_heading, 2, col, Qt.AlignCenter)
        col += 1
        
        # Battery
        battery_icon = self._create_image_label('battery')
        grid_layout.addWidget(battery_icon, 0, col, Qt.AlignCenter)
        grid_layout.addWidget(QLabel("Battery"), 1, col, Qt.AlignCenter)
        self.label_battery_voltage = QLabel("0.0")
        grid_layout.addWidget(self.label_battery_voltage, 2, col, Qt.AlignCenter)
    
        return panel
        
    def _update_telemetry_display(self, data):
        """Update all telemetry displays"""
        self.label_flight_mode.setText(data['flight_mode'])
        
        # Update armed status and icon
        if data['armed']:
            self.label_armed_status.setText("Armed")
            self.label_armed_status.setStyleSheet("color: red; font-weight: bold;")
            if 'shield_red' in self.images and self.images['shield_red']:
                self.armed_icon.setPixmap(self.images['shield_red'])
        else:
            self.label_armed_status.setText("Disarmed")
            self.label_armed_status.setStyleSheet("color: green; font-weight: bold;")
            if 'shield_green' in self.images and self.images['shield_green']:
                self.armed_icon.setPixmap(self.images['shield_green'])
        
        self.label_gps_coordinates.setText(f"{data['gps_lat']:.8f}, {data['gps_lon']:.8f}")
        self.label_altitude.setText(f"{data['altitude']:.2f}")
        self.label_ground_speed.setText(f"{data['ground_speed']:.2f}")
        self.label_heading.setText(f"{data['heading']:.1f}")
        
        
        # Update map with position and heading
        self.map_widget.update_drone_position(data['gps_lat'], data['gps_lon'])
        self.map_widget.update_drone_heading(data['heading'])
        
    def _update_connection_status(self, connected):
        """Update connection status display and icon"""
        if connected:
            self.label_connection_status.setText("Connected")
            self.label_connection_status.setStyleSheet("font-weight: bold; color: green;")
            if 'connection_on' in self.images and self.images['connection_on']:
                self.connection_icon.setPixmap(self.images['connection_on'])
        else:
            self.label_connection_status.setText("Disconnected")
            self.label_connection_status.setStyleSheet("font-weight: bold; color: red;")
            if 'connection_off' in self.images and self.images['connection_off']:
                self.connection_icon.setPixmap(self.images['connection_off'])
    
    def _create_right_panel(self):
        """Create the right side panel"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Text logo instead of image
        logo_widget = QWidget()
        logo_layout = QVBoxLayout(logo_widget)
        logo_layout.setSpacing(1)  # Small gap between lines
        
        # "Horizon 7" in cyan
        horizon_label = QLabel("Horizon 7")
        horizon_label.setAlignment(Qt.AlignCenter)
        horizon_label.setStyleSheet("""
            font-size: 28px; 
            font-weight: bold; 
            color: #00BFFF;
            margin: 0px;
        """)
        
        # "Custom UI" in black, smaller
        custom_label = QLabel("Ground Control Station")
        custom_label.setAlignment(Qt.AlignCenter)
        custom_label.setStyleSheet("""
            font-size: 18px;
            font-weight: bold; 
            color: black;
            margin: 0px;
        """)
        
        logo_layout.addWidget(horizon_label)
        logo_layout.addWidget(custom_label)
        logo_widget.setFixedHeight(80)  # Compact height
        
        layout.addWidget(logo_widget)
        
        # Add other content below the logo
        layout.addWidget(QLabel(""))  # Small spacer

        # Connection section
        connection_widget = QWidget()
        connection_layout = QVBoxLayout(connection_widget)

        # UDP Port label
        udp_label = QLabel("UDP Port:")
        udp_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        connection_layout.addWidget(udp_label)

        # UDP Port input field
        self.udp_input = QLineEdit()
        self.udp_input.setText("14551")  # Default port
        self.udp_input.setPlaceholderText("Enter UDP port...")
        self.udp_input.setStyleSheet("""
            QLineEdit {
                padding: 5px;
                font-size: 12px;
                border: 1px solid gray;
                border-radius: 3px;
            }
        """)
        connection_layout.addWidget(self.udp_input)

        # Small spacer
        connection_layout.addWidget(QLabel(""))

        # Connect button
        self.connect_button = QPushButton("Connect")
        self.connect_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-weight: bold;
                font-size: 14px;
                padding: 8px;
                border: none;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
        """)
        self.connect_button.clicked.connect(self._fake_connect)
        connection_layout.addWidget(self.connect_button)

        layout.addWidget(connection_widget)

        # Add separator line
        separator = QLabel()
        separator.setFixedHeight(2)
        separator.setStyleSheet("background-color: #CCCCCC; margin: 10px 0px;")
        layout.addWidget(separator)

        # Drone Launch Section
        launch_widget = QWidget()
        launch_layout = QVBoxLayout(launch_widget)

        # Section title
        launch_title = QLabel("Launch Drones")
        launch_title.setStyleSheet("font-weight: bold; font-size: 14px; margin-bottom: 5px;")
        launch_title.setAlignment(Qt.AlignCenter)
        launch_layout.addWidget(launch_title)

        # Create 4 drone buttons in 2x2 grid
        buttons_layout = QGridLayout()

        for i in range(1, 5):
            drone_button = QPushButton(f"Drone {i}")
            drone_button.setStyleSheet("""
                QPushButton {
                    background-color: #2196F3;
                    color: white;
                    font-weight: bold;
                    font-size: 12px;
                    padding: 8px;
                    border: none;
                    border-radius: 5px;
                    min-height: 30px;
                }
                QPushButton:hover {
                    background-color: #1976D2;
                }
                QPushButton:pressed {
                    background-color: #0D47A1;
                }
            """)
            drone_button.clicked.connect(lambda checked, drone_id=i: self._launch_drone(drone_id))
            
            # Arrange in 2x2 grid
            row = (i - 1) // 2
            col = (i - 1) % 2
            buttons_layout.addWidget(drone_button, row, col)

        launch_layout.addLayout(buttons_layout)
        layout.addWidget(launch_widget)

        layout.addStretch()

        layout.addStretch()
        
        return panel

    def _fake_connect(self):
        """Real connect/disconnect functionality"""
        if self.connect_button.text() == "Connect":
            # Get port from input
            try:
                port = int(self.udp_input.text())
            except ValueError:
                print("Invalid port number!")
                return
            
            # Start real connection
            self.telemetry_receiver.port = port
            self.telemetry_receiver.start_receiving()
            
            # Update button to disconnect state
            self.connect_button.setText("Disconnect")
            self.connect_button.setStyleSheet("""
                QPushButton {
                    background-color: #f44336;
                    color: white;
                    font-weight: bold;
                    font-size: 14px;
                    padding: 8px;
                    border: none;
                    border-radius: 5px;
                }
                QPushButton:hover {
                    background-color: #da190b;
                }
            """)
            print(f"Connecting to UDP port {port}...")
            
        else:
            # Disconnect
            self.telemetry_receiver.stop_receiving()
            
            # Update button to connect state
            self.connect_button.setText("Connect")
            self.connect_button.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    font-weight: bold;
                    font-size: 14px;
                    padding: 8px;
                    border: none;
                    border-radius: 5px;
                }
                QPushButton:hover {
                    background-color: #45a049;
                }
            """)
            print("Disconnected from UDP")
            
            # Reset connection status display
            self._update_connection_status(False)
    
    def closeEvent(self, event):
        """Handle application close"""
        self.telemetry_receiver.stop_receiving()
        event.accept()


def main():
    import signal
    import sys
    
    app = QApplication(sys.argv)
    app.setStyle('WindowsVista')

    # Add custom grey theme
    app.setStyleSheet("""
        QWidget {
            background-color: #F0F0F0;
            color: #333333;
        }
        QLabel {
            background-color: transparent;
        }
        QPushButton {
            background-color: #E0E0E0;
            border: 1px solid #CCCCCC;
            padding: 5px;
        }
        QPushButton:hover {
            background-color: #D0D0D0;
        }
        QLineEdit {
            background-color: #FFFFFF;
            border: 1px solid #CCCCCC;
        }
    """)
    
    # Create main window
    window = DroneMonitorUI()
    
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("\nShutting down...")
        window.telemetry_receiver.stop_receiving()
        app.quit()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Show window
    window.show()
    
    # Enable Ctrl+C in Qt
    timer = QTimer()
    timer.start(500)  # Check for signals every 500ms
    timer.timeout.connect(lambda: None)
    
    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        print("\nShutting down...")
        window.telemetry_receiver.stop_receiving()
        sys.exit(0)


if __name__ == "__main__":
    main()