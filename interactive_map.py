import sys
import requests
import math
import threading
import queue
from concurrent.futures import ThreadPoolExecutor, as_completed
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PySide6.QtCore import Qt, QTimer, QPoint, Signal, QObject, QPointF
from PySide6.QtGui import QPainter, QPixmap, QFont, QWheelEvent, QMouseEvent, QPen, QBrush, QColor, QPolygonF
import tempfile
import os

class TileDownloader(QObject):
    """Async tile downloader"""
    tile_downloaded = Signal(int, int, int, QPixmap)
    
    def __init__(self):
        super().__init__()
        self.executor = ThreadPoolExecutor(max_workers=8)  # Download 8 tiles concurrently
        self.temp_dir = tempfile.mkdtemp()
        print(f"Tile cache directory: {self.temp_dir}")
        
    def download_tiles_batch(self, tile_requests):
        """Download multiple tiles concurrently"""
        futures = []
        for zoom, x, y in tile_requests:
            future = self.executor.submit(self._download_single_tile, zoom, x, y)
            futures.append(future)
        
        # Process completed downloads
        for future in as_completed(futures):
            result = future.result()
            if result:
                zoom, x, y, pixmap = result
                self.tile_downloaded.emit(zoom, x, y, pixmap)
    
    def _download_single_tile(self, zoom, x, y):
        """Download a single tile"""
        try:
            # Check if already cached
            tile_path = os.path.join(self.temp_dir, f"{zoom}_{x}_{y}.png")
            if os.path.exists(tile_path):
                pixmap = QPixmap(tile_path)
                if not pixmap.isNull():
                    return (zoom, x, y, pixmap)
            
            # Download from server
            url = f"https://tile.openstreetmap.org/{zoom}/{x}/{y}.png"
            headers = {'User-Agent': 'PyQt OSM Viewer 1.0'}
            
            response = requests.get(url, headers=headers, timeout=15)
            if response.status_code == 200:
                # Save to cache
                with open(tile_path, 'wb') as f:
                    f.write(response.content)
                
                # Load as pixmap
                pixmap = QPixmap(tile_path)
                if not pixmap.isNull():
                    print(f"Downloaded tile {zoom}/{x}/{y}")
                    return (zoom, x, y, pixmap)
                    
        except Exception as e:
            print(f"Error downloading tile {zoom}/{x}/{y}: {e}")
        
        return None

class SimpleMapWidget(QWidget):
    """Simple widget that downloads and displays OpenStreetMap tiles"""
    
    def __init__(self):
        super().__init__()
        self.setMinimumSize(800, 600)
        
        # Map parameters
        self.center_lat = 41.0082  # Istanbul
        self.center_lon = 28.9784
        self.zoom = 13
        self.tile_size = 256
        
        # Mouse interaction
        self.last_mouse_pos = QPoint()
        self.is_dragging = False
        
        # Demo drone position and heading
        self.drone_lat = 41.0082  # Start at Istanbul center
        self.drone_lon = 28.9784
        self.drone_heading = 45.0  # Demo heading (45 degrees = northeast)
        self.demo_mode = True
        
        # Tile cache and downloader
        self.tiles = {}
        self.downloader = TileDownloader()
        self.downloader.tile_downloaded.connect(self.on_tile_downloaded)
        
        # Load initial tiles
        self.load_tiles_around_center(radius=2)  # Load 5x5 grid initially
        
        # Demo timer to rotate drone arrow
        self.demo_timer = QTimer()
        self.demo_timer.timeout.connect(self.update_demo_drone)
        self.demo_timer.start(100)  # Update every 100ms for smooth rotation
        
    def deg2num(self, lat_deg, lon_deg, zoom):
        """Convert lat/lon to tile numbers"""
        lat_rad = math.radians(lat_deg)
        n = 2.0 ** zoom
        x = int((lon_deg + 180.0) / 360.0 * n)
        y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
        return (x, y)
    
    def pixels_to_degrees(self, pixel_x, pixel_y):
        """Convert pixel movement to lat/lon degrees"""
        # At this zoom level, how many degrees per pixel?
        meters_per_pixel = 156543.03392 * math.cos(math.radians(self.center_lat)) / (2 ** self.zoom)
        degrees_per_meter_lat = 1.0 / 111320.0  # Approximately
        degrees_per_meter_lon = 1.0 / (111320.0 * math.cos(math.radians(self.center_lat)))
        
        lat_change = -pixel_y * meters_per_pixel * degrees_per_meter_lat
        lon_change = pixel_x * meters_per_pixel * degrees_per_meter_lon
        
        return lat_change, lon_change
    
    def update_demo_drone(self):
        """Update demo drone position and heading"""
        if self.demo_mode:
            # Rotate heading for demo
            self.drone_heading += 2.0  # Rotate 2 degrees per update
            if self.drone_heading >= 360:
                self.drone_heading = 0
            
            # Optional: small circular movement
            import time
            t = time.time() * 0.5  # Slow circular motion
            radius = 0.002  # Small radius
            self.drone_lat = 41.0082 + radius * math.sin(t)
            self.drone_lon = 28.9784 + radius * math.cos(t)
            
            self.update()  # Trigger repaint
    
    def load_tiles_around_center(self, radius=1):
        """Load tiles in a grid around center"""
        center_x, center_y = self.deg2num(self.center_lat, self.center_lon, self.zoom)
        
        tile_requests = []
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                tile_x = center_x + dx
                tile_y = center_y + dy
                
                # Check if tile is valid
                max_tile = 2 ** self.zoom
                if 0 <= tile_x < max_tile and 0 <= tile_y < max_tile:
                    if (self.zoom, tile_x, tile_y) not in self.tiles:
                        tile_requests.append((self.zoom, tile_x, tile_y))
        
        if tile_requests:
            print(f"Requesting {len(tile_requests)} tiles...")
            # Download in background thread
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
    
    def mousePressEvent(self, event: QMouseEvent):
        """Handle mouse press for dragging"""
        if event.button() == Qt.LeftButton:
            self.is_dragging = True
            self.last_mouse_pos = event.position().toPoint()
            self.setCursor(Qt.ClosedHandCursor)
    
    def mouseMoveEvent(self, event: QMouseEvent):
        """Handle mouse move for panning"""
        if self.is_dragging:
            # Calculate movement
            current_pos = event.position().toPoint()
            dx = current_pos.x() - self.last_mouse_pos.x()
            dy = current_pos.y() - self.last_mouse_pos.y()
            
            # Convert to lat/lon change (positive dx = drag right = move map right = show area to the left)
            lat_change, lon_change = self.pixels_to_degrees(dx, dy)
            
            # Update center (subtract to reverse direction)
            self.center_lat -= lat_change
            self.center_lon -= lon_change
            
            # Constrain to valid ranges
            self.center_lat = max(-85, min(85, self.center_lat))
            self.center_lon = max(-180, min(180, self.center_lon))
            
            self.last_mouse_pos = current_pos
            self.update()
    
    def mouseReleaseEvent(self, event: QMouseEvent):
        """Handle mouse release"""
        if event.button() == Qt.LeftButton:
            self.is_dragging = False
            self.setCursor(Qt.ArrowCursor)
            
            # Load new tiles after panning
            QTimer.singleShot(300, lambda: self.load_tiles_around_center(radius=2))
    
    def wheelEvent(self, event: QWheelEvent):
        """Handle mouse wheel for zooming"""
        # Get mouse position relative to widget center
        mouse_x = event.position().x() - self.width() / 2
        mouse_y = event.position().y() - self.height() / 2
        
        # Calculate lat/lon at mouse position before zoom
        lat_change, lon_change = self.pixels_to_degrees(mouse_x, mouse_y)
        mouse_lat = self.center_lat + lat_change
        mouse_lon = self.center_lon + lon_change
        
        # Zoom in or out
        if event.angleDelta().y() > 0:  # Scroll up - zoom in
            if self.zoom < 18:
                self.zoom += 1
        else:  # Scroll down - zoom out
            if self.zoom > 1:
                self.zoom -= 1
        
        # Recalculate center to keep mouse position at same geographic location
        lat_change_new, lon_change_new = self.pixels_to_degrees(mouse_x, mouse_y)
        self.center_lat = mouse_lat - lat_change_new
        self.center_lon = mouse_lon - lon_change_new
        
        # Constrain to valid ranges
        self.center_lat = max(-85, min(85, self.center_lat))
        self.center_lon = max(-180, min(180, self.center_lon))
        
        # Clear tiles and reload with larger radius for smoother zooming
        self.tiles.clear()
        self.load_tiles_around_center(radius=3)
        
        print(f"Zoomed to level {self.zoom}, center: {self.center_lat:.6f}, {self.center_lon:.6f}")
    
    def paintEvent(self, event):
        """Paint the map"""
        painter = QPainter(self)
        
        # Fill background
        painter.fillRect(self.rect(), Qt.lightGray)
        
        # Draw debug info
        painter.setPen(Qt.black)
        painter.setFont(QFont("Arial", 12))
        painter.drawText(10, 25, f"Center: {self.center_lat:.6f}, {self.center_lon:.6f}")
        painter.drawText(10, 45, f"Zoom: {self.zoom}")
        painter.drawText(10, 65, f"Tiles cached: {len(self.tiles)}")
        painter.drawText(10, 85, "Left-click and drag to pan, scroll wheel to zoom")
        
        if len(self.tiles) == 0:
            painter.drawText(10, 105, "Loading tiles...")
            # Still draw the drone arrow even without tiles
            self._draw_demo_drone_arrow(painter)
            return
        
        # Calculate center tile
        center_x, center_y = self.deg2num(self.center_lat, self.center_lon, self.zoom)
        
        # Widget center
        widget_center_x = self.width() // 2
        widget_center_y = self.height() // 2
        
        # Draw tiles
        for (zoom, tile_x, tile_y), pixmap in self.tiles.items():
            if zoom == self.zoom:
                # Calculate position relative to center tile
                dx = tile_x - center_x
                dy = tile_y - center_y
                
                # Draw position
                draw_x = widget_center_x + (dx * self.tile_size) - self.tile_size // 2
                draw_y = widget_center_y + (dy * self.tile_size) - self.tile_size // 2
                
                painter.drawPixmap(draw_x, draw_y, pixmap)
        
        # Draw demo drone arrow
        self._draw_demo_drone_arrow(painter)
        
        # Draw center crosshair
        painter.setPen(Qt.blue)
        painter.drawLine(widget_center_x - 20, widget_center_y, widget_center_x + 20, widget_center_y)
        painter.drawLine(widget_center_x, widget_center_y - 20, widget_center_x, widget_center_y + 20)
    
    def _draw_demo_drone_arrow(self, painter):
        """Draw drone arrow at current position with current heading"""
        # Calculate drone position on screen
        lat_diff = self.drone_lat - self.center_lat
        lon_diff = self.drone_lon - self.center_lon
        
        # Convert to pixels (simple approximation)
        lat_change, lon_change = self.pixels_to_degrees(1, 1)  # Get scale per pixel
        if lat_change != 0 and lon_change != 0:
            scale_y = 1.0 / lat_change
            scale_x = 1.0 / lon_change
            
            drone_x = self.width() // 2 + int(lon_diff * scale_x)
            drone_y = self.height() // 2 - int(lat_diff * scale_y)
            
            # Always draw arrow even if slightly off screen (for demo purposes)
            # Clamp to visible area for demo
            drone_x = max(50, min(self.width() - 50, drone_x))
            drone_y = max(50, min(self.height() - 50, drone_y))
            
            self._draw_arrow_at_position(painter, drone_x, drone_y, self.drone_heading)
        else:
            # Fallback: draw at center if we can't calculate position
            self._draw_arrow_at_position(painter, self.width() // 2, self.height() // 2, self.drone_heading)
    
    def _draw_arrow_at_position(self, painter, x, y, heading_deg):
        """Draw a directional arrow at specific position"""
        # Save current painter state
        painter.save()
        
        # Move to drone position
        painter.translate(x, y)
        
        # Rotate based on heading (convert to Qt coordinate system)
        qt_rotation = heading_deg - 90  # Adjust so 0° points up (north)
        painter.rotate(qt_rotation)
        
        # Define arrow shape (pointing upward in local coordinates)
        arrow_size = 25
        arrow = QPolygonF([
            QPointF(0, -arrow_size),           # Tip (pointing up)
            QPointF(-arrow_size//3, arrow_size//3),    # Left wing
            QPointF(0, arrow_size//6),         # Body center
            QPointF(arrow_size//3, arrow_size//3),     # Right wing
        ])
        
        # Draw arrow shadow (for better visibility)
        painter.setPen(QPen(QColor(0, 0, 0), 3))
        painter.setBrush(QBrush(QColor(0, 0, 0, 100)))
        shadow_arrow = QPolygonF()
        for point in arrow:
            shadow_arrow.append(QPointF(point.x() + 2, point.y() + 2))
        painter.drawPolygon(shadow_arrow)
        
        # Draw main arrow
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.setBrush(QBrush(QColor(255, 50, 50)))  # Red arrow
        painter.drawPolygon(arrow)
        
        # Draw center dot
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        painter.drawEllipse(-4, -4, 8, 8)
        
        # Restore painter state
        painter.restore()
        
        # Draw heading text below drone
        painter.setPen(QColor(0, 0, 0))
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        
        # Background for text
        text = f"{heading_deg:.0f}°"
        text_width = painter.fontMetrics().horizontalAdvance(text)
        text_height = painter.fontMetrics().height()
        
        text_rect = QPointF(x - text_width//2, y + arrow_size + text_height + 5)
        painter.fillRect(int(text_rect.x() - 2), int(text_rect.y() - text_height), 
                        text_width + 4, text_height + 2, QColor(255, 255, 255, 200))
        
        # Draw heading text
        painter.setPen(QColor(0, 0, 0))
        painter.drawText(int(text_rect.x()), int(text_rect.y()), text)
        
        # Draw "DEMO DRONE" label above
        label_y = y - arrow_size - 10
        drone_text = "DEMO DRONE"
        drone_width = painter.fontMetrics().horizontalAdvance(drone_text)
        painter.fillRect(x - drone_width//2 - 2, label_y - text_height, 
                        drone_width + 4, text_height + 2, QColor(255, 255, 255, 200))
        painter.setPen(QColor(255, 0, 0))
        painter.drawText(x - drone_width//2, label_y, drone_text)
    
    def zoom_in(self):
        """Zoom in"""
        if self.zoom < 18:
            self.zoom += 1
            self.tiles.clear()
            self.load_tiles_around_center(radius=2)
    
    def zoom_out(self):
        """Zoom out"""
        if self.zoom > 1:
            self.zoom -= 1
            self.tiles.clear()
            self.load_tiles_around_center(radius=2)


class SimpleMapWindow(QMainWindow):
    """Simple map window"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simple OpenStreetMap Test - Drone Arrow Demo")
        self.setGeometry(100, 100, 900, 700)
        
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Layout
        layout = QVBoxLayout(central_widget)
        
        # Info label
        info_label = QLabel("Demo: Rotating drone arrow - Red arrow shows drone position and heading")
        info_label.setStyleSheet("font-weight: bold; color: blue;")
        layout.addWidget(info_label)
        
        # Controls
        controls = QHBoxLayout()
        zoom_in_btn = QPushButton("Zoom In")
        zoom_out_btn = QPushButton("Zoom Out")
        reload_btn = QPushButton("Reload Tiles")
        toggle_demo_btn = QPushButton("Toggle Demo")
        
        controls.addWidget(zoom_in_btn)
        controls.addWidget(zoom_out_btn)
        controls.addWidget(reload_btn)
        controls.addWidget(toggle_demo_btn)
        controls.addStretch()
        
        # Map widget
        self.map_widget = SimpleMapWidget()
        
        # Add to layout
        layout.addLayout(controls)
        layout.addWidget(self.map_widget)
        
        # Connect buttons
        zoom_in_btn.clicked.connect(self.map_widget.zoom_in)
        zoom_out_btn.clicked.connect(self.map_widget.zoom_out)
        reload_btn.clicked.connect(self.reload_tiles)
        toggle_demo_btn.clicked.connect(self.toggle_demo)
        
    def toggle_demo(self):
        """Toggle demo mode on/off"""
        self.map_widget.demo_mode = not self.map_widget.demo_mode
        if self.map_widget.demo_mode:
            self.map_widget.demo_timer.start()
            print("Demo mode ON - Arrow will rotate and move")
        else:
            self.map_widget.demo_timer.stop()
            print("Demo mode OFF - Arrow is static")
        
    def reload_tiles(self):
        """Reload tiles"""
        self.map_widget.tiles.clear()
        self.map_widget.load_tiles_around_center()


def main():
    app = QApplication(sys.argv)
    
    # Test internet connection first
    try:
        response = requests.get("https://tile.openstreetmap.org/0/0/0.png", timeout=5)
        print(f"Internet test: HTTP {response.status_code}")
    except Exception as e:
        print(f"Internet connection failed: {e}")
        print("You can still see the drone arrow demo without map tiles")
    
    window = SimpleMapWindow()
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()