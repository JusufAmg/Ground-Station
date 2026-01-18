import sys
import requests
import math
import threading
import logging
from concurrent.futures import ThreadPoolExecutor, as_completed
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PySide6.QtCore import Qt, QTimer, QPoint, Signal, QObject, QPointF
from PySide6.QtGui import QPainter, QPixmap, QFont, QWheelEvent, QMouseEvent, QPen, QBrush, QColor, QPolygonF
import tempfile
import os

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TileDownloader(QObject):
    MAX_WORKERS = 8
    
    tile_downloaded = Signal(int, int, int, QPixmap)
    
    def __init__(self):
        super().__init__()
        self.executor = ThreadPoolExecutor(max_workers=self.MAX_WORKERS)
        self.temp_dir = tempfile.mkdtemp()
        logger.info(f"Tile cache: {self.temp_dir}")
        
    def download_tiles_batch(self, tile_requests):
        futures = [self.executor.submit(self._download_single_tile, zoom, x, y) 
                   for zoom, x, y in tile_requests]
        
        for future in as_completed(futures):
            result = future.result()
            if result:
                zoom, x, y, pixmap = result
                self.tile_downloaded.emit(zoom, x, y, pixmap)
    
    def _download_single_tile(self, zoom, x, y):
        try:
            tile_path = os.path.join(self.temp_dir, f"{zoom}_{x}_{y}.png")
            if os.path.exists(tile_path):
                pixmap = QPixmap(tile_path)
                if not pixmap.isNull():
                    return (zoom, x, y, pixmap)
            
            url = f"https://tile.openstreetmap.org/{zoom}/{x}/{y}.png"
            headers = {'User-Agent': 'PyQt OSM Viewer 1.0'}
            
            response = requests.get(url, headers=headers, timeout=15)
            if response.status_code == 200:
                with open(tile_path, 'wb') as f:
                    f.write(response.content)
                
                pixmap = QPixmap(tile_path)
                if not pixmap.isNull():
                    return (zoom, x, y, pixmap)
                    
        except Exception as e:
            logger.debug(f"Error downloading tile {zoom}/{x}/{y}: {e}")
        
        return None


class SimpleMapWidget(QWidget):
    DEFAULT_CENTER = (41.0082, 28.9784)  # Istanbul
    DEFAULT_ZOOM = 13
    TILE_SIZE = 256
    DEMO_ROTATION_SPEED = 2.0  # degrees per update
    DEMO_ORBIT_RADIUS = 0.002
    
    def __init__(self):
        super().__init__()
        self.setMinimumSize(800, 600)
        
        self.center_lat, self.center_lon = self.DEFAULT_CENTER
        self.zoom = self.DEFAULT_ZOOM
        self.tile_size = self.TILE_SIZE
        
        self.last_mouse_pos = QPoint()
        self.is_dragging = False
        
        self.drone_lat, self.drone_lon = self.DEFAULT_CENTER
        self.drone_heading = 45.0
        self.demo_mode = True
        
        self.tiles = {}
        self.downloader = TileDownloader()
        self.downloader.tile_downloaded.connect(self.on_tile_downloaded)
        
        self.load_tiles_around_center(radius=2)
        
        self.demo_timer = QTimer()
        self.demo_timer.timeout.connect(self.update_demo_drone)
        self.demo_timer.start(100)
        
    def deg2num(self, lat_deg, lon_deg, zoom):
        lat_rad = math.radians(lat_deg)
        n = 2.0 ** zoom
        x = int((lon_deg + 180.0) / 360.0 * n)
        y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
        return (x, y)
    
    def pixels_to_degrees(self, pixel_x, pixel_y):
        meters_per_pixel = 156543.03392 * math.cos(math.radians(self.center_lat)) / (2 ** self.zoom)
        degrees_per_meter_lat = 1.0 / 111320.0
        degrees_per_meter_lon = 1.0 / (111320.0 * math.cos(math.radians(self.center_lat)))
        
        lat_change = -pixel_y * meters_per_pixel * degrees_per_meter_lat
        lon_change = pixel_x * meters_per_pixel * degrees_per_meter_lon
        
        return lat_change, lon_change
    
    def update_demo_drone(self):
        if self.demo_mode:
            self.drone_heading += self.DEMO_ROTATION_SPEED
            if self.drone_heading >= 360:
                self.drone_heading = 0
            
            import time
            t = time.time() * 0.5
            self.drone_lat = self.DEFAULT_CENTER[0] + self.DEMO_ORBIT_RADIUS * math.sin(t)
            self.drone_lon = self.DEFAULT_CENTER[1] + self.DEMO_ORBIT_RADIUS * math.cos(t)
            
            self.update()
    
    def load_tiles_around_center(self, radius=1):
        center_x, center_y = self.deg2num(self.center_lat, self.center_lon, self.zoom)
        
        tile_requests = []
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                tile_x = center_x + dx
                tile_y = center_y + dy
                
                max_tile = 2 ** self.zoom
                if 0 <= tile_x < max_tile and 0 <= tile_y < max_tile:
                    if (self.zoom, tile_x, tile_y) not in self.tiles:
                        tile_requests.append((self.zoom, tile_x, tile_y))
        
        if tile_requests:
            threading.Thread(
                target=self.downloader.download_tiles_batch, 
                args=(tile_requests,),
                daemon=True
            ).start()
    
    def on_tile_downloaded(self, zoom, x, y, pixmap):
        if zoom == self.zoom:
            self.tiles[(zoom, x, y)] = pixmap
            self.update()
    
    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.LeftButton:
            self.is_dragging = True
            self.last_mouse_pos = event.position().toPoint()
            self.setCursor(Qt.ClosedHandCursor)
    
    def mouseMoveEvent(self, event: QMouseEvent):
        if self.is_dragging:
            current_pos = event.position().toPoint()
            dx = current_pos.x() - self.last_mouse_pos.x()
            dy = current_pos.y() - self.last_mouse_pos.y()
            
            lat_change, lon_change = self.pixels_to_degrees(dx, dy)
            
            self.center_lat = max(-85, min(85, self.center_lat - lat_change))
            self.center_lon = max(-180, min(180, self.center_lon - lon_change))
            
            self.last_mouse_pos = current_pos
            self.update()
    
    def mouseReleaseEvent(self, event: QMouseEvent):
        if event.button() == Qt.LeftButton:
            self.is_dragging = False
            self.setCursor(Qt.ArrowCursor)
            QTimer.singleShot(300, lambda: self.load_tiles_around_center(radius=2))
    
    def wheelEvent(self, event: QWheelEvent):
        mouse_x = event.position().x() - self.width() / 2
        mouse_y = event.position().y() - self.height() / 2
        
        lat_change, lon_change = self.pixels_to_degrees(mouse_x, mouse_y)
        mouse_lat = self.center_lat + lat_change
        mouse_lon = self.center_lon + lon_change
        
        if event.angleDelta().y() > 0:
            self.zoom = min(18, self.zoom + 1)
        else:
            self.zoom = max(1, self.zoom - 1)
        
        lat_change_new, lon_change_new = self.pixels_to_degrees(mouse_x, mouse_y)
        self.center_lat = max(-85, min(85, mouse_lat - lat_change_new))
        self.center_lon = max(-180, min(180, mouse_lon - lon_change_new))
        
        self.tiles.clear()
        self.load_tiles_around_center(radius=3)
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), Qt.lightGray)
        
        painter.setPen(Qt.black)
        painter.setFont(QFont("Arial", 12))
        painter.drawText(10, 25, f"Center: {self.center_lat:.6f}, {self.center_lon:.6f}")
        painter.drawText(10, 45, f"Zoom: {self.zoom}")
        painter.drawText(10, 65, f"Tiles cached: {len(self.tiles)}")
        painter.drawText(10, 85, "Left-click and drag to pan, scroll wheel to zoom")
        
        if len(self.tiles) == 0:
            painter.drawText(10, 105, "Loading tiles...")
            self._draw_demo_drone_arrow(painter)
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
        
        self._draw_demo_drone_arrow(painter)
        
        painter.setPen(Qt.blue)
        painter.drawLine(widget_center_x - 20, widget_center_y, widget_center_x + 20, widget_center_y)
        painter.drawLine(widget_center_x, widget_center_y - 20, widget_center_x, widget_center_y + 20)
    
    def _draw_demo_drone_arrow(self, painter):
        lat_diff = self.drone_lat - self.center_lat
        lon_diff = self.drone_lon - self.center_lon
        
        lat_change, lon_change = self.pixels_to_degrees(1, 1)
        if lat_change != 0 and lon_change != 0:
            scale_y = 1.0 / lat_change
            scale_x = 1.0 / lon_change
            
            drone_x = self.width() // 2 + int(lon_diff * scale_x)
            drone_y = self.height() // 2 - int(lat_diff * scale_y)
            
            drone_x = max(50, min(self.width() - 50, drone_x))
            drone_y = max(50, min(self.height() - 50, drone_y))
            
            self._draw_arrow_at_position(painter, drone_x, drone_y, self.drone_heading)
        else:
            self._draw_arrow_at_position(painter, self.width() // 2, self.height() // 2, self.drone_heading)
    
    def _draw_arrow_at_position(self, painter, x, y, heading_deg):
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
        shadow_arrow = QPolygonF([QPointF(p.x() + 2, p.y() + 2) for p in arrow])
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
        drone_text = "DEMO DRONE"
        drone_width = painter.fontMetrics().horizontalAdvance(drone_text)
        painter.fillRect(x - drone_width//2 - 2, label_y - text_height, 
                        drone_width + 4, text_height + 2, QColor(255, 255, 255, 200))
        painter.setPen(QColor(255, 0, 0))
        painter.drawText(x - drone_width//2, label_y, drone_text)
    
    def zoom_in(self):
        if self.zoom < 18:
            self.zoom += 1
            self.tiles.clear()
            self.load_tiles_around_center(radius=2)
    
    def zoom_out(self):
        if self.zoom > 1:
            self.zoom -= 1
            self.tiles.clear()
            self.load_tiles_around_center(radius=2)


class SimpleMapWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenStreetMap Demo - Drone Arrow")
        self.setGeometry(100, 100, 900, 700)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout(central_widget)
        
        info_label = QLabel("Demo: Rotating drone arrow - Red arrow shows drone position and heading")
        info_label.setStyleSheet("font-weight: bold; color: blue;")
        layout.addWidget(info_label)
        
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
        
        self.map_widget = SimpleMapWidget()
        
        layout.addLayout(controls)
        layout.addWidget(self.map_widget)
        
        zoom_in_btn.clicked.connect(self.map_widget.zoom_in)
        zoom_out_btn.clicked.connect(self.map_widget.zoom_out)
        reload_btn.clicked.connect(self.reload_tiles)
        toggle_demo_btn.clicked.connect(self.toggle_demo)
        
    def toggle_demo(self):
        self.map_widget.demo_mode = not self.map_widget.demo_mode
        if self.map_widget.demo_mode:
            self.map_widget.demo_timer.start()
            logger.info("Demo mode ON")
        else:
            self.map_widget.demo_timer.stop()
            logger.info("Demo mode OFF")
        
    def reload_tiles(self):
        self.map_widget.tiles.clear()
        self.map_widget.load_tiles_around_center()


def main():
    app = QApplication(sys.argv)
    
    try:
        response = requests.get("https://tile.openstreetmap.org/0/0/0.png", timeout=5)
        logger.info(f"Internet test: HTTP {response.status_code}")
    except Exception as e:
        logger.warning(f"Internet connection failed: {e}")
        logger.info("Demo will work without map tiles")
    
    window = SimpleMapWindow()
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()