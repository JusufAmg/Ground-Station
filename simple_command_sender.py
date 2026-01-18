import sys
import socket
import time
import serial.tools.list_ports
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QComboBox, QTextEdit
from PySide6.QtCore import Qt
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

class MAVLinkCommander(QWidget):
    """Simple MAVLink command sender"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MAVLink Command Sender")
        self.setGeometry(200, 200, 400, 300)
        
        # MAVLink connection
        self.connection = None
        self.target_system = 1    # Pixhawk system ID
        self.target_component = 1 # Autopilot component ID
        
        # Flight modes (ArduCopter)
        self.flight_modes = {
            'STABILIZE': 0,
            'ACRO': 1,
            'ALT_HOLD': 2,
            'AUTO': 3,
            'GUIDED': 4,
            'LOITER': 5,
            'RTL': 6,
            'CIRCLE': 7,
            'LAND': 9,
            'DRIFT': 11,
            'SPORT': 13,
            'FLIP': 14,
            'AUTOTUNE': 15,
            'POSHOLD': 16,
            'BRAKE': 17
        }
        
        self.setup_ui()
        
    def setup_ui(self):
        """Setup the user interface"""
        layout = QVBoxLayout(self)
        
        # Title
        title = QLabel("MAVLink Command Sender")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 18px; font-weight: bold; margin: 10px;")
        layout.addWidget(title)
        
        # Connection section
        conn_layout = QHBoxLayout()
        self.connect_btn = QPushButton("Connect to Pixhawk")
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        
        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(self.status_label)
        layout.addLayout(conn_layout)
        
        # Flight mode section
        mode_layout = QVBoxLayout()
        mode_label = QLabel("Flight Mode Commands:")
        mode_label.setStyleSheet("font-weight: bold; margin-top: 20px;")
        mode_layout.addWidget(mode_label)
        
        # Mode selector
        mode_selector_layout = QHBoxLayout()
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(list(self.flight_modes.keys()))
        self.mode_combo.setCurrentText('ALT_HOLD')
        
        self.set_mode_btn = QPushButton("Set Flight Mode")
        self.set_mode_btn.clicked.connect(self.set_flight_mode)
        self.set_mode_btn.setEnabled(False)
        
        mode_selector_layout.addWidget(QLabel("Mode:"))
        mode_selector_layout.addWidget(self.mode_combo)
        mode_selector_layout.addWidget(self.set_mode_btn)
        mode_layout.addLayout(mode_selector_layout)
        
        layout.addLayout(mode_layout)
        
        # Quick mode buttons
        quick_layout = QVBoxLayout()
        quick_label = QLabel("Quick Mode Changes:")
        quick_label.setStyleSheet("font-weight: bold; margin-top: 20px;")
        quick_layout.addWidget(quick_label)
        
        # Row 1
        row1 = QHBoxLayout()
        self.stabilize_btn = QPushButton("STABILIZE")
        self.althold_btn = QPushButton("ALT_HOLD")
        self.loiter_btn = QPushButton("LOITER")
        
        self.stabilize_btn.clicked.connect(lambda: self.quick_mode_change('STABILIZE'))
        self.althold_btn.clicked.connect(lambda: self.quick_mode_change('ALT_HOLD'))
        self.loiter_btn.clicked.connect(lambda: self.quick_mode_change('LOITER'))
        
        row1.addWidget(self.stabilize_btn)
        row1.addWidget(self.althold_btn)
        row1.addWidget(self.loiter_btn)
        quick_layout.addLayout(row1)
        
        # Row 2
        row2 = QHBoxLayout()
        self.rtl_btn = QPushButton("RTL (Return)")
        self.land_btn = QPushButton("LAND")
        self.guided_btn = QPushButton("GUIDED")
        
        self.rtl_btn.clicked.connect(lambda: self.quick_mode_change('RTL'))
        self.land_btn.clicked.connect(lambda: self.quick_mode_change('LAND'))
        self.guided_btn.clicked.connect(lambda: self.quick_mode_change('GUIDED'))
        
        # Style emergency buttons
        self.rtl_btn.setStyleSheet("background-color: orange; font-weight: bold;")
        self.land_btn.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        
        row2.addWidget(self.rtl_btn)
        row2.addWidget(self.land_btn)
        row2.addWidget(self.guided_btn)
        quick_layout.addLayout(row2)
        
        layout.addLayout(quick_layout)
        
        # Initially disable mode buttons
        self.set_mode_buttons_enabled(False)
        
        # Status area
        self.log_label = QLabel("Ready to connect...")
        self.log_label.setAlignment(Qt.AlignTop)
        self.log_label.setStyleSheet("border: 1px solid gray; padding: 10px; margin-top: 20px;")
        self.log_label.setWordWrap(True)
        layout.addWidget(self.log_label)
        
    def show_qgc_setup_info(self):
        """Show QGC setup instructions"""
        info = """
QGC Setup Required:
1. Start QGroundControl
2. Connect QGC to Pixhawk via USB
3. Go to: Application Settings → MAVLink
4. Enable "MAVLink forwarding"
5. Set Host: 127.0.0.1:14551
6. Click Apply/OK
7. Then try connecting this app

Current setup should be:
Pixhawk ← USB → QGC → UDP Forward → This App
        """
        self.log_label.setText(info.strip())
        
    def set_mode_buttons_enabled(self, enabled):
        """Enable/disable mode buttons"""
        self.set_mode_btn.setEnabled(enabled)
        self.stabilize_btn.setEnabled(enabled)
        self.althold_btn.setEnabled(enabled)
        self.loiter_btn.setEnabled(enabled)
        self.rtl_btn.setEnabled(enabled)
        self.land_btn.setEnabled(enabled)
        self.guided_btn.setEnabled(enabled)
        
    def toggle_connection(self):
        """Connect or disconnect from Pixhawk"""
        if self.connection is None:
            self.connect_to_pixhawk()
        else:
            self.disconnect_from_pixhawk()
            
    def connect_to_pixhawk(self):
        """Connect to Pixhawk via QGC UDP forward"""
        try:
            print("Connecting to QGC UDP forward on port 14551...")
            self.log_label.setText("Connecting to QGC UDP forward (port 14551)...")
            QApplication.processEvents()
            
            # Connect to QGC's UDP forward port
            self.connection = mavutil.mavlink_connection('udp:localhost:14551')
            
            # Wait for heartbeat to confirm connection
            print("Waiting for heartbeat from QGC forward...")
            msg = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            
            if msg:
                self.target_system = msg.get_srcSystem()
                self.target_component = msg.get_srcComponent()
                
                self.status_label.setText("Connected via QGC")
                self.status_label.setStyleSheet("color: green; font-weight: bold;")
                self.connect_btn.setText("Disconnect")
                self.set_mode_buttons_enabled(True)
                
                self.log_label.setText(f"✅ Connected via QGC UDP forward\nPort: 14551\nSystem {self.target_system}, Component {self.target_component}\n\nNote: Commands sent through QGC forward")
                print(f"Success! Connected via QGC UDP forward")
                print(f"Target: System {self.target_system}, Component {self.target_component}")
                return
                
            else:
                raise Exception("No heartbeat received from QGC forward.\n\nMake sure:\n1. QGC is running and connected to Pixhawk\n2. QGC MAVLink forwarding is enabled to 127.0.0.1:14551")
                
        except Exception as e:
            self.connection = None
            self.log_label.setText(f"❌ Connection failed:\n{str(e)}")
            print(f"Connection failed: {e}")
            
            # Show QGC setup instructions
            self.show_qgc_setup_info()
            
    def disconnect_from_pixhawk(self):
        """Disconnect from Pixhawk"""
        if self.connection:
            self.connection.close()
            self.connection = None
            
        self.status_label.setText("Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        self.connect_btn.setText("Connect to Pixhawk")
        self.set_mode_buttons_enabled(False)
        self.log_label.setText("Disconnected from Pixhawk")
        
    def set_flight_mode(self):
        """Set flight mode using dropdown selection"""
        selected_mode = self.mode_combo.currentText()
        self.send_mode_change(selected_mode)
        
    def quick_mode_change(self, mode_name):
        """Quick mode change using buttons"""
        self.send_mode_change(mode_name)
        
    def send_mode_change(self, mode_name):
        """Send flight mode change command to Pixhawk"""
        if not self.connection:
            self.log_label.setText("Error: Not connected to Pixhawk!")
            return
            
        if mode_name not in self.flight_modes:
            self.log_label.setText(f"Error: Unknown flight mode {mode_name}")
            return
            
        try:
            custom_mode = self.flight_modes[mode_name]
            
            print(f"Sending mode change command: {mode_name} (mode {custom_mode})")
            
            # Correct base mode calculation
            base_mode = (mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | 
                        mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED |
                        mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
            
            print(f"Using base_mode: {base_mode} (should be 89)")
            
            # Method 1: Using SET_MODE message with correct base mode
            self.connection.mav.set_mode_send(
                self.target_system,     # Target system
                base_mode,              # Base mode (corrected)
                custom_mode             # Custom mode
            )
            print(f"Sent SET_MODE: target_system={self.target_system}, base_mode={base_mode}, custom_mode={custom_mode}")
            
            # Method 2: COMMAND_LONG with corrected parameters
            self.connection.mav.command_long_send(
                self.target_system,                    # Target system
                self.target_component,                 # Target component  
                mavlink.MAV_CMD_DO_SET_MODE,          # Command
                0,                                     # Confirmation
                base_mode,                             # Param 1: base mode (corrected)
                custom_mode,                           # Param 2: custom mode
                0, 0, 0, 0, 0                         # Params 3-7 (unused)
            )
            print(f"Sent COMMAND_LONG: target={self.target_system}.{self.target_component}, cmd={mavlink.MAV_CMD_DO_SET_MODE}, base_mode={base_mode}")
            
            self.log_label.setText(f"✅ Sent command: {mode_name} (mode {custom_mode})\nBase mode: {base_mode}\nCheck QGC flight mode display...")
            
            # Check acknowledgment
            self.check_command_ack()
            
        except Exception as e:
            self.log_label.setText(f"❌ Error sending command: {str(e)}")
            print(f"Error sending mode change: {e}")
            import traceback
            traceback.print_exc()
    
    def check_command_ack(self):
        """Check for command acknowledgment"""
        try:
            # Wait briefly for acknowledgment
            ack_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
            if ack_msg:
                result = ack_msg.result
                command = ack_msg.command
                
                result_text = {
                    mavlink.MAV_RESULT_ACCEPTED: "✅ ACCEPTED",
                    mavlink.MAV_RESULT_TEMPORARILY_REJECTED: "⚠️ TEMPORARILY_REJECTED", 
                    mavlink.MAV_RESULT_DENIED: "❌ DENIED",
                    mavlink.MAV_RESULT_UNSUPPORTED: "❌ UNSUPPORTED",
                    mavlink.MAV_RESULT_FAILED: "❌ FAILED",
                    mavlink.MAV_RESULT_IN_PROGRESS: "🔄 IN_PROGRESS"
                }.get(result, f"Unknown result: {result}")
                
                print(f"Command ACK: {result_text} for command {command}")
                current_text = self.log_label.text()
                self.log_label.setText(f"{current_text}\n\nCommand result: {result_text}")
            else:
                print("No command acknowledgment received")
                current_text = self.log_label.text()
                self.log_label.setText(f"{current_text}\n\nNo acknowledgment received (timeout)")
                
        except Exception as e:
            print(f"Error checking acknowledgment: {e}")
            
    def send_command_long_mode_change(self, custom_mode):
        """Alternative method using COMMAND_LONG"""
        try:
            self.connection.mav.command_long_send(
                self.target_system,        # Target system
                self.target_component,     # Target component
                mavlink.MAV_CMD_DO_SET_MODE,  # Command
                0,                         # Confirmation
                mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # Param 1: base mode
                custom_mode,               # Param 2: custom mode
                0, 0, 0, 0, 0             # Params 3-7 (unused)
            )
            print(f"Also sent COMMAND_LONG for mode {custom_mode}")
        except Exception as e:
            print(f"COMMAND_LONG method failed: {e}")


class CommandSenderWindow(QMainWindow):
    """Main window for command sender"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Command Sender")
        self.setGeometry(100, 100, 450, 400)
        
        # Create commander widget
        self.commander = MAVLinkCommander()
        self.setCentralWidget(self.commander)


def main():
    app = QApplication(sys.argv)
    
    window = CommandSenderWindow()
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()