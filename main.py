import sys
import asyncio
import time
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QListWidget, QTextEdit, QLabel,
    QSlider, QComboBox, QLineEdit, QSplitter, 
    QTabWidget, QFrame, QSizePolicy
)
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QImage, QPixmap, QPainter, QFont, QColor
from bleak import BleakScanner, BleakClient
import qasync
import pygame
import cv2
import numpy as np

# BuWizz 2.0 BLE constants
BUWIZZ_SERVICE_UUID = "936e67b1-1999-b388-144f-b740000054e"
DATA_CHAR_UUID       = "000092d1-0000-1000-8000-00805f9b34fb"

# Custom QLabel for status indicators
class StatusLabel(QLabel):
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setFixedHeight(20)
        self.set_status("disconnected")
        
    def set_status(self, status):
        self.status = status
        if status == "connected":
            self.setStyleSheet("background-color: #4CAF50; color: white; border-radius: 5px;")
        elif status == "disconnected":
            self.setStyleSheet("background-color: #F44336; color: white; border-radius: 5px;")
        elif status == "active":
            self.setStyleSheet("background-color: #2196F3; color: white; border-radius: 5px;")

class CameraThread(QThread):
    new_frame = pyqtSignal(np.ndarray)
    error_occurred = pyqtSignal(str)
    
    def __init__(self, url):
        super().__init__()
        self.url = url
        self.running = False
        self.frame_counter = 0
        self.frame_skip = 1  # Process every Nth frame
        self.last_frame = None

    def run(self):
        self.running = True
        cap = cv2.VideoCapture(self.url)
        
        if not cap.isOpened():
            self.error_occurred.emit("Failed to open camera stream")
            return
            
        # Set lower resolution if supported
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Request MJPEG stream if possible
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        
        # Reduce buffer size to minimize latency
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        while self.running:
            self.frame_counter += 1
            if self.frame_counter % self.frame_skip != 0:
                continue
                
            ret, frame = cap.read()
            if not ret:
                self.error_occurred.emit("Frame read error")
                break
                
            # Store frame without copy for reuse
            self.last_frame = frame
            self.new_frame.emit(frame)
        
        cap.release()

    def stop(self):
        self.running = False
        self.wait()

class BuWizzApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BuWizz FPV Controller")
        self.resize(1200, 800)
        self.setStyleSheet("""
            QWidget {
                background-color: #2c3e50;
                color: #ecf0f1;
                font-family: 'Segoe UI';
            }
            QPushButton {
                background-color: #3498db;
                color: white;
                border: none;
                border-radius: 4px;
                padding: 8px;
                min-height: 15px;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
            QPushButton:disabled {
                background-color: #7f8c8d;
            }
            QPushButton#danger {
                background-color: #e74c3c;
            }
            QPushButton#danger:hover {
                background-color: #c0392b;
            }
            QPushButton#success {
                background-color: #2ecc71;
            }
            QPushButton#success:hover {
                background-color: #27ae60;
            }
            QTabWidget::pane {
                border: 1px solid #34495e;
                background: #34495e;
                border-radius: 4px;
            }
            QTabBar::tab {
                background: #2c3e50;
                color: #bdc3c7;
                padding: 8px 15px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                border: 1px solid #34495e;
            }
            QTabBar::tab:selected {
                background: #3498db;
                color: white;
                border-bottom: 2px solid #3498db;
            }
            QListWidget, QTextEdit, QLineEdit, QComboBox {
                background-color: #34495e;
                border: 1px solid #2c3e50;
                border-radius: 4px;
                padding: 5px;
                color: #ecf0f1;
            }
            QSlider::groove:horizontal {
                height: 8px;
                background: #34495e;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #3498db;
                border: 1px solid #2980b9;
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
            QSlider::sub-page:horizontal {
                background: #3498db;
                border-radius: 4px;
            }
            QFrame {
                background-color: #34495e;
                border-radius: 8px;
                padding: 15px;
            }
            QLabel#title {
                font-size: 16px;
                font-weight: bold;
                color: #3498db;
                padding: 5px 0;
            }
            QLabel#section {
                font-size: 14px;
                font-weight: bold;
                padding: 5px 0;
                color: #3498db;
            }
        """)
        
        self.devices = {}
        self.client = None

        # Main layout
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)

        # Create a horizontal splitter for camera and controls
        splitter = QSplitter(Qt.Orientation.Horizontal)
        
        # Left panel - Camera view
        camera_frame = QFrame()
        camera_frame.setObjectName("cameraFrame")
        camera_frame.setStyleSheet("QFrame#cameraFrame { background-color: black; border-radius: 8px; }")
        camera_layout = QVBoxLayout(camera_frame)
        camera_layout.setContentsMargins(10, 10, 10, 10)
        camera_layout.setSpacing(10)
        
        # Camera title bar
        camera_title = QHBoxLayout()
        camera_title.addWidget(QLabel("FPV CAMERA FEED"))
        
        # Status indicators
        status_layout = QHBoxLayout()
        self.camera_status = StatusLabel("CAMERA: OFF")
        self.buwizz_status = StatusLabel("BUWIZZ: DISCONNECTED")
        status_layout.addWidget(self.camera_status)
        status_layout.addWidget(self.buwizz_status)
        camera_title.addLayout(status_layout)
        camera_layout.addLayout(camera_title)
        
        # Video display
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setStyleSheet("background-color: black;")
        camera_layout.addWidget(self.video_label, 1)
        
        # Camera controls
        cam_control_layout = QHBoxLayout()
        
        # Camera URL input
        url_layout = QVBoxLayout()
        url_layout.addWidget(QLabel("Camera URL:"))
        self.camera_url = QLineEdit("http://192.168.178.93:8080/video")
        url_layout.addWidget(self.camera_url)
        cam_control_layout.addLayout(url_layout)
        
        # Camera control buttons
        self.cam_start_btn = QPushButton("Start Stream")
        self.cam_start_btn.setObjectName("success")
        self.cam_start_btn.clicked.connect(self.start_camera_stream)
        cam_control_layout.addWidget(self.cam_start_btn)
        
        self.cam_stop_btn = QPushButton("Stop Stream")
        self.cam_stop_btn.setObjectName("danger")
        self.cam_stop_btn.clicked.connect(self.stop_camera_stream)
        self.cam_stop_btn.setEnabled(False)
        cam_control_layout.addWidget(self.cam_stop_btn)
        
        # Performance controls
        quality_layout = QVBoxLayout()
        quality_layout.addWidget(QLabel("Quality:"))
        self.quality_combo = QComboBox()
        self.quality_combo.addItems(["Low (15fps)", "Medium (20fps)", "High (30fps)"])
        self.quality_combo.setCurrentIndex(1)
        quality_layout.addWidget(self.quality_combo)
        cam_control_layout.addLayout(quality_layout)
        
        camera_layout.addLayout(cam_control_layout)
        
        # Right panel - Control tabs
        tabs_frame = QFrame()
        tabs_layout = QVBoxLayout(tabs_frame)
        tabs_layout.setContentsMargins(0, 0, 0, 0)
        
        self.tab_widget = QTabWidget()
        self.tab_widget.setStyleSheet("QTabWidget { background: transparent; }")
        
        # Connection Tab
        connection_tab = QWidget()
        connection_layout = QVBoxLayout(connection_tab)
        connection_layout.setSpacing(15)
        
        # Connection section
        conn_frame = QFrame()
        conn_frame.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        conn_layout = QVBoxLayout(conn_frame)
        conn_layout.setContentsMargins(15, 15, 15, 15)
        
        conn_title = QLabel("BUWIZZ CONNECTION")
        conn_title.setObjectName("title")
        conn_layout.addWidget(conn_title)
        
        # Scan button at top
        scan_layout = QHBoxLayout()
        self.scan_btn = QPushButton("Scan for BuWizz")
        self.scan_btn.setObjectName("success")
        self.scan_btn.clicked.connect(self.on_scan)
        scan_layout.addWidget(self.scan_btn)
        conn_layout.addLayout(scan_layout)
        
        # Device list
        self.device_list = QListWidget()
        self.device_list.setFixedHeight(80)  # Reduced height
        conn_layout.addWidget(self.device_list)
        
        # Connect buttons
        conn_btn_layout = QHBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setObjectName("success")
        self.connect_btn.clicked.connect(self.on_connect)
        conn_btn_layout.addWidget(self.connect_btn)
        
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setObjectName("danger")
        self.disconnect_btn.clicked.connect(self.on_disconnect)
        self.disconnect_btn.setEnabled(False)
        conn_btn_layout.addWidget(self.disconnect_btn)
        conn_layout.addLayout(conn_btn_layout)
        
        # Power control
        power_layout = QHBoxLayout()
        power_layout.addWidget(QLabel("Power Level:"))
        self.power_combo = QComboBox()
        self.power_combo.addItems(["Disabled", "Slow", "Normal", "Fast", "LDCRS"])
        self.power_combo.setCurrentIndex(2)  # default Normal
        self.power_combo.setEnabled(False)
        self.power_combo.currentIndexChanged.connect(self.on_power_changed)
        power_layout.addWidget(self.power_combo, 1)
        conn_layout.addLayout(power_layout)
        
        # Test button
        test_layout = QHBoxLayout()
        self.test1_btn = QPushButton("Test Motor 1")
        self.test1_btn.setObjectName("success")
        self.test1_btn.clicked.connect(self.on_test1)
        self.test1_btn.setEnabled(False)
        test_layout.addWidget(self.test1_btn)
        conn_layout.addLayout(test_layout)
        
        connection_layout.addWidget(conn_frame)
        
        # Joystick section
        joy_frame = QFrame()
        joy_layout = QVBoxLayout(joy_frame)
        joy_layout.setContentsMargins(15, 15, 15, 15)
        
        joy_title = QLabel("JOYSTICK CONTROL")
        joy_title.setObjectName("title")
        joy_layout.addWidget(joy_title)
        
        self.joystick_btn = QPushButton("Enable Joystick")
        self.joystick_btn.setCheckable(True)
        self.joystick_btn.setObjectName("success")
        self.joystick_btn.clicked.connect(self.on_joystick_toggle)
        joy_layout.addWidget(self.joystick_btn)
        
        self.js_status = QLabel("Status: Not connected")
        self.js_status.setStyleSheet("font-style: italic;")
        joy_layout.addWidget(self.js_status)
        
        connection_layout.addWidget(joy_frame, 1)
        self.tab_widget.addTab(connection_tab, "Connection")
        
        # Motors Tab
        motors_tab = QWidget()
        motors_layout = QVBoxLayout(motors_tab)
        motors_layout.setSpacing(15)
        
        # Motor sliders
        motors_frame = QFrame()
        motors_frame_layout = QVBoxLayout(motors_frame)
        motors_frame_layout.setContentsMargins(15, 15, 15, 15)
        
        motors_title = QLabel("MOTOR CONTROL")
        motors_title.setObjectName("title")
        motors_frame_layout.addWidget(motors_title)
        
        self.sliders = []
        for i in range(1, 5):
            motor_layout = QVBoxLayout()
            motor_layout.addWidget(QLabel(f"MOTOR {i}"))
            
            slider_layout = QHBoxLayout()
            sld = QSlider(Qt.Orientation.Horizontal)
            sld.setRange(-127, 127)
            sld.setValue(0)
            sld.setEnabled(False)
            sld.valueChanged.connect(self.on_slider_changed)
            slider_layout.addWidget(sld, 1)
            
            val_lbl = QLabel("0")
            val_lbl.setFixedWidth(30)
            val_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            slider_layout.addWidget(val_lbl)
            
            motor_layout.addLayout(slider_layout)
            motors_frame_layout.addLayout(motor_layout)
            self.sliders.append((sld, val_lbl))
        
        # Reset button
        self.reset_btn = QPushButton("Reset All Motors")
        self.reset_btn.setObjectName("danger")
        self.reset_btn.setEnabled(False)
        self.reset_btn.clicked.connect(self.on_reset_sliders)
        motors_frame_layout.addWidget(self.reset_btn)
        
        motors_layout.addWidget(motors_frame)
        self.tab_widget.addTab(motors_tab, "Motors")
        
        tabs_layout.addWidget(self.tab_widget)
        
        # Add to splitter
        splitter.addWidget(camera_frame)
        splitter.addWidget(tabs_frame)
        splitter.setSizes([700, 300])
        
        main_layout.addWidget(splitter, 1)
        
        # Log section
        log_frame = QFrame()
        log_layout = QVBoxLayout(log_frame)
        log_layout.setContentsMargins(15, 15, 15, 15)
        
        log_title = QLabel("SYSTEM LOG")
        log_title.setObjectName("title")
        log_layout.addWidget(log_title)
        
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMinimumHeight(100)
        log_layout.addWidget(self.log)
        
        main_layout.addWidget(log_frame)
        
        self.setLayout(main_layout)

        # Initialize joystick variables
        self.joystick = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_joystick)
        self.steering_gain = 0.1  # Adjust steering sensitivity

        # Camera variables
        self.camera_thread = None
        self.last_frame_time = 0

    # Camera stream functions
    def start_camera_stream(self):
        if not self.camera_url.text():
            self.log.append("⚠️ Enter camera URL first")
            return
            
        if self.camera_thread and self.camera_thread.isRunning():
            self.stop_camera_stream()
            
        self.camera_thread = CameraThread(self.camera_url.text())
        
        # Configure based on quality selection
        quality = self.quality_combo.currentIndex()
        if quality == 0:    # Low
            self.camera_thread.frame_skip = 2
        elif quality == 1:  # Medium
            self.camera_thread.frame_skip = 1
        else:               # High
            self.camera_thread.frame_skip = 0  # Process all frames
            
        self.camera_thread.new_frame.connect(self.update_frame, Qt.ConnectionType.QueuedConnection)
        self.camera_thread.error_occurred.connect(self.handle_camera_error)
        self.camera_thread.start()
        
        self.cam_start_btn.setEnabled(False)
        self.cam_stop_btn.setEnabled(True)
        self.camera_status.set_status("active")
        self.log.append("▶ Camera stream started")

    def stop_camera_stream(self):
        if self.camera_thread:
            self.camera_thread.stop()
            self.camera_thread = None
            
        self.cam_start_btn.setEnabled(True)
        self.cam_stop_btn.setEnabled(False)
        self.video_label.clear()
        self.video_label.setStyleSheet("background-color: black;")
        self.camera_status.set_status("disconnected")
        self.log.append("⏹ Camera stream stopped")

    @pyqtSlot(np.ndarray)
    def update_frame(self, frame):
        # Calculate actual FPS
        current_time = time.time()
        fps = 1 / (current_time - self.last_frame_time) if self.last_frame_time else 0
        self.last_frame_time = current_time
        
        # Only update display if we have >10 FPS or it's been >100ms since last update
        if fps < 10 and (current_time - self.last_frame_time) < 0.1:
            return
            
        # Convert to QImage without copying data
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        q_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        
        # Scale to display size while maintaining aspect ratio
        pixmap = QPixmap.fromImage(q_img).scaled(
            self.video_label.width(), 
            self.video_label.height(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )
        
        # Display FPS in corner
        painter = QPainter(pixmap)
        painter.setPen(QColor(Qt.GlobalColor.white))
        painter.setFont(QFont("Arial", 12))
        painter.drawText(10, 30, f"FPS: {fps:.1f}")
        painter.end()
        
        self.video_label.setPixmap(pixmap)

    @pyqtSlot(str)
    def handle_camera_error(self, message):
        self.log.append(f"❌ Camera Error: {message}")
        self.stop_camera_stream()

    # ── BLE scan ─────────────────────────────────────────────
    async def scan(self):
        self.device_list.clear()
        self.devices.clear()
        self.log.append("🔍 Scanning for BuWizz (3 s)…")
        def cb(dev, adv):
            name = dev.name or adv.local_name or "Unknown"
            uuids = adv.service_uuids or []
            if dev.address not in self.devices and (
                "buwizz" in name.lower() or BUWIZZ_SERVICE_UUID in uuids
            ):
                self.devices[dev.address] = dev
                self.log.append(f"  • {name} [{dev.address}]")
        scanner = BleakScanner(detection_callback=cb)
        await scanner.start(); await asyncio.sleep(3); await scanner.stop()
        for addr, d in self.devices.items():
            n = d.name or "BuWizz"
            self.device_list.addItem(f"{n} ({addr})")
        self.log.append(f"✅ Found {len(self.devices)} device(s).")

    # ── Connect / Disconnect ─────────────────────────────────
    async def connect(self):
        idx = self.device_list.currentRow()
        if idx < 0:
            self.log.append("⚠️ Select a device first.")
            return
        addr = list(self.devices.keys())[idx]
        self.log.append(f"🔗 Connecting to {addr}…")
        self.client = BleakClient(addr)
        try:
            await self.client.connect()
            self.log.append("✅ Connected.")
            await self.client.start_notify(DATA_CHAR_UUID, self._on_notify)
            self.log.append("🔔 Notifications ON")
            # enable controls
            self.disconnect_btn.setEnabled(True)
            self.test1_btn.setEnabled(True)
            self.power_combo.setEnabled(True)
            self.reset_btn.setEnabled(True)
            for s, lbl in self.sliders:
                s.setEnabled(True)
            self.connect_btn.setEnabled(False)
            self.buwizz_status.set_status("connected")
        except Exception as e:
            self.log.append(f"❌ Connect failed: {e}")

    async def disconnect(self):
        if self.client and self.client.is_connected:
            await self.client.stop_notify(DATA_CHAR_UUID)
            await self.client.disconnect()
            self.log.append("⛔ Disconnected.")
        self.client = None
        # disable controls
        self.disconnect_btn.setEnabled(False)
        self.test1_btn.setEnabled(False)
        self.power_combo.setEnabled(False)
        self.reset_btn.setEnabled(False)
        for s, lbl in self.sliders:
            s.setEnabled(False)
        self.connect_btn.setEnabled(True)
        self.buwizz_status.set_status("disconnected")

    # ── Notification handler ─────────────────────────────────
    def _on_notify(self, _, data: bytearray):
        self.log.append(f"📥 {data.hex()}")

    # ── Quick Motor 1 Test ──────────────────────────────────
    async def test1(self):
        await self.send_power_level(self.power_combo.currentIndex())
        pkt_on = bytes([0x10, 0x7F, 0, 0, 0, 0, 0, 0])
        await self._write(pkt_on); self.log.append("▶ Motor 1 ON")
        await asyncio.sleep(1.0)
        pkt_off = bytes([0x10, 0, 0, 0, 0, 0, 0, 0])
        await self._write(pkt_off); self.log.append("▶ Motor 1 OFF")

    # ── Power level command ─────────────────────────────────
    def on_power_changed(self, idx):
        if self.client and self.client.is_connected:
            asyncio.create_task(self.send_power_level(idx))

    async def send_power_level(self, level: int):
        """0x11 Set power level (0–4)."""
        pkt = bytes([0x11, level])
        await self._write(pkt)
        self.log.append(f"▶ Power level → {self.power_combo.itemText(level)}")

    # ── Slider handling ─────────────────────────────────────
    def on_slider_changed(self, _):
        # update labels
        values = []
        for s, lbl in self.sliders:
            v = s.value()
            lbl.setText(str(v))
            values.append(v)
        # send combined motor data
        asyncio.create_task(self.send_motor_data(*values))

    async def send_motor_data(self, m1, m2, m3, m4, brake=0):
        # clamp to −127…127
        def clamp(x): 
            x = int(round(max(-127, min(127, x))))
            return x & 0xFF
        pkt = bytes([
            0x10,
            clamp(m1), clamp(m2), clamp(m3), clamp(m4),
            brake & 0x0F,
            0x00, 0x00
        ])
        await self._write(pkt)
        self.log.append(f"▶ Motor data: [{m1},{m2},{m3},{m4}]")

    # ── Reset sliders ───────────────────────────────────────
    def on_reset_sliders(self):
        for s, lbl in self.sliders:
            s.blockSignals(True)
            s.setValue(0)
            lbl.setText("0")
            s.blockSignals(False)
        # send zero
        asyncio.create_task(self.send_motor_data(0,0,0,0))

    # ── Low-level write ─────────────────────────────────────
    async def _write(self, data: bytes):
        if self.client and self.client.is_connected:
            await self.client.write_gatt_char(DATA_CHAR_UUID, data, False)

    # ── Joystick Handling ────────────────────────────────────
    def init_joystick(self):
        try:
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.js_status.setText(f"Connected: {self.joystick.get_name()}")
                # Set axis mapping
                self.STEER_AXIS = 0  # X-axis for steering
                self.THROTTLE_AXIS = 1  # Y-axis for throttle (forward/reverse)
                return True
            self.js_status.setText("No joystick detected")
        except Exception as e:
            self.js_status.setText(f"Error: {str(e)}")
        return False

    def read_joystick(self):
        if not self.joystick or not self.client or not self.client.is_connected:
            return
            
        pygame.event.pump()  # Process event queue
        
        try:
            # Get axis values (-1.0 to 1.0)
            steering = self.joystick.get_axis(self.STEER_AXIS)
            throttle = self.joystick.get_axis(self.THROTTLE_AXIS)  # Invert so forward is up
            
            # Deadzone to prevent small movements from activating motors
            deadzone = 0.1
            if abs(throttle) < deadzone and abs(steering) < deadzone:
                self.update_motor_sliders(0, 0, 0, 0)
                asyncio.create_task(self.send_motor_data(0, 0, 0, 0))
                return
                
            # Calculate absolute speed factor (0-1)
            speed_factor = min(1.0, abs(throttle))
            
            # DYNAMIC STEERING GAIN: 
            # - High gain (0.5) at low speeds
            # - Low gain (0.1) at high speeds
            steering_gain = 0.5 - (0.4 * speed_factor)
            
            # Base power from throttle
            base_power = throttle * 127
            
            # Steering effect (differential power)
            steering_effect = steering * abs(base_power) * steering_gain
            
            # Calculate motor powers
            left_power = base_power + steering_effect
            right_power = base_power - steering_effect
            
            # Invert the right motor (since it's mounted opposite)
            left_power = -left_power
            
            # Clamp values to motor range
            left_power = max(-127, min(127, left_power))
            right_power = max(-127, min(127, right_power))
            
            # Convert to integers
            left_power = int(round(left_power))
            right_power = int(round(right_power))
            
            # Update sliders and send command
            self.update_motor_sliders(left_power, right_power, 0, 0)
            asyncio.create_task(self.send_motor_data(left_power, right_power, 0, 0))
            
            # Log values for debugging
            self.log.append(f"🎮 T:{throttle:.2f} S:{steering:.2f} Gain:{steering_gain:.2f} → L:{left_power} R:{right_power}")
            
        except Exception as e:
            self.log.append(f"🎮 Joystick error: {e}")

    def on_joystick_toggle(self, checked):
        if checked:
            if self.init_joystick():
                self.timer.start(30)  # 20Hz update
                self.log.append("🎮 Joystick enabled")
            else:
                self.joystick_btn.setChecked(False)
        else:
            self.timer.stop()
            if self.joystick:
                self.joystick.quit()
            self.js_status.setText("Status: Disabled")
            self.log.append("🎮 Joystick disabled")
            # Reset motors when disabling
            asyncio.create_task(self.send_motor_data(0, 0, 0, 0))


    def update_motor_sliders(self, m1, m2, m3, m4):
        """Update sliders without triggering send events"""
        # Block signals during update
        for sld, _ in self.sliders:
            sld.blockSignals(True)
        
        self.sliders[0][0].setValue(int(m1))
        self.sliders[1][0].setValue(int(m2))
        self.sliders[2][0].setValue(int(m3))
        self.sliders[3][0].setValue(int(m4))
        
        # Update labels
        self.sliders[0][1].setText(str(int(m1)))
        self.sliders[1][1].setText(str(int(m2)))
        self.sliders[2][1].setText(str(int(m3)))
        self.sliders[3][1].setText(str(int(m4)))
        
        # Re-enable signals
        for sld, _ in self.sliders:
            sld.blockSignals(False)

    # ── Qt slots ────────────────────────────────────────────
    def on_scan(self):      asyncio.create_task(self.scan())
    def on_connect(self):   asyncio.create_task(self.connect())
    def on_disconnect(self):asyncio.create_task(self.disconnect())
    def on_test1(self):     asyncio.create_task(self.test1())

def main():
    app = QApplication(sys.argv)
    loop = qasync.QEventLoop(app)
    asyncio.set_event_loop(loop)
    window = BuWizzApp()
    window.show()
    with loop:
        loop.run_forever()

if __name__ == "__main__":
    main()