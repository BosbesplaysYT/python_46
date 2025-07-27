import sys
import asyncio
import time
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QListWidget, QTextEdit, QLabel,
    QSlider, QComboBox, QGroupBox
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QImage, QPixmap
from bleak import BleakScanner, BleakClient
import qasync
import cv2
import numpy as np

# BuWizz 2.0 BLE constants
BUWIZZ_SERVICE_UUID = "936e67b1-1999-b388-144f-b740000054e"
DATA_CHAR_UUID = "000092d1-0000-1000-8000-00805f9b34fb"

class BuWizzApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BuWizz 2.0 Controller")
        self.resize(1000, 850)  # Wider window

        self.devices = {}
        self.client = None

        # Motion detection variables
        self.motion_detection_active = False
        self.prev_frame = None
        self.motion_threshold = 5000  # Default motion sensitivity
        self.trigger_cooldown = False

        # Main layout (horizontal)
        main_layout = QHBoxLayout()

        # Left column (existing controls)
        left_layout = QVBoxLayout()

        # 1) Scan & Connect
        left_layout.addWidget(QLabel("→ 1) Scan & Connect"))
        self.scan_btn = QPushButton("Scan for BuWizz")
        self.scan_btn.clicked.connect(self.on_scan)
        left_layout.addWidget(self.scan_btn)
        self.device_list = QListWidget()
        left_layout.addWidget(self.device_list)
        hc = QHBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.on_connect)
        hc.addWidget(self.connect_btn)
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.on_disconnect)
        self.disconnect_btn.setEnabled(False)
        hc.addWidget(self.disconnect_btn)
        left_layout.addLayout(hc)

        # 2) Quick Motor 1 Test
        left_layout.addWidget(QLabel("→ 2) Quick Motor 1 Test"))
        self.test1_btn = QPushButton("Run Motor 1 for 1 s")
        self.test1_btn.clicked.connect(self.on_test1)
        self.test1_btn.setEnabled(False)
        left_layout.addWidget(self.test1_btn)

        # 3) Power Level Selector
        left_layout.addWidget(QLabel("→ 3) Power Level"))
        pl_layout = QHBoxLayout()
        self.power_combo = QComboBox()
        self.power_combo.addItems(["Disabled", "Slow", "Normal", "Fast", "LDCRS"])
        self.power_combo.setCurrentIndex(2)  # default Normal
        self.power_combo.setEnabled(False)
        self.power_combo.currentIndexChanged.connect(self.on_power_changed)
        pl_layout.addWidget(QLabel("Power:"))
        pl_layout.addWidget(self.power_combo)
        left_layout.addLayout(pl_layout)

        # 4) Real-Time Motor Sliders
        left_layout.addWidget(QLabel("→ 4) Real-Time Motor Control"))
        self.sliders = []
        for i in range(1, 5):
            row = QHBoxLayout()
            row.addWidget(QLabel(f"Motor {i}"))
            sld = QSlider(Qt.Orientation.Horizontal)
            sld.setRange(-127, 127)
            sld.setValue(0)
            sld.setEnabled(False)
            sld.valueChanged.connect(self.on_slider_changed)
            row.addWidget(sld, stretch=1)
            val_lbl = QLabel("0")
            row.addWidget(val_lbl)
            self.sliders.append((sld, val_lbl))
            left_layout.addLayout(row)

        # Reset Sliders Button
        self.reset_btn = QPushButton("Reset Sliders to 0")
        self.reset_btn.setEnabled(False)
        self.reset_btn.clicked.connect(self.on_reset_sliders)
        left_layout.addWidget(self.reset_btn)

        # Log
        left_layout.addWidget(QLabel("Log:"))
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        left_layout.addWidget(self.log)

        # Right column (new motion detection section)
        right_layout = QVBoxLayout()
        motion_group = QGroupBox("→ Motion Detection Control")
        motion_layout = QVBoxLayout()
        
        # Motion threshold control
        th_layout = QHBoxLayout()
        th_layout.addWidget(QLabel("Motion Sensitivity:"))
        self.threshold_slider = QSlider(Qt.Orientation.Horizontal)
        self.threshold_slider.setRange(1000, 20000)
        self.threshold_slider.setValue(5000)
        self.threshold_slider.setTickInterval(1000)
        self.threshold_slider.valueChanged.connect(self.on_threshold_changed)
        th_layout.addWidget(self.threshold_slider)
        self.threshold_label = QLabel("5000")
        th_layout.addWidget(self.threshold_label)
        motion_layout.addLayout(th_layout)

        # Camera feed display
        self.camera_label = QLabel("Camera Feed")
        self.camera_label.setFixedSize(320, 240)
        self.camera_label.setStyleSheet("background-color: black;")
        motion_layout.addWidget(self.camera_label)

        # Motion status
        motion_layout.addWidget(QLabel("Motion Status:"))
        self.motion_status = QLabel("INACTIVE")
        self.motion_status.setStyleSheet("font-weight: bold; color: gray;")
        motion_layout.addWidget(self.motion_status)

        # Start/stop detection
        btn_layout = QHBoxLayout()
        self.start_motion_btn = QPushButton("Start Motion Detection")
        self.start_motion_btn.clicked.connect(self.start_motion_detection)
        btn_layout.addWidget(self.start_motion_btn)
        
        self.stop_motion_btn = QPushButton("Stop Motion Detection")
        self.stop_motion_btn.clicked.connect(self.stop_motion_detection)
        self.stop_motion_btn.setEnabled(False)
        btn_layout.addWidget(self.stop_motion_btn)
        motion_layout.addLayout(btn_layout)

        motion_group.setLayout(motion_layout)
        right_layout.addWidget(motion_group)
        right_layout.addStretch()

        # Add both columns to main layout
        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)
        self.setLayout(main_layout)

        # Camera setup
        self.capture = cv2.VideoCapture(0)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_camera_feed)

    # ── BLE scan ─────────────────────────────────────────────
    async def scan(self):
        self.device_list.clear()
        self.devices.clear()
        self.log.append("🔍 Scanning for BuWizz (5 s)…")
        def cb(dev, adv):
            name = dev.name or adv.local_name or "Unknown"
            uuids = adv.service_uuids or []
            if dev.address not in self.devices and (
                "buwizz" in name.lower() or BUWIZZ_SERVICE_UUID in uuids
            ):
                self.devices[dev.address] = dev
                self.log.append(f"  • {name} [{dev.address}]")
        scanner = BleakScanner(detection_callback=cb)
        await scanner.start(); await asyncio.sleep(5); await scanner.stop()
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
        # Stop motion detection if active
        if self.motion_detection_active:
            self.stop_motion_detection()

    # ── Notification handler ─────────────────────────────────
    def _on_notify(self, _, data: bytearray):
        self.log.append(f"📥 {data.hex()}")

    # ── Quick Motor 1 Test ──────────────────────────────────
    async def test1(self):
        await self.send_power_level(self.power_combo.currentIndex())
        pkt_on = bytes([0x10, 0x7F, 0, 0, 0, 0, 0, 0])
        await self._write(pkt_on); self.log.append("▶ Motor 1 ON")
        await asyncio.sleep(1.0)
        pkt_off = bytes([0x10, 0, 0, 0, 0, 0, 0, 0])
        await self._write(pkt_off); self.log.append("▶ Motor 1 OFF")

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
        def clamp(x): return max(-127, min(127, x)) & 0xFF
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

    # ── Motion Detection Methods ────────────────────────────
    def on_threshold_changed(self, value):
        self.motion_threshold = value
        self.threshold_label.setText(str(value))

    def start_motion_detection(self):
        if not self.client or not self.client.is_connected:
            self.log.append("⚠️ Connect to BuWizz first!")
            return
            
        self.motion_detection_active = True
        self.prev_frame = None
        self.start_motion_btn.setEnabled(False)
        self.stop_motion_btn.setEnabled(True)
        self.motion_status.setText("ACTIVE")
        self.motion_status.setStyleSheet("font-weight: bold; color: green;")
        self.timer.start(30)  # ~33 FPS
        self.log.append("▶ Motion detection started")

    def stop_motion_detection(self):
        self.motion_detection_active = False
        self.start_motion_btn.setEnabled(True)
        self.stop_motion_btn.setEnabled(False)
        self.motion_status.setText("INACTIVE")
        self.motion_status.setStyleSheet("font-weight: bold; color: gray;")
        self.timer.stop()
        # Display black screen
        self.camera_label.setPixmap(QPixmap(320, 240))
        self.camera_label.setStyleSheet("background-color: black;")
        self.log.append("⏹ Motion detection stopped")

    def update_camera_feed(self):
        ret, frame = self.capture.read()
        if not ret:
            return
            
        # Process frame for motion detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        
        motion_detected = False
        if self.prev_frame is not None:
            frame_diff = cv2.absdiff(self.prev_frame, gray)
            thresh = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)[1]
            thresh = cv2.dilate(thresh, None, iterations=2)
            
            # Calculate motion level
            motion_level = cv2.countNonZero(thresh)
            motion_detected = motion_level > self.motion_threshold
            
            # Visual feedback
            if motion_detected:
                cv2.putText(frame, "MOTION DETECTED!", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                # Trigger motor if not in cooldown
                if not self.trigger_cooldown:
                    self.trigger_motor_action()
            
            # Draw motion indicator
            cv2.putText(frame, f"Motion: {motion_level}", (10, frame.shape[0]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        self.prev_frame = gray
        
        # Display frame
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        self.camera_label.setPixmap(QPixmap.fromImage(qt_image))

    def trigger_motor_action(self):
        self.trigger_cooldown = True
        self.log.append("🚨 Motion detected! Triggering motor...")
        
        # Run motor at 100% power in normal mode
        asyncio.create_task(self.run_motion_trigger())

    async def run_motion_trigger(self):
        try:
            # Set power level to normal (index 2)
            await self.send_power_level(2)
            
            # Run motor 1 at full power (127)
            pkt_on = bytes([0x10, 0x7F, 0, 0, 0, 0, 0, 0])
            await self._write(pkt_on)
            
            # Wait 1 second
            await asyncio.sleep(1.0)
            
            # Stop motor
            pkt_off = bytes([0x10, 0, 0, 0, 0, 0, 0, 0])
            await self._write(pkt_off)
            
            self.log.append("✅ Motor action completed")
        except Exception as e:
            self.log.append(f"❌ Motor trigger failed: {e}")
        finally:
            # Reset cooldown after 1 second
            await asyncio.sleep(1.0)
            self.trigger_cooldown = False

    # ── Qt slots ────────────────────────────────────────────
    def on_scan(self):      asyncio.create_task(self.scan())
    def on_connect(self):   asyncio.create_task(self.connect())
    def on_disconnect(self):asyncio.create_task(self.disconnect())
    def on_test1(self):     asyncio.create_task(self.test1())
    
    def closeEvent(self, event):
        # Clean up resources
        if self.motion_detection_active:
            self.stop_motion_detection()
        if self.capture.isOpened():
            self.capture.release()
        event.accept()

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