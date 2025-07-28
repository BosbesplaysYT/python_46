import sys
import asyncio
import time
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QListWidget, QTextEdit, QLabel,
    QSlider, QComboBox
)
from PyQt6.QtCore import Qt, QTimer
from bleak import BleakScanner, BleakClient
import qasync
import pygame

# BuWizz 2.0 BLE constants
BUWIZZ_SERVICE_UUID = "936e67b1-1999-b388-144f-b740000054e"
DATA_CHAR_UUID       = "000092d1-0000-1000-8000-00805f9b34fb"

class BuWizzApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BuWizz 2.0 Controller")
        self.resize(500, 850)

        self.devices = {}
        self.client = None

        layout = QVBoxLayout()

        # 1) Scan & Connect
        layout.addWidget(QLabel("→ 1) Scan & Connect"))
        self.scan_btn = QPushButton("Scan for BuWizz")
        self.scan_btn.clicked.connect(self.on_scan)
        layout.addWidget(self.scan_btn)
        self.device_list = QListWidget()
        layout.addWidget(self.device_list)
        hc = QHBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.on_connect)
        hc.addWidget(self.connect_btn)
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.on_disconnect)
        self.disconnect_btn.setEnabled(False)
        hc.addWidget(self.disconnect_btn)
        layout.addLayout(hc)

        # 2) Quick Motor 1 Test
        layout.addWidget(QLabel("→ 2) Quick Motor 1 Test"))
        self.test1_btn = QPushButton("Run Motor 1 for 1 s")
        self.test1_btn.clicked.connect(self.on_test1)
        self.test1_btn.setEnabled(False)
        layout.addWidget(self.test1_btn)

        # 3) Power Level Selector
        layout.addWidget(QLabel("→ 3) Power Level"))
        pl_layout = QHBoxLayout()
        self.power_combo = QComboBox()
        self.power_combo.addItems(["Disabled", "Slow", "Normal", "Fast", "LDCRS"])
        self.power_combo.setCurrentIndex(2)  # default Normal
        self.power_combo.setEnabled(False)
        self.power_combo.currentIndexChanged.connect(self.on_power_changed)
        pl_layout.addWidget(QLabel("Power:"))
        pl_layout.addWidget(self.power_combo)
        layout.addLayout(pl_layout)

        # 4) Real‑Time Motor Sliders
        layout.addWidget(QLabel("→ 4) Real‑Time Motor Control"))
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
            layout.addLayout(row)

        # Reset Sliders Button
        self.reset_btn = QPushButton("Reset Sliders to 0")
        self.reset_btn.setEnabled(False)
        self.reset_btn.clicked.connect(self.on_reset_sliders)
        layout.addWidget(self.reset_btn)

        # 5) Joystick Control Section
        layout.addWidget(QLabel("→ 5) Joystick Control"))
        self.joystick_btn = QPushButton("Enable Joystick")
        self.joystick_btn.setCheckable(True)
        self.joystick_btn.clicked.connect(self.on_joystick_toggle)
        layout.addWidget(self.joystick_btn)
        self.js_status = QLabel("Status: Not connected")
        layout.addWidget(self.js_status)
        
        # Initialize joystick variables
        self.joystick = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_joystick)
        self.steering_gain = 0.1  # Adjust steering sensitivity

        # Log
        layout.addWidget(QLabel("Log:"))
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        layout.addWidget(self.log)

        self.setLayout(layout)

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
            right_power = -right_power
            
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
