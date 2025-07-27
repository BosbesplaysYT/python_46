import sys
import asyncio
import time
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QListWidget, QTextEdit, QLabel, QSlider
)
from PyQt6.QtCore import Qt
from bleak import BleakScanner, BleakClient
import qasync

# BuWizz 2.0 BLE constants
BUWIZZ_SERVICE_UUID = "936e67b1-1999-b388-144f-b740000054e"
DATA_CHAR_UUID       = "000092d1-0000-1000-8000-00805f9b34fb"

class BuWizzApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BuWizz 2.0 Controller")
        self.resize(500, 800)

        self.devices = {}
        self.client = None

        # ── Main layout ─────────────────────────────────────────
        layout = QVBoxLayout()

        # Step 1: Scan & Connect
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

        # Step 2: Quick Motor Test
        layout.addWidget(QLabel("→ 2) Quick Motor 1 Test"))
        self.test1_btn = QPushButton("Run Motor 1 for 1 s")
        self.test1_btn.clicked.connect(self.on_test1)
        self.test1_btn.setEnabled(False)
        layout.addWidget(self.test1_btn)

        # Step 3: Real-time Motor Sliders
        layout.addWidget(QLabel("→ 3) Real-Time Motor Control"))
        self.sliders = []
        for i in range(1, 5):
            row = QHBoxLayout()
            lbl = QLabel(f"Motor {i}")
            row.addWidget(lbl)
            sld = QSlider(Qt.Orientation.Horizontal)
            sld.setRange(-127, 127)
            sld.setValue(0)
            sld.setEnabled(False)
            sld.valueChanged.connect(self.on_slider_changed)
            row.addWidget(sld, stretch=1)
            val_lbl = QLabel("0")
            row.addWidget(val_lbl)
            # remember both slider and its label
            self.sliders.append((sld, val_lbl))
            layout.addLayout(row)

        # Log output
        layout.addWidget(QLabel("Log:"))
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        layout.addWidget(self.log)

        self.setLayout(layout)

    # ── BLE scan ─────────────────────────────────────────────
    async def scan(self):
        self.device_list.clear()
        self.devices.clear()
        self.log.append("🔍 Scanning for BuWizz (5 s)…")

        def cb(dev, adv):
            name = dev.name or adv.local_name or "Unknown"
            uuids = adv.service_uuids or []
            if dev.address not in self.devices and (
                "buwizz" in name.lower() or BUWIZZ_SERVICE_UUID in uuids
            ):
                self.devices[dev.address] = dev
                self.log.append(f"  • {name} [{dev.address}]")

        scanner = BleakScanner(detection_callback=cb)
        await scanner.start()
        await asyncio.sleep(5.0)
        await scanner.stop()

        for addr, d in self.devices.items():
            name = d.name or "BuWizz"
            self.device_list.addItem(f"{name} ({addr})")
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
        for s, lbl in self.sliders:
            s.setEnabled(False)
        self.connect_btn.setEnabled(True)

    # ── Notification handler ─────────────────────────────────
    def _on_notify(self, _, data: bytearray):
        # just log raw hex occasionally
        self.log.append(f"📥 {data.hex()}")

    # ── Quick Motor 1 Test ──────────────────────────────────
    async def test1(self):
        # set Normal power
        await self._write(bytes([0x11, 0x02]))
        self.log.append("▶ Power=Normal")
        # run motor 1
        pkt_on = bytes([0x10, 0x7F, 0, 0, 0, 0, 0, 0])
        await self._write(pkt_on)
        self.log.append("▶ Motor 1 ON")
        await asyncio.sleep(1.0)
        pkt_off = bytes([0x10, 0, 0, 0, 0, 0, 0, 0])
        await self._write(pkt_off)
        self.log.append("▶ Motor 1 OFF")

    # ── Real‑time slider handler ─────────────────────────────
    def on_slider_changed(self, _):
        # update labels and send combined motor data
        values = []
        for s, lbl in self.sliders:
            v = s.value()
            lbl.setText(str(v))
            values.append(v)
        # send packet
        asyncio.create_task(self.send_motor_data(*values))

    # ── Send motor + power packets ───────────────────────────
    async def send_motor_data(self, m1, m2, m3, m4, brake=0):
        # ensure power is at least Normal
        await self._write(bytes([0x11, 0x02]))
        # clamp -127..127 and build 8-byte packet
        def clamp(x): return max(-127, min(127, x)) & 0xFF
        pkt = bytes([
            0x10,
            clamp(m1), clamp(m2), clamp(m3), clamp(m4),
            brake & 0x0F,
            0x00, 0x00
        ])
        await self._write(pkt)
        self.log.append(f"▶ Mdata: [{m1},{m2},{m3},{m4}]")

    async def _write(self, data: bytes):
        # write with no response for speed
        if self.client and self.client.is_connected:
            await self.client.write_gatt_char(DATA_CHAR_UUID, data, False)

    # ── Qt Slots ────────────────────────────────────────────
    def on_scan(self):      asyncio.create_task(self.scan())
    def on_connect(self):   asyncio.create_task(self.connect())
    def on_disconnect(self):asyncio.create_task(self.disconnect())
    def on_test1(self):     asyncio.create_task(self.test1())

def main():
    app = QApplication(sys.argv)
    loop = qasync.QEventLoop(app)
    asyncio.set_event_loop(loop)
    w = BuWizzApp()
    w.show()
    with loop:
        loop.run_forever()

if __name__ == "__main__":
    main()
