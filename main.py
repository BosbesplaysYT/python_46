import sys
import asyncio
import time
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton,
    QListWidget, QTextEdit, QLabel, QHBoxLayout, QLineEdit
)
from bleak import BleakScanner, BleakClient
import qasync

# BuWizz 2.0 BLE constants
BUWIZZ_SERVICE_UUID = "936e67b1-1999-b388-144f-b740000054e"
DATA_CHAR_UUID       = "000092d1-0000-1000-8000-00805f9b34fb"

class BuWizzApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BuWizz 2.0 Controller")
        self.resize(450, 750)

        self.devices = {}
        self.client = None
        self._last_log = 0.0

        layout = QVBoxLayout()

        # ── Scan & Connect ───────────────────────────────────────
        layout.addWidget(QLabel("→ 1) Scan & Connect"))
        self.scan_btn   = QPushButton("Scan for BuWizz")
        self.scan_btn.clicked.connect(self.on_scan)
        layout.addWidget(self.scan_btn)

        self.device_list = QListWidget()
        layout.addWidget(self.device_list)

        hc = QHBoxLayout()
        self.connect_btn    = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.on_connect)
        hc.addWidget(self.connect_btn)

        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.on_disconnect)
        self.disconnect_btn.setEnabled(False)
        hc.addWidget(self.disconnect_btn)
        layout.addLayout(hc)

        # ── Pre‑built Motor Test ─────────────────────────────────────────
        layout.addWidget(QLabel("→ 2) Quick Motor 1 Test"))
        self.test1_btn = QPushButton("Run Motor 1 for 1 s")
        self.test1_btn.clicked.connect(self.on_test1)
        self.test1_btn.setEnabled(False)
        layout.addWidget(self.test1_btn)

        # ── Manual Hex Editor ────────────────────────────────────────
        layout.addWidget(QLabel("→ 3) Manual Packet Send"))
        self.hex_input = QLineEdit("10 7F 00 00 00 00 00 00")
        layout.addWidget(self.hex_input)
        self.send_hex_btn = QPushButton("Send Hex Packet")
        self.send_hex_btn.clicked.connect(self.on_send_hex)
        self.send_hex_btn.setEnabled(False)
        layout.addWidget(self.send_hex_btn)

        # ── Log ────────────────────────────────────────────────
        layout.addWidget(QLabel("Log:"))
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        layout.addWidget(self.log)

        self.setLayout(layout)


    async def scan(self):
        self.device_list.clear()
        self.devices.clear()
        self.log.append("🔍 Scanning for BuWizz (5 s)…")
        def cb(dev, adv):
            name = dev.name or adv.local_name or "Unknown"
            uuids = adv.service_uuids or []
            if dev.address not in self.devices and ("buwizz" in name.lower() or BUWIZZ_SERVICE_UUID in uuids):
                self.devices[dev.address] = dev
                self.log.append(f"  • {name} [{dev.address}] uuids={uuids}")
        scanner = BleakScanner(detection_callback=cb)
        await scanner.start(); await asyncio.sleep(5); await scanner.stop()
        for addr, d in self.devices.items():
            n = d.name or "BuWizz"
            self.device_list.addItem(f"{n} ({addr})")
        self.log.append(f"✅ Found {len(self.devices)} device(s).")


    async def connect(self):
        idx = self.device_list.currentRow()
        if idx < 0:
            self.log.append("⚠️ Select a device first.")
            return
        addr = list(self.devices.keys())[idx]
        self.log.append(f"🔗 Connecting to {addr}…")
        self.client = BleakClient(addr)
        await self.client.connect()
        self.log.append("✅ Connected.")
        # start notifications
        await self.client.start_notify(DATA_CHAR_UUID, self._on_notify)
        self.log.append("🔔 Notifications ON")
        # enable buttons
        self.disconnect_btn.setEnabled(True)
        self.test1_btn.setEnabled(True)
        self.send_hex_btn.setEnabled(True)
        self.connect_btn.setEnabled(False)

    async def disconnect(self):
        if self.client and self.client.is_connected:
            await self.client.stop_notify(DATA_CHAR_UUID)
            await self.client.disconnect()
            self.log.append("⛔ Disconnected.")
        self.client = None
        self.disconnect_btn.setEnabled(False)
        self.test1_btn.setEnabled(False)
        self.send_hex_btn.setEnabled(False)
        self.connect_btn.setEnabled(True)

    def _on_notify(self, _, data: bytearray):
        # throttle to 2 Hz
        now = time.time()
        if now - self._last_log > 0.5:
            self._last_log = now
            self.log.append(f"📥 {data.hex()}")

    async def test1(self):
        # Normal power
        await self._write(bytes([0x11, 0x02]))
        self.log.append("▶ Power=Normal")
        # Motor1 full forward
        pkt = bytes([0x10, 0x7F, 0, 0, 0, 0, 0, 0])
        await self._write(pkt)
        self.log.append("▶ Motor1 ON")
        await asyncio.sleep(1.0)
        # Stop
        pkt = bytes([0x10, 0,0,0,0,0,0,0])
        await self._write(pkt)
        self.log.append("▶ Motor1 OFF")

    async def send_hex(self, hexstr: str):
        try:
            parts = hexstr.strip().split()
            pkt = bytes(int(p, 16) for p in parts)
        except:
            self.log.append("❌ Invalid hex")
            return
        await self._write(pkt)
        self.log.append(f"▶ Sent [{hexstr}]")

    async def _write(self, data: bytes):
        # always write with response=True
        await self.client.write_gatt_char(DATA_CHAR_UUID, data, response=True)


    # ── Qt slots ────────────────────────────────────────────
    def on_scan(self):      asyncio.create_task(self.scan())
    def on_connect(self):   asyncio.create_task(self.connect())
    def on_disconnect(self):asyncio.create_task(self.disconnect())
    def on_test1(self):     asyncio.create_task(self.test1())
    def on_send_hex(self):  asyncio.create_task(self.send_hex(self.hex_input.text()))

def main():
    app = QApplication(sys.argv)
    loop = qasync.QEventLoop(app)
    asyncio.set_event_loop(loop)
    w = BuWizzApp(); w.show()
    with loop:
        loop.run_forever()

if __name__ == "__main__":
    main()
