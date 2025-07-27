# BuWizz 2.0 Python Controller

A cross‑platform Python GUI application to scan, connect and control BuWizz 2.0 Bluetooth LEGO®‑compatible hubs.  
Built with [Bleak](https://github.com/hbldh/bleak) for BLE and [PyQt6](https://www.riverbankcomputing.com/software/pyqt/) + [qasync](https://github.com/CabbageDevelopment/qasync) for an async UI.

---

## 🚀 Features

1. **Device discovery**  
   - Scans for BuWizz 2.0 hubs advertising the official service UUID or “BuWizz” in their name.  
2. **Real‑time motor control**  
   - Four sliders (–127…+127) for outputs 1–4.  
   - Sends combined `0x10 Set motor data` packets instantly on slider move.  
   - Automatically sets “Normal” power level (`0x11`) if needed.  
3. **Quick motor test**  
   - One‑click “Run Motor 1 for 1 s” demo.  
4. **Live status logging**  
   - Receives and displays periodic status packets (~25 Hz) from the hub.  
   - Raw hex notifications logged for debugging.  
5. **Clean connect/disconnect**  
   - Buttons to connect, enable notifications, and gracefully tear down the BLE link.

---
