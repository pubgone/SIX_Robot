# 🦖 Six-Legged Robot Firmware for ESP32-S3

> Real-time hexapod robot firmware for **ESP32-S3** using **PlatformIO**. Features serial servo control, kinematic solver, BLE & TCP remote control, and OTA updates — designed for agile, wireless, low-latency walking robots.

Perfect for makers, educators, and robotics developers building next-gen legged platforms with minimal wiring and maximum flexibility.

---

## 🧠 Features

- ✅ **Serial Servo Control** — Drives UART-based servos (e.g., LX-16A, STS series) via configurable UART bus
- ✅ **Kinematic Engine** — Real-time forward & inverse kinematics for precise foot positioning
- ✅ **Multiple Gaits** — Built-in Tripod, Wave, and Ripple gaits with adjustable stride, height, and speed
- ✅ **BLE Remote Control** — Connect via phone/tablet using standard BLE UART service
- ✅ **TCP Command Interface** — Send motion commands over WiFi via raw TCP socket (port 8080)
- ✅ **OTA Updates** — Upload new firmware wirelessly via WiFi (Web UI or PlatformIO CLI)
- ✅ **Unified Command Set** — Identical commands over Serial, BLE, and TCP
- ✅ **Low Latency & Resource Efficient** — Runs at 50–100Hz with <100KB RAM usage

---

## 📦 Hardware Requirements

- **Main Board**: ESP32-S3 
- **Servos**: 18x UART-controlled servos 
- **Power**: 2S–3S LiPo + 5V/3A+ regulator for servos; 3.3V LDO for ESP32-S3
- **Wiring**: Shared UART bus for servos 

---

## ⚙️ Software Setup

### 1. Install PlatformIO

Install [PlatformIO in VS Code](https://platformio.org/install/ide?install=vscode) or via CLI:
