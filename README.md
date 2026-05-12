# PneumaGe Firmware (v1.2.0)

Firmware for the PneumaGe open-source, double-walled static accumulation chamber. This firmware is optimized for the **Arduino Nano 33 BLE Sense**, featuring real-time sensor data acquisition, internal pump circulation, and BLE streaming to the PneumaGe mobile app.

---

## 🚀 Overview
The PneumaGe firmware manages high-precision soil gas flux monitoring with a focus on the Arduino Nano 33 BLE microcontroller. It handles sensor data acquisition, pump control via PWM, disturbance detection through an onboard IMU, and secure BLE communication with the companion mobile application.

## 🛠 Features
- **Arduino Nano 33 BLE Sense Optimized:** Tailored for the nRF52840 ARM Cortex-M4 processor.
- **GATT BLE Profile:** Stream real-time gas concentrations and system health to the PneumaGe mobile app.
- **Environmental Monitoring:** High-precision sensor fusion with onboard BMP390 (pressure) and SHT45 (humidity/temperature).
- **Safety Interrupts:** IMU-based tilt and bump detection to invalidate corrupted samples.
- **Peripheral Simulation Mode:** Develop and test offline with simulated sensor data.

## 📋 Hardware Requirements
- **Microcontroller:** Arduino Nano 33 BLE Sense (Rev 2 recommended).
- **Sensors:** 
  - SprintIR-W CO2 (NDIR gas sensor)
  - BMP390 (pressure, onboard)
  - SHT45 (humidity & temperature, onboard)
  - BMI270 (IMU, onboard)
- **Actuators:** BoxerPump 22KD (PWM-controlled via D1).
- **Chamber:** Double-walled accumulation chamber with PTFE (Teflon) interior lining.

## ⚙️ Installation & Setup

### Using Arduino IDE
1. **Install the Arduino Nano 33 BLE Board Package:**
   - Open Arduino IDE → Tools → Board Manager
   - Search for "Arduino Mbed OS Nano Boards" and install the latest version.

2. **Select Your Board:**
   - Tools → Board → Arduino Mbed OS Nano Boards → Arduino Nano 33 BLE

3. **Install Required Libraries:**
   - Sketch → Include Library → Manage Libraries
   - Install: `ArduinoBLE`, `Arduino_BMI270_BMM150`, `ArduinoJson`

4. **Configuration:**
   - Open `pgfw.ino` and adjust compile-time settings as needed.
   - Set `SIMULATE_PERIPHERALS` to `true` for offline testing.

5. **Build & Upload:**
   - Connect your Nano 33 BLE via USB.
   - Sketch → Upload (or Ctrl+U).

### Using VS Code + Arduino Extension (Alternative)
1. Install the Arduino Extension in VS Code.
2. Open this folder and select your board from the status bar.
3. Click the Upload button (→ arrow icon).

### Using arduino-cli (Command Line)
1. **Install arduino-cli:**
   ```bash
   curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
   ```

2. **Install the Arduino Nano 33 BLE Board Package:**
   ```bash
   arduino-cli core update-index
   arduino-cli core install arduino:mbed_nano
   ```

3. **Install Required Libraries:**
   ```bash
   arduino-cli lib install ArduinoBLE
   arduino-cli lib install Arduino_BMI270_BMM150
   arduino-cli lib install ArduinoJson
   ```

4. **Compile the Firmware:**
   ```bash
   arduino-cli compile --fqbn arduino:mbed_nano:nano33ble pgfw.ino
   ```

5. **Upload to Your Board:**
   ```bash
   arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:mbed_nano:nano33ble pgfw.ino
   ```
   *Note: Replace `/dev/ttyACM0` with your actual port. Use `arduino-cli board list` to detect connected boards.*

6. **Monitor Serial Output (Optional):**
   ```bash
   arduino-cli monitor -p /dev/ttyACM0
   ```

## 📡 BLE Characteristics
| Characteristic      | UUID    | Type         | Description                |
|:--------------------|:--------|:-------------|:---------------------------|
| **Gas Concentration**| `...0001` | Notify       | PPM values (CO2)           |
| **Chamber Stats**   | `...0002` | Read/Notify  | Temp, Pressure, Humidity   |
| **System Command**  | `...0003` | Write        | Hex control commands       |
| **System Status**   | `...0005` | Read/Notify  | Status flags (tilt, bump, timeout) |
| **Battery Life**    | `...0006` | Read/Notify  | State of charge (%)        |
| **Device Info**     | `...0007` | Notify       | Streamed device JSON       |

## 🧪 BLE Testing with nRF Connect

Use a generic BLE utility like **nRF Connect for Mobile** to validate firmware behavior before integrating with the PneumaGe app.

**Note on Heartbeat:** The firmware has a 30-second safety timeout (increased for easier debugging). To keep the device from auto-shutting down during testing, periodically send a heartbeat command (`170` as UINT8 to `...0003`).

| Test Case | Characteristic | Action in nRF Connect | Expected Result |
| :--- | :--- | :--- | :--- |
| **1. Connection** | N/A | Scan and connect to `PneumaGe-Sim`. | Successful connection, services discovered. |
| **2. Stream Device Info** | `...0007` (Device Info) | Tap **Subscribe** (triple down-arrows). | Stream of packets received, ending with zero-length packet. Combined data forms valid JSON. |
| **3. Stream Data** | `...0001`, `...0002` | Tap **Subscribe** for each. | `Gas Concentration` and `Chamber Stats` update every second. |
| **4. Start Measurement** | `...0003` (System Command) | Write `01` (UINT8). | `Gas Concentration` begins to increase. Serial shows `[BLE CMD] Received: 0x1`. |
| **5. Set Pump Speed** | `...0003` (System Command) | Write `18` (UINT8). | Serial shows `[BLE CMD] Received: 0x12` (HIGH). |
| **6. Stop Measurement** | `...0003` (System Command) | Write `00` (UINT8). | `Gas Concentration` decays to baseline. Serial shows `[BLE CMD] Received: 0x0`. |
| **7. Set Level** | `...0003` (System Command) | Write `48` (UINT8). | Serial shows IMU recalibrated. |
| **8. Test Tilt Alert** | `...0005` (System Status) | Subscribe, then tilt device >3°. | Notification `0x01` received. Serial shows "Chamber Tilted!". |
| **9. Test Bump Alert** | `...0005` (System Status) | Subscribe, then sharply tap device. | Notification `0x02` received. Serial shows "Bump Detected!". |
| **10. Heartbeat Timeout** | `...0005` (System Status) | Connect and wait >30s without command. | Notification `0x07` received. Serial shows "System auto-shutdown". |
| **11. Heartbeat Recovery** | `...0003` (System Command) | After timeout, write `170` (UINT8). | Serial shows "System recovered". Status notifies `0x00`. |

## 📝 Simulation Mode

For offline development and testing, the firmware includes a **Peripheral Simulation Mode**:

```cpp
#define SIMULATE_PERIPHERALS true
```

When enabled, the firmware generates synthetic sensor data:
- **CO2 Sensor:** Simulates rising concentrations during measurement, decay at baseline.
- **Environmental Sensors:** Stable values with gentle sinusoidal variation.
- **Battery:** Simulates drain, faster when pump is active.
- **Pump Current:** Nominal 200mA draw with noise.

To use real hardware sensors, set `SIMULATE_PERIPHERALS` to `false` and ensure all required libraries are installed.

## ⚖️ License
This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

Copyright 2026 Shereef Sayed. This open-source project is intended for the global research community.

---
*Developed by Shereef Sayed | PneumaGe*