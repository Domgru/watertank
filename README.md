# ESP32-CAM Automated Water Tank Controller

A production-ready IoT water tank level controller and automation system built on the ESP32-CAM platform. The system monitors water depth using an ultrasonic sensor, manages high-voltage pump relays dynamically via a volatile RAM state matrix, and offers live photo verification alongside a responsive English smartphone dashboard.

## 🚀 Key Features

* **Zero Flash Wear Architecture:** All dynamic operational counters (like fill cycles) are computed exclusively in volatile RAM. Configuration boundaries are fully hard-coded into the firmware ROM. The system performs **zero write operations** to the onboard flash memory during operation, extending the microchip lifetime indefinitely.
* **Electrical Boot Safety:** Utilizes a high-level triggered relay isolated on **GPIO 15** combined with software initialization guards. This eliminates dangerous "low-pulse power surges" during boot-up, preventing the pump from accidentally short-cycling when power returns.
* **Smart Signal Damping:** Features a 5-sample moving average noise filter combined with a localized 15-step warm-up transducer array to absorb line interference, cable resistance, and surface reflections.
* **Dual Safety Timeouts:** Includes an explicit timed **Post-Run duration** to bypass sensor blind zones and fully top off the reservoir, backed by a hard **Maximum Runtime safety cut-off** to prevent dry running or flooding in case of pipe bursts.
* **Visual Telemetry Dashboard:** Responsive web interface showing live water levels, persistent rolling history logs, locked configuration read-outs, and an explicit live photo capture feature to visually verify the tank state.

## 🔧 Hardware Configuration

| Component | ESP32-CAM Pin | Notes |
| :--- | :--- | :--- |
| **Pump Relay** | GPIO 15 | Configured with High-Level Jumper (Safe at boot) |
| **Ultrasonic TRIG** | GPIO 13 | Trigger Output Burst |
| **Ultrasonic ECHO** | GPIO 12 | Input (Requires 5V to 3.3V Voltage Divider) |
| **Onboard Flash LED**| GPIO 4 | Synchronized for low-light camera captures |

## 📦 Directory Structure

```text
watertank_control/
├── watertank_control.ino  # Main application logic & web endpoints
└── credentials.h             # Ignored via git; contains Wi-Fi secrets
```

## 🛠️ Installation & Secure Deployment

### 1. Hide Your Network Credentials
To ensure your private Wi-Fi passwords are never leaked to public repositories, create a file named `credentials.h` in the same directory as your main sketch:

```cpp
#ifndef CREDENTIALS_H
#define CREDENTIALS_H

const char* SECRET_SSID = "Your_WiFi_Name";
const char* SECRET_PASSWORD = "Your_Secret_Password";

#endif
```

### 2. Configure Your Local Git Filter
Add `credentials.h` to your project's `.gitignore` file before running your initial commit sequence:

```text
# Ignore local Wi-Fi credentials
watertank_control/credentials.h

# Arduino IDE compilation outputs
/build/
```

### 3. Firmware Customization
Since configurations are hard-coded into the firmware for absolute system stability and storage protection, customize your physical tank depth metrics at the top of the main file before flashing:

```cpp
const float limitEmpty = 76.0;         // Pump starts at 76 cm distance (Empty Tank)
const float limitFull  = 30.0;         // Post-run triggers at 30 cm distance (Water at Rim)
const unsigned long postRunDuration = 90000;  // 90 seconds top-off duration (ms)
```

## 📱 Web Interface

The system serves a responsive dashboard locally. Once connected to your local router (e.g. Jio network environment), access the panel via:
* **URL:** `http://192.168.29.19` (Static IP) or `http://watertank.local` (mDNS)

All input fields are explicitly flagged as `readonly` in the UI to prevent erratic configuration overwrites, providing a true data-diode visualization panel for remote status checking.

