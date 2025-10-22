# ITB de Labo - Sensor Module

![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)
![Platform](https://img.shields.io/badge/platform-ESP32-green.svg)
![Framework](https://img.shields.io/badge/framework-Arduino-teal.svg)

A multi-sensor data acquisition system built on ESP32 for monitoring temperature, humidity, extreme temperatures, and acceleration data with an intuitive OLED menu interface.

---


## Overview

The ITB de Labo Sensor Module is an ESP32-based environmental and motion monitoring system designed for laboratory and industrial applications. It features multiple sensors accessible through a menu-driven OLED interface with configurable debug modes for development and production use.

---

## Features

### Sensor Capabilities
- **DHT22 Sensor** - Temperature (-40 to 80°C) and Humidity (0-100%) monitoring
- **MAX6675 Thermocouple** - High-temperature measurements up to 1024°C
- **BNO055 IMU** - 9-axis motion tracking (accelerometer, gyroscope, magnetometer)
- **128x64 OLED Display** - Real-time data visualization
- **NeoPixel RGB LED** - Visual status feedback

---

## Hardware Requirements

### Core Components
| Component | Model | Quantity | Purpose |
|-----------|-------|----------|---------|
| Microcontroller | ESP32 DevKit C | 1 | Main processor |
| Temperature/Humidity Sensor | DHT22 (AM2302) | 1 | Environmental monitoring |
| Thermocouple Module | MAX6675 + K-Type TC | 1 | High-temp measurement |
| IMU Sensor | BNO055 (Adafruit/DFRobot) | 1 | Motion/acceleration |
| OLED Display | SSD1306 128x64 I2C | 1 | User interface |
| Push Buttons | Tactile switches | 3 | Navigation controls |



## Pin Configuration

### Complete Pin Mapping Table

| Pin Number | GPIO | Function | Component | Signal Type | Notes |
|------------|------|----------|-----------|-------------|-------|
| **Input Pins (Buttons)** |
| 1 | GPIO 32 | Button - SELECT | Tactile Switch | Digital Input (Pull-up) | Confirm/Enter |
| 2 | GPIO 36 (VP) | Button - DOWN | Tactile Switch | Digital Input (Pull-up) | Menu Down |
| 3 | GPIO 25 | Button - UP | Tactile Switch | Digital Input (Pull-up) | Menu Up |
| **Output Pins** |
| 4 | GPIO 16 | NeoPixel LED | WS2812B LED | Digital Output (Data) | Status indicator |
| 5 | GPIO 2 | Built-in LED | Onboard LED | Digital Output | System status |
| **I2C Bus** |
| 6 | GPIO 21 | I2C SDA | OLED + BNO055 | I2C Data | Shared bus |
| 7 | GPIO 22 | I2C SCL | OLED + BNO055 | I2C Clock | Shared bus |
| **SPI Bus (MAX6675)** |
| 8 | GPIO 14 | SPI SCK | MAX6675 | SPI Clock | Thermocouple |
| 9 | GPIO 12 | SPI MISO | MAX6675 | SPI Data In | Thermocouple |
| 10 | GPIO 13 | SPI MOSI | MAX6675 | SPI Data Out | Not used by MAX6675 |
| 11 | GPIO 15 | SPI CS | MAX6675 | Chip Select | Thermocouple |
| **Digital Sensor** |
| 12 | GPIO 33 | DHT22 Data | DHT22 | Digital I/O | Temp/Humidity |
| **Power Pins** |
| 13 | 3V3 | Power Out | All Sensors | +3.3V | Regulated output |
| 14 | 5V | Power In/Out | NeoPixel, DHT22 | +5V | From USB or Vin |
| 15 | GND | Ground | All Components | Ground | Common ground |

---

## Hardware Connections

### Pin Connection Diagram

#### **1. Button Connections**
```
ESP32                    Button
---------------------------------
GPIO 32 ---------->----- SELECT (+ 10kΩ to 3V3 optional)
GPIO 36 ---------->----- DOWN   (+ 10kΩ to 3V3 optional)
GPIO 25 ---------->----- UP     (+ 10kΩ to 3V3 optional)
GND   ----------<------- Common terminal of all buttons
```

**Note:** ESP32 internal pull-ups are enabled in code. External pull-up resistors are optional.

#### **2. DHT22 Temperature/Humidity Sensor**
```
DHT22 Pin          ESP32
---------------------------------
VCC (Pin 1)  ----> 5V or 3V3
DATA (Pin 2) ----> GPIO 33
NC   (Pin 3)       (Not Connected)
GND  (Pin 4) ----> GND
```


#### **3. MAX6675 Thermocouple Module**
```
MAX6675 Pin        ESP32
---------------------------------
VCC ----------> 5V or 3V3
GND ----------> GND
SCK ----------> GPIO 14 (SPI Clock)
CS  ----------> GPIO 15 (Chip Select)
SO  ----------> GPIO 12 (MISO - Data Out)

Thermocouple Connections:
+ (Positive) ---> Red wire to MAX6675 "+"
- (Negative) ---> Blue/Yellow wire to MAX6675 "-"
```

**Warning:** Ensure correct thermocouple polarity. Reversed connections will show incorrect temperatures.

#### **4. BNO055 IMU Sensor (I2C)**
```
BNO055 Pin         ESP32
---------------------------------
VIN ----------> 3V3 or 5V (check module spec)
GND ----------> GND
SDA ----------> GPIO 21 (I2C Data)
SCL ----------> GPIO 22 (I2C Clock)
```

**I2C Address:** Default is 0x28.

#### **5. SSD1306 OLED Display (I2C)**
```
OLED Pin           ESP32
---------------------------------
VCC ----------> 3V3
GND ----------> GND
SDA ----------> GPIO 21 (I2C Data - shared with BNO055)
SCL ----------> GPIO 22 (I2C Clock - shared with BNO055)
```

**I2C Address:** Typically 0x3C. 

---

### I2C Bus Configuration

The I2C bus is **shared** between the OLED display and BNO055 sensor:

```
ESP32 GPIO 21 (SDA) ----+---- OLED SDA
                        |
                        +---- BNO055 SDA

ESP32 GPIO 22 (SCL) ----+---- OLED SCL
                        |
                        +---- BNO055 SCL
```

## Debug Mode Configuration

The system supports two operational modes controlled by a single define:

### Location in Code
**File:** `src/main.cpp` (around line 30)

```cpp
// ===== DEBUG MODE CONFIGURATION =====
// Set to true to enable full debug logging
// Set to false to only print current menu data
#define DEBUG_MODE false    // <-- Change this line
// ====================================
```

### Debug Mode: ON (`DEBUG_MODE true`)

**Use Case:** Development, troubleshooting, system diagnostics

**Output Includes:**
- State machine transitions
- Sensor initialization status
- Detailed error messages
- All sensor readings with labels
- Button press events
- System timestamps

**Example Output:**
```
[SETUP] Starting setup...
[SETUP] Hardware initialized.
[MAIN_STATE] State: Intro
[MAIN_STATE] In introduction state.
[MAIN_STATE] State: Configuration
[MAIN_STATE] [MENU_1]
[MAIN_STATE] Humidity: 45.20%, Temperature: 23.50°C, MAX6675: 25.30°C
```

### Debug Mode: OFF (`DEBUG_MODE false`)

**Use Case:** Production, data logging, clean output

**Output:** Only current menu data (updates every 1 second)

**Menu 1 Output (Temp & Humidity):**
```
Temp: 23.50°C, Humidity: 45.20%
Temp: 23.52°C, Humidity: 45.18%
```

**Menu 2 Output (Acceleration):**
```
Ax: 0.12, Ay: -0.05, Az: 9.81
Ax: 0.13, Ay: -0.04, Az: 9.82
```

**Menu 3 Output (Extreme Temp):**
```
Extreme Temp: 150.25°C
Extreme Temp: 150.30°C
```

**Menu 4-6 Output:**
```
Environment: No data available
Settings: No data available
Reset: Standby
```

---

## PlatformIO Setup

### Step 1: Install PlatformIO

1. Open **Visual Studio Code**
2. Go to **Extensions** (Ctrl+Shift+X / Cmd+Shift+X)
3. Search for **"PlatformIO IDE"**
4. Click **Install**
5. Restart VS Code

### Step 2: Create New Project

**Option A: New Project**
```bash
# Using PlatformIO CLI
pio project init --board esp32dev

# Or use PlatformIO GUI:
# Click PlatformIO icon > New Project > Select "Espressif ESP32 Dev Module"
```

**Option B: Clone Repository**
```bash
git clone git@github.com:itbdelaboprogramming/CIREn_IoT.git -b sm_prototype_serial
cd sensor-module
code .  # Opens VS Code in project directory
```

### Step 3: Configure `platformio.ini`

Create or edit `platformio.ini` in project root:

```ini
; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:upesy_wroom]
platform = espressif32
board = rymcu-esp32-devkitc
framework = arduino
lib_deps =
  olikraus/U8g2@^2.36.7
  adafruit/Adafruit Unified Sensor@^1.1.6
  adafruit/DHT sensor library@^1.4.4
  https://github.com/dedalqq/esp32-mcp2515.git
  somhi/ESP8266 SSD1306@^1.0.0
  adafruit/Adafruit GFX Library@^1.12.1
  huynhtancuong/ADXL345@^1.0.2
  dfrobot/DFRobot_BME680@^2.0.0
  dfrobot/DFRobot_BMP280@^1.0.1
  https://github.com/DFRobot/DFRobot_BNO055.git
  adafruit/MAX6675 library@^1.1.2
  martinsos/HCSR04@^2.0.0
  adafruit/Adafruit NeoPixel@^1.15.1


monitor_speed = 115200
; upload_port = /dev/ttyUSB*
; upload_port = COM3
```

### Step 4: Project Structure

```
├── platformio.ini
├── README.md
├── src
│   ├── drivers
│   │   ├── canbus
│   │   │   ├── Canbus.cpp
│   │   │   └── Canbus.h
│   │   └── spi_api
│   │       ├── ESP32_SPI_API.cpp
│   │       └── ESP32_SPI_API.h
│   └── main.cpp
└── test
    └── README
```

### Step 5: Build and Upload

#### **Using PlatformIO GUI:**

1. Open project in VS Code
2. Click **PlatformIO icon** (alien head) in left sidebar
3. Under **PROJECT TASKS** → **esp32dev**:
   - Click **Build** (✓) to compile
   - Click **Upload** (→) to flash ESP32
   - Click **Monitor** to open serial monitor

### Step 6: Monitor Serial Output

```bash
# Open serial monitor
pio device monitor

# Monitor with filters
pio device monitor --filter colorize --filter time

# Monitor with different baud rate
pio device monitor -b 115200
```

**Exit serial monitor:** `Ctrl+C`

---

## Operation Guide

### Power-On Sequence

1. **Intro Screen (5 seconds)**
   - Displays "ITB de Labo" and "Sensor Module"
   - Shows debug mode status
   - Press SELECT to continue

2. **Request ID Screen (5 seconds auto-advance)**
   - Displays device MAC address
   - Shows elapsed time
   - Automatically advances to Configuration

3. **Configuration Menu**
   - Shows 6 menu options
   - Use UP/DOWN to navigate
   - Press SELECT to enter chosen menu

4. **Main Screen**
   - Displays selected sensor data
   - Updates every 200ms (screen) and 1000ms (sensors)
   - Press SELECT to return to Configuration

### Button Controls

| Button | Function | Context |
|--------|----------|---------|
| **SELECT** | Advance state / Confirm selection | All states |
| **UP** | Navigate menu upward | Configuration only |
| **DOWN** | Navigate menu downward | Configuration only |

### Navigation Flow

```
┌─────────────┐
│ Intro Screen│ ◄──────────────────┐
└──────┬──────┘                    │
       │ SELECT                    │
       ▼                           │ SELECT
┌─────────────┐                    │
│ Request ID  │                    │
│ (5 seconds) │                    │
└──────┬──────┘                    │
       │ Auto                      │
       ▼                           │
┌─────────────┐                    │
│Configuration│ ◄─────┐            │
│    Menu     │       │            │
└──────┬──────┘       │ SELECT     │
       │ SELECT       │            │
       ▼              │            │
┌─────────────┐       │            │
│ Main Screen │───────┘            │
│ (Menu Data) │────────────────────┘
└─────────────┘       SELECT
```

---

## Menu System

### Menu 1: Temperature & Humidity

**Displays:**
- DHT22 Temperature (°C)
- DHT22 Humidity (%)
- Device MAC address (last 3 bytes)

**Serial Output (Debug OFF):**
```
Temp: 23.50°C, Humidity: 45.20%
```

**Update Interval:** 1 second

---

### Menu 2: Acceleration

**Displays:**
- X-axis acceleration
- Y-axis acceleration
- Z-axis acceleration

**Serial Output (Debug OFF):**
```
Ax: 0.12, Ay: -0.05, Az: 9.81
```

**Update Interval:** 1 second

**Note:** Values in m/s². Static device should read ~9.81 m/s² on Z-axis (gravity).

---

### Menu 3: Extreme Temperature

**Displays:**
- MAX6675 thermocouple reading (°C)

**Serial Output (Debug OFF):**
```
Extreme Temp: 150.25°C
```

**Update Interval:** 1 second

**Range:** 0-1024°C (depends on K-type thermocouple)

---

### Menu 4: Environment

**Status:** Reserved for future use

**Displays:**
- "No data available"

**Serial Output (Debug OFF):**
```
Environment: No data available
```

---

### Menu 5: Settings

**Status:** Reserved for configuration options

**Displays:**
- "No settings available"

**Serial Output (Debug OFF):**
```
Settings: No data available
```

---

### Menu 6: Reset

**Status:** Reserved for system reset functionality

**Displays:**
- "Press to reset device"

**Serial Output (Debug OFF):**
```
Reset: Standby
```

---


## LED Status Indicators

The NeoPixel LED provides visual feedback on system state:

| Color | State | Meaning |
|-------|-------|---------|
| 🟢 **Green** | Request ID | Entering device identification |
| 🟡 **Yellow** | Main (Configured) | System is configured and running |
| 🟣 **Purple** | Menu Selected | Entering main screen from menu |
| 🔵 **Blue** | Return to Intro | Returning to introduction screen |

**Note:** LED changes occur during state transitions triggered by the SELECT button.

---
