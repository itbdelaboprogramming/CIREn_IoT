# ITB de Labo – ESP-NOW Sensor Module Network

**Version:** 2.0.0
**Platform:** ESP32
**Framework:** Arduino
**Protocol:** ESP-NOW

---

## 1. Overview

The **ITB de Labo Sensor Module Network** is a modular, wireless, multi-sensor data acquisition system utilizing the ESP-NOW protocol for real-time environmental and motion monitoring. The system consists of multiple ESP32-based sensor nodes that transmit data wirelessly to a receiver hub without requiring Wi-Fi connectivity.

Each node functions as an independent sensing unit equipped with specific sensors such as **DHT22** (temperature and humidity) or **LIS331HH** (three-axis accelerometer). The receiver node collects and processes data for storage, monitoring, or integration with a central dashboard such as **CIREn**.

---

## 2. System Architecture

```
[ Sensor Node 1 - DHT22 ]         \
                                    ---> [ ESP32 Receiver / Gateway ]
[ Sensor Node 2 - LIS331HH ]      /
```

Each sensor node periodically acquires data and transmits it to the receiver node via ESP-NOW. The receiver node aggregates and processes the data for further analysis or visualization.

---

## 3. Features

### 3.1 Communication

* Utilizes ESP-NOW peer-to-peer data transfer
* Operates with low latency (typically below 10 ms)
* Does not require Wi-Fi access points or internet connectivity

### 3.2 Supported Sensors

| Sensor   | Type                     | Description                                                      |
| -------- | ------------------------ | ---------------------------------------------------------------- |
| DHT22    | Temperature and Humidity | Measurement range: -40 °C to 80 °C, 0–100% RH                    |
| LIS331HH | 3-Axis Accelerometer     | Measurement range: ±6 g, ±12 g, ±24 g, up to 1 kHz sampling rate |

### 3.3 System Capabilities

* Modular firmware per sensor node
* Configurable sampling and transmission intervals
* Lightweight data format for efficient communication
* Optional serial debug mode

---

## 4. ESP-NOW Data Format

All nodes transmit data using a unified structure to ensure compatibility and ease of integration at the receiver:

```cpp
typedef struct {
  uint8_t sensorType;      // 1 = DHT22, 2 = LIS331HH, 3 = reserved
  float value1;            // Primary measurement (e.g., Temperature or Acc X)
  float value2;            // Secondary measurement (e.g., Humidity or Acc Y)
  float value3;            // Tertiary measurement (e.g., reserved or Acc Z)
  unsigned long timestamp; // System time in milliseconds
} SensorData;
```

---

## 5. Sensor Node Implementations

### 5.1 DHT22 Sensor Node

* Reads temperature and humidity at a fixed sampling rate
* Transmits data to the receiver every two seconds
* Utilizes GPIO 33 for the data line
* Based on the standard DHT22 sensor library

**Configuration:**

```cpp
#define DHT_DATA_PIN GPIO_PIN33
#define DHT_TYPE DHT22
sensorData.sensorType = 1;
```

**Typical Serial Output:**

```
=== DHT Sensor Module ===
MAC Address: 40:22:D8:E8:8B:88
DHT Read - Temp: 27.40°C, Humidity: 62.10%
Data sent successfully
Send Status: Success
```

---

### 5.2 LIS331HH Accelerometer Node

* Measures acceleration across X, Y, and Z axes via I2C communication
* Transmits averaged data every two seconds
* Supports ±6 g, ±12 g, and ±24 g measurement ranges
* Utilizes GPIO 21 (SDA) and GPIO 22 (SCL) for I2C

**Configuration:**

```cpp
DFRobot_LIS331HH_I2C acce(&Wire, 0x18);
acce.setRange(DFRobot_LIS::eLis331hh_6g);
acce.setAcquireRate(DFRobot_LIS::eNormal_1000HZ);
sensorData.sensorType = 2;
```

**Typical Serial Output:**

```
=== LIS331HH Accelerometer Module ===
Chip ID: 0x32
Range set to ±6g
Acquire rate set to 1000Hz
x: -40 mg    y: 10 mg    z: 1024 mg
Data sent successfully
Send Status: Success
```

---

## 6. Hardware Configuration

### 6.1 Common Pin Assignments

| Signal     | ESP32 Pin | Description               |
| ---------- | --------- | ------------------------- |
| I2C SDA    | GPIO 21   | I2C data line             |
| I2C SCL    | GPIO 22   | I2C clock line            |
| DHT22 Data | GPIO 33   | Used by DHT22 module only |
| 3V3 / 5V   | Power     | Sensor power supply       |
| GND        | Ground    | Common reference          |

**Notes:**

* The LIS331HH default I2C address is `0x18`.
* For DHT22, a 10 kΩ pull-up resistor between **DATA** and **VCC** is recommended to ensure stable operation.

---

## 7. Timing Configuration

| Parameter              | Default Value | Description                      |
| ---------------------- | ------------- | -------------------------------- |
| `SENSOR_READ_INTERVAL` | 1000 ms       | Sensor data acquisition interval |
| `SEND_INTERVAL`        | 2000 ms       | Data transmission interval       |

Both parameters can be modified in the source code to balance accuracy and network efficiency.

---

## 8. Debugging

Serial debugging can be enabled or disabled through the following definition:

```cpp
#define DEBUG_MODE true
```

When disabled, only essential operational messages will be displayed.

---

## 9. Deployment Guide

1. **Configure Receiver MAC Address**
   Update the receiver’s MAC address in each sensor node before compilation:

   ```cpp
   uint8_t receiverAddress[] = {0x40, 0x22, 0xD8, 0xE8, 0x8B, 0x88};
   ```

2. **Compilation and Upload**

   * Use PlatformIO or Arduino IDE
   * Select board type: `ESP32 DevKitC`
   * Recommended upload speed: 115200 baud or higher

3. **Testing Procedure**

   * Flash the receiver module first
   * Flash and power up sensor nodes subsequently
   * Monitor both receiver and sensor serial outputs to verify transmission

---

## 10. Future Work

* Development of a receiver module with integrated Wi-Fi and MQTT bridge for CIREn dashboard integration
* Implementation of power-saving features using deep sleep modes
* Addition of data calibration, filtering, and averaging mechanisms