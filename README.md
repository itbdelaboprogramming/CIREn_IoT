# ITB de Labo - Sensor Module

## Overview

ESP32-based multi-sensor vibration monitoring system with a master-slave wireless architecture. The **slave** device reads data from a CWT vibration sensor (via Modbus RTU) and transmits it wirelessly to the **master** device using ESP-NOW. The master streams the received sensor metrics over serial for logging or further processing.



## Installation (Windows)

### Prerequisites

1. Download and install [VS Code](https://code.visualstudio.com)
2. Open the **Extensions** panel (`Ctrl+Shift+X`) and search for **PlatformIO IDE** — install it
3. Restart VS Code after installation completes
4. Open the project folder in VS Code — PlatformIO auto-detects `platformio.ini`
5. On the first build, all library dependencies are downloaded and installed automatically



## Build Modes

This project has two device roles — **slave** and **master** — each with dedicated build environments.

### Slave Environments

| Environment | Mode | Purpose |
|---|---|---|
| `slave` | Main Program | Upload to the slave device — reads the CWT sensor and transmits data to the master |
| `slave_mac` | MAC Print | Diagnostic — prints the slave's MAC address every 5 seconds |

### Master Environments

| Environment | Mode | Purpose |
|---|---|---|
| `master` | Main Program | Upload to the master device — receives sensor data and streams each metric to serial |
| `master_mac` | MAC Print | Diagnostic — prints the master's MAC address every 5 seconds |
| `master_compact` | Compact Print | Upload to master — outputs a consolidated 1-second summary of all received metrics |

### Upload Command

```bash
pio run -e <environment> -t upload

# Examples
pio run -e slave -t upload
pio run -e master_compact -t upload
```

### Typical Workflow

1. Flash `slave_mac` to the slave and `master_mac` to the master to obtain each device's MAC address
2. Update the peer MAC address in the source code with the values obtained above
3. Flash `slave` to the slave device and `master` (or `master_compact`) to the master device



## Serial Monitor

```bash
pio device monitor
```

Baud rate: **115200**
