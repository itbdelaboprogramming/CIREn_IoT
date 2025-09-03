# CIREn_IoT
Modular IoT "CIREn" Project by NIW &amp; ITB de Labo

## Getting Started

This project is built using the [PlatformIO](https://platformio.org/) ecosystem. To set up and run the project, follow the steps below:

### Requirements

- [Visual Studio Code](https://code.visualstudio.com/)
- [PlatformIO IDE extension](https://platformio.org/install)

### Steps to Run

1. **Clone the repository:**
   ```bash
   git clone git@github.com:itbdelaboprogramming/CIREn_IoT.git
   cd CIREn_IOT
   ```

2. **Open the project using VS Code with the PlatformIO extension.**

3. **Build and Upload the Firmware:**
   - Use the "PlatformIO: Upload" command from the command palette, or click the upload button in the bottom bar of PlatformIO.
   - Make sure the correct board (e.g., `esp32doit-devkit-v1`) is selected in `platformio.ini`.

---

## Project Structure

The project is organized to support modular sensor integration and maintainability.

```
.
├── platformio.ini         # PlatformIO configuration file
├── src/                   # Main source code
│   ├── main.cpp           # Entry point for firmware
│   └── drivers/           # Modular sensor drivers
│       ├── canbus/       
│       ├── encoder/       
│       ├── proximity/     
│       ├── temperature/   
│       └── ultrasonic/    
├── include/               # Header files
│   └── pinout/            # Board-specific pin configurations
├── lib/                   # Optional local libraries 
├── test/                  # Unit or integration tests 
├── .vscode/               # VS Code configurations for debugging and IntelliSense
└── .pio/ and build/       # PlatformIO build artifacts (auto-generated)
```

### Highlights

- **Modular Drivers**: Each sensor driver is organized in its own subfolder within `src/drivers/`, containing `.cpp` and `.h` files to allow easy reuse and extension.
- **Pinout Management**: The `include/pinout/` folder provides board-specific pin mappings, allowing portability across different development boards.
- **Main Logic**: The `main.cpp` file initializes and reads data from the sensors via the corresponding driver classes.

![alt text](image.png)

# Resume Daily Report: Muhammad Furqan Riansyah Putra					

| Name                          | Role       | Start Time | Time Tracking | Total Duration | Work Progress |
|-------------------------------|------------|------------|---------------|----------------|---------------|
| Muhammad Furqan Riansyah Putra | Part-timer | 2025-08-05 06:10:32 | 02:15:20 | 2.25 | IoT: - Research ESP-NOW packet structure - Test simple data exchange between modules |
| Muhammad Furqan Riansyah Putra | Part-timer | 2025-08-06 20:45:18 | 03:19:48 | 3.33 | IoT: - Research MQTT protocol - Setup local Mosquitto broker - Test publish & subscribe with ESP32 |
| Muhammad Furqan Riansyah Putra | Part-timer | 2025-08-07 06:05:44 | 01:50:12 | 1.83 | IoT: - Research CANBUS communication - Recheck CAN adapter setup - Validate old firmware compatibility |
| Muhammad Furqan Riansyah Putra | Part-timer | 2025-08-08 21:00:25 | 04:00:36 | 4 | IoT: - Research server automation script - Write bash draft for InfluxDB & Grafana installation - Add auto-start services |
| Muhammad Furqan Riansyah Putra | Part-timer | 2025-08-09 14:30:59 | 05:15:18 | 5.25 | IoT: - Research MQTT to InfluxDB integration - Connect MQTT → InfluxDB → Grafana pipeline - Debug missing data issue |
| Muhammad Furqan Riansyah Putra | Part-timer | 2025-08-10 16:20:41 | 06:10:30 | 6.17 | IoT: - Research ESP-NOW reliability - Implement multi-sensor sending - Handle packet loss & retransmit - Verify data on PC |
| Muhammad Furqan Riansyah Putra | Part-timer | 2025-08-11 06:40:11 | 02:45:36 | 2.75 | IoT: - Research documentation best practice - Write README for ESP-NOW module - Update folder structure notes |
| Muhammad Furqan Riansyah Putra | Part-timer | 2025-08-12 20:55:27 | 04:30:14 | 4.5 | IoT: - Research CANBUS performance - Multi-node CAN test - Measure latency & stability under load |
| Muhammad Furqan Riansyah Putra | Part-timer | 2025-08-13 06:15:52 | 01:30:14 | 1.5 | IoT: - Research script automation - Add environment variable setup in bash script |
| Muhammad Furqan Riansyah Putra | Part-timer | 2025-08-14 21:10:09 | 07:00:24 | 7 | IoT: - Research Grafana dashboards - Create dashboard templates - Configure multiple data sources - Test real-time refresh |
| Muhammad Furqan Riansyah Putra | Part-timer | 2025-08-15 22:20:38 | 06:40:12 | 6.67 | IoT: - Research MQTT Sensor programming - Implement WiFi reconnect logic - Add JSON formatting for payload |
| Muhammad Furqan Riansyah Putra | Part-timer | 2025-08-16 13:45:46 | 05:50:14 | 5.83 | IoT: - Research ESP-NOW + MQTT Hybrid - Test fallback (send via ESP-NOW if WiFi down) - Evaluate performance |
| Muhammad Furqan Riansyah Putra | Part-timer | 2025-08-17 15:10:21 | 07:34:48 | 7.58 | IoT: - Research CANBUS stress test - Run continuous data - Validate packet integrity - Weekly progress meeting |
| Muhammad Furqan Riansyah Putra | Part-timer | 2025-08-18 06:25:13 | 02:19:48 | 2.33 | IoT: - Research Tailscale VPN for remote access - Note integration challenges |
| Muhammad Furqan Riansyah Putra | Part-timer | 2025-08-19 20:50:42 | 05:10:12 | 5.17 | IoT: - Research project integration - Merge branches into unified project - Clean unused test files |
| Muhammad Furqan Riansyah Putra | Part-timer | 2025-08-20 21:30:33 | 04:20:12 | 4.34 | IoT: - Research reporting format - Write Monthly Report draft - Summarize ESP-NOW, MQTT, CANBUS progress - Plan next month tasks |
|                               |            |            | **Total**     | **74.52** |               |
