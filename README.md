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
