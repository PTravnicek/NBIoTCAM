# ESP32 GPS Photo Uploader

This project implements a GPS-enabled photo capture and upload system using an ESP32 camera module with NB-IoT connectivity.

## Hardware Components

| Component         | Description                        |
|-------------------|------------------------------------|
| ESP32-S3          | Main microcontroller                |
| OV2640 Camera     | Image capture                       |
| DFRobot GNSS      | GPS/GLONASS module                  |
| AXP313A           | Power management IC                 |
| BC95 NB-IoT Modem | Cellular connectivity (UDP/TCP)     |
| LD7032 OLED       | 60x32 mini display                  |
| AHT20 Sensor      | Temperature and humidity            |
| Battery           | Li-ion/LiPo with voltage divider    |

## Software Features

- Captures high-quality photos using ESP32 camera
- Collects GPS data (latitude, longitude, altitude, speed, course)
- Transmits photos over TCP with start/end markers
- Sends sensor and GPS telemetry over UDP in Telegraf line protocol format
- Battery level monitoring and reporting
- Robust network connection handling with auto-retry
- Persistent modem initialization state
- Deep sleep and night mode for power saving
- OLED status display for battery and system state
- Automatic network registration and DNS-based server addressing

## Data Protocols

### TCP Photo Transfer
Photos are sent in chunks with special markers:
- Start marker: `[0x00, 0x00, 0x00, 0x00]`
- End marker: `[0xFF, 0xFF, 0xFF, 0xFF]`
- Maximum chunk size: 1300 bytes
- Automatic retry on failed transmissions
- Timeout handling for incomplete transfers (20 seconds)

### UDP Sensor & GPS Data Format
Data is sent in Telegraf line protocol format:
```
gps_data,device=esp32 gpsFIX=true,temperature=23.1,humidity=45.2,battery=42
```

## Setup Instructions

1. Install PlatformIO
2. Configure your server DNS and ports in `main.cpp`:
   ```cpp
   const String SERVER_IP = "www.iot-magic.com";
   const int TCP_PORT = 8009;
   const int UDP_PORT = 8094;
   ```
3. Upload the code to your ESP32

## Documentation

- **LaTeX Chart**: See `documents/capacity_chart.tex` for a professional, customizable chart visualizing container fill levels over time for different materials (concrete, bricks, AAC, asphalt, free space). The chart is styled for presentations and can be compiled to PDF.
- **Datasheets**: See `datasheets/` for hardware reference manuals.
- **Node-RED Flows**: Node-RED server-side flows for TCP/UDP data processing are available (see comments in `main.cpp`).

## Server-Side Processing

The project includes Node-RED flows for processing incoming data:

### TCP Photo Handling
- Detects start marker and begins accumulation
- Handles timeouts (20 second threshold)
- Removes protocol markers from final image
- Outputs complete JPEG when end marker is detected
- Automatic cleanup of incomplete transfers

### UDP Flow
- Processes sensor and GPS telemetry
- Parses Telegraf line protocol format
- Converts to structured data for storage/visualization
- Handles both hex and direct ASCII formats

## Power & Network Management

- AXP313A manages camera power, battery monitoring, and power optimization
- Automatic network registration and DNS-based server addressing
- Configurable retry attempts for failed transmissions
- Support for both home network and roaming
- IP address monitoring and reporting

## Recent Changes
- Switched to DNS for server address (`www.iot-magic.com`)
- Added LaTeX chart and documentation in `documents/`
- Updated hardware/software documentation and tables
- Improved power management and deep sleep logic

---

For questions or contributions, please open an issue or pull request on GitHub.
