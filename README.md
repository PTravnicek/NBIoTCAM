# ESP32 GPS Photo Uploader

This project implements a GPS-enabled photo capture and upload system using an ESP32 camera module with NB-IoT connectivity.

## Hardware Components

- ESP32-S3 with Camera
- DFRobot GNSS Module
- AXP313A Power Management
- BC95 NB-IoT Modem

## Features

- Captures high-quality photos using ESP32 camera
- Collects GPS data (latitude, longitude, altitude, speed, course)
- Transmits photos over TCP with start/end markers
- Sends GPS telemetry over UDP in Telegraf line protocol format
- Supports both outdoor (real GPS) and indoor (mock location) modes
- Battery level monitoring
- Robust network connection handling with auto-retry
- Persistent modem initialization state

## Data Protocols

### TCP Photo Transfer
Photos are sent in chunks with special markers:
- Start marker: `[0x00, 0x00, 0x00, 0x00]`
- End marker: `[0xFF, 0xFF, 0xFF, 0xFF]`
- Maximum chunk size: 1300 bytes
- Automatic retry on failed transmissions
- Timeout handling for incomplete transfers (20 seconds)

### UDP GPS Data Format
GPS data is sent in Telegraf line protocol format:
```
gps_data,device=esp32 latitude=50.123400,longitude=14.123400,altitude=123.4,speed=0.00,course=0.00,satellites=0,battery=42
```

## Setup Instructions

1. Install PlatformIO
2. Configure your server IP and ports in `main.cpp`:
   ```cpp
   const String SERVER_IP = "your.server.ip";
   const int TCP_PORT = 8009;
   const int UDP_PORT = 8094;
   ```
3. Set the operating mode in `main.cpp`:
   ```cpp
   Mode currentMode = OUTDOOR;  // or INDOOR for testing
   ```
4. Upload the code to your ESP32

## Server-Side Processing

The project includes Node-RED flows for processing incoming data:

### TCP Photo Handling
- Detects start marker and begins accumulation
- Handles timeouts (20 second threshold)
- Removes protocol markers from final image
- Outputs complete JPEG when end marker is detected
- Automatic cleanup of incomplete transfers

### UDP Flow
- Processes GPS telemetry
- Parses Telegraf line protocol format
- Converts to structured data for storage/visualization
- Handles both hex and direct ASCII formats

## Power Management

The AXP313A power management IC handles:
- Camera power control
- Battery monitoring
- Power optimization

## Network Management

- Automatic network registration handling
- Connection status monitoring
- Configurable retry attempts for failed transmissions
- Support for both home network and roaming
- IP address monitoring and reporting
