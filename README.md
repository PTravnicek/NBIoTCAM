# ESP32 NB-IoT Smart Container Classification System

This project implements an intelligent container fill-level detection system using ESP32 camera module with Edge Impulse machine learning, NB-IoT connectivity, and comprehensive environmental monitoring. The system automatically classifies containers as "filled" or "empty" using on-device AI inference and transmits data over NB-IoT when high-confidence detections are achieved.

## Hardware Components

| Component         | Description                        | Notes                              |
|-------------------|------------------------------------|-----------------------------------|
| ESP32-S3          | Main microcontroller              | FireBeetle ESP32-S3              |
| OV2640 Camera     | Image capture                     | SVGA quality, auto-exposure       |
| DFRobot GNSS      | GPS/GLONASS module                | I2C interface, configurable       |
| AXP313A           | Power management IC               | Camera power control              |
| BC95-GR NB-IoT    | Cellular connectivity (UDP/TCP)  | **Updated for BC95-GR commands**  |
| LD7032 OLED       | 60x32 mini display               | SPI interface, status display     |
| SHT40 Sensor      | Temperature and humidity          | I2C sensor (DFRobot SHT40)        |
| Battery Monitor   | Li-ion/LiPo with voltage divider | Analog reading with calibration   |

## Software Features

### Core Functionality
- **AI-Powered Container Classification**: Edge Impulse machine learning model for real-time container fill-level detection
- **High-Confidence Detection**: Only transmits data when classification confidence ≥ 80% (classification) / ≥ 60% (object detection)
- **Continuous Image Processing**: Real-time camera feed with automatic classification
- **Smart Data Transmission**: Triggers NB-IoT communication only on high-confidence detections
- **GPS/GLONASS Positioning**: Real-time location with satellite count tracking
- **Environmental Monitoring**: Temperature (°C), humidity (%RH), and battery level
- **NB-IoT Connectivity**: Robust TCP/UDP transmission with BC95-GR support
- **Power Management**: Deep sleep modes with configurable wake cycles
- **Status Display**: Real-time OLED feedback during operations

### Machine Learning Integration
- **Edge Impulse Framework**: On-device inference using optimized neural networks
- **Object Detection**: Supports both classification and bounding box detection modes
- **Memory Optimization**: PSRAM utilization for large model inference
- **Classification Labels**: "level_0" (empty), "level_1" (25%), "level_2" (50%), "level_3" (75%), "level_4" (full) container states
- **Confidence Thresholding**: Effective thresholds: 80% (classification) / 60% (object detection)
- **Image Preprocessing**: Configurable top cropping for model compatibility (default: 40%)

### AI Workflow
1. **Continuous Monitoring**: Camera continuously captures and processes frames
2. **Real-Time Classification**: Each frame is analyzed by the Edge Impulse model
3. **Confidence Evaluation**: System waits for detection confidence ≥ 60%
4. **Data Transmission**: High-confidence detections trigger NB-IoT transmission
5. **Sleep Activation**: System enters deep sleep after successful data transmission
6. **Wake Cycle**: Device wakes after ~100 seconds and repeats the process

### Network & Connectivity
- **BC95-GR Compatibility**: Updated for latest Quectel BC95-GR commands
- **Proper PDP Context Setup**: Automatic network attachment with authentication
- **Robust Connection Handling**: Auto-retry with exponential backoff
- **Network Time Synchronization**: Automatic sleep scheduling based on network time
- **Band Configuration**: Automatic band selection (no forced band)

### Power Optimization
- **Event-Driven Operation**: Deep sleep triggered by high-confidence AI detections
- **Night Mode**: Configurable sleep schedule (default: 20:00-06:00)
- **Dynamic Sleep Duration**: Standard 2-minute cycles with extended night sleep
- **Intelligent Wake Logic**: Continuous inference until high-confidence detection
- **Camera Power Control**: Automatic power-down after photo capture and transmission

## BC95-GR Compatibility

This project has been updated for the **Quectel BC95-GR** module, which uses different AT commands compared to the original BC95:

### Updated Commands
| Function | Old BC95 | New BC95-GR | Description |
|----------|----------|-------------|-------------|
| Band Config | `AT+NBAND` | `AT+QBAND` | Frequency band configuration |
| Socket Open | `AT+NSOCR` | `AT+QIOPEN` | Create TCP/UDP connection |
| Send Data | `AT+NSOSD`/`AT+NSOST` | `AT+QISENDEX` | Transmit data |
| Receive Data | `AT+NSORF` | `AT+QIRD` | Read incoming data |
| Close Socket | `AT+NSOCL` | `AT+QICLOSE` | Close connection |

### Network Attachment Sequence
The BC95-GR requires a specific initialization sequence:
1. `AT+CGDCONT=0,"IPV4V6","<your_apn>"` (fallback to context 1 if needed) – Define PDP context/APN
2. `AT+NRB` – Reboot module
3. `AT+CFUN=1` – Set full functionality
4. `AT+CIMI` – Verify SIM/IMSI
5. `AT+QBAND?` – Verify band capabilities (keep auto selection)
6. `AT+CGATT=1` – Trigger network attachment (optionally `AT+COPS=0` for automatic operator selection)

## Data Protocols

### TCP Photo Transfer
Photos are transmitted with protocol markers for server-side processing:
- **Start marker**: `[0x00, 0x00, 0x00, 0x00]`
- **End marker**: `[0xFF, 0xFF, 0xFF, 0xFF]`
- **Chunk size**: 1024 bytes (optimized for BC95-GR)
- **Format**: HEX-encoded binary data
- **Retry logic**: Automatic retransmission on failure
- **Connection**: TCP with context ID 1, connect ID 0

### UDP Sensor & GPS Data Format
Telemetry data uses Telegraf line protocol format with container classification results:
```
gps_data,device=esp32 gpsFIX=true,temperature=23.1,humidity=45.2,battery=42,fillLevel=0.75
```

**Field Descriptions:**
- `gpsFIX`: Boolean indicating GPS fix status
- `temperature`: Temperature in Celsius from SHT40 sensor
- `humidity`: Relative humidity percentage from SHT40 sensor
- `battery`: Battery percentage (0-100%)
- `fillLevel`: Container classification result (0, 0.25, 0.5, 0.75, 100 for full; -1 for unknown)

**Connection Details:**
- Context ID: 1
- Connect ID: 1  
- Local Port: 3014
- Format: HEX-encoded ASCII

## Configuration

### Server Settings
Configure your server endpoints in `main.cpp`:
```cpp
const String SERVER_IP = "34.10.203.180";  // Your server IP
const int TCP_PORT = 8009;                   // Photo upload port
const int UDP_PORT = 8094;                   // Sensor data port
```

### APN Settings
The APN is configured during PDP context setup in `src/main.cpp`. Default is `iot.1nce.net` with fallback between contexts 0 and 1. Change to your carrier APN as needed:
```cpp
// Example in initializeModule():
String apnResponse = sendATCommand("AT+CGDCONT=0,\"IPV4V6\",\"iot.1nce.net\"", 4000);
if (apnResponse.indexOf("OK") == -1) {
  apnResponse = sendATCommand("AT+CGDCONT=1,\"IPV4V6\",\"iot.1nce.net\"", 4000);
}
```

### Sleep Schedule
Customize sleep/wake times and AI inference intervals:
```cpp
int napTimeHour = 20;               // Sleep start (24-hour format)
int wakeUpHour = 6;                 // Wake time (24-hour format)
int TIMEZONE_OFFSET = 0;            // Local timezone offset
#define DEFAULT_SLEEP_TIME 100      // Normal sleep duration (seconds)
```

### AI Classification Settings
Thresholds are defined in `src/main.cpp`:
- Classification mode: 0.8f
- Object detection mode: 0.6f
- Cropping: `topCropPercent` controls top-of-image cropping
```cpp
// In src/main.cpp
static bool debug_nn = false;
int topCropPercent = 40; // Crop top 40% of the image before classification
```

### Hardware Pins
Key pin definitions (see `main.cpp` for complete list):
```cpp
#define VDD_POWER 13        // Power control pin
#define ANALOG_PIN A5       // Battery monitoring
#define BC95_RX_PIN 44      // NB-IoT module RX
#define BC95_TX_PIN 43      // NB-IoT module TX
```

## Setup Instructions

### Prerequisites
- PlatformIO IDE
- BC95-GR NB-IoT module with active SIM card
- ESP32-S3 development board (FireBeetle recommended)
- Server infrastructure for receiving TCP/UDP data

### Installation Steps
1. **Clone Repository**
   ```bash
   git clone <repository-url>
   cd DFR0975_CAM_NBIOT_CONTAINER_CLASSIFICATION_V0
   ```

2. **Configure PlatformIO**
   - Open project in PlatformIO
   - Install required libraries (auto-detected from `platformio.ini`)
   - **Note**: Edge Impulse library is included in `lib/` directory

3. **Update Configuration**
   - Set server IP addresses and ports
   - Adjust sleep schedule if needed
   - Configure timezone offset
   - Set AI confidence threshold (default: 60%)

4. **Hardware Setup**
   - Connect all components according to pin definitions
   - Insert activated NB-IoT SIM card
   - Connect battery with proper voltage divider
   - Ensure proper lighting for camera classification

5. **Upload and Monitor**
   ```bash
   pio run --target upload
   pio device monitor
   ```

## Documentation

### Project Files
- **📊 Power Analysis**: `POWER_CONSUMPTION/` - Power consumption measurements
- **📋 Datasheets**: `datasheets/` - Hardware reference manuals including BC95-GR AT commands
- **📄 LaTeX Charts**: `documents/` - Professional charts for capacity monitoring
- **🔧 Node-RED Flows**: `src/nodeRed/` - Server-side data processing flows
- **📸 Test Photos**: `photos/` - Sample captured images

### Server-Side Processing

#### TCP Photo Handler (Node-RED)
```javascript
// Accumulates chunks between start/end markers
// Handles 20-second timeout for incomplete transfers
// Removes protocol markers from final JPEG
// See: src/nodeRed/accumulateAndCheckTCPPhoto.js
```

#### UDP Telemetry Processor
```javascript
// Parses Telegraf line protocol
// Converts HEX to ASCII
// Structures data for database storage
// See: src/nodeRed/ReturnUDPToRM.js
```

## Power Management

### Battery Monitoring
- **Voltage Divider**: 3.3V → 4.2V range mapping
- **Calibration Formula**: `battery_voltage = 1.3975 * divider_voltage - 0.4168`
- **Percentage Calculation**: Linear mapping from 3.0V (0%) to 4.2V (100%)

### Sleep Modes
- **Normal Operation**: ~100-second wake cycles for continuous monitoring
- **AI-Triggered Sleep**: Deep sleep activated after high-confidence detection and data transmission
- **Night Mode**: Extended sleep until wake time (20:00-06:00 default)
- **Deep Sleep Current**: ~865µA (optimized)
- **Camera Power-Down**: Automatic after photo capture and classification

### Network Optimization
- **Connection Cleanup**: All sockets closed before sleep
- **Module Reset**: `AT+CFUN=0` for minimal power
- **Network Detach**: `AT+CGATT=0` before sleep

## Troubleshooting

### Common Issues

**Network Registration Fails**
- Check SIM card activation and data plan
- Verify NB-IoT coverage in your area
- Check antenna connection
- Monitor signal quality with `AT+CSQ`

**Photo Upload Errors**
- Verify server is listening on TCP port
- Check network connectivity with UDP test
- Monitor memory usage during large photo captures
- Verify server can handle chunked data protocol

**AI Classification Issues**
- Check Edge Impulse model is properly loaded
- Monitor PSRAM usage during inference
- Verify camera image quality and lighting conditions
- Check confidence threshold settings (default: 60%)
- Enable debug mode: `debug_nn = true` for detailed inference logs

**High Power Consumption**
- Ensure camera power is disabled after capture
- Check all connections are closed before sleep
- Verify `VDD_POWER` pin control is working
- Monitor `AT+CFUN=0` execution

**GPS/GNSS Issues**
- Allow 2+ minutes for initial fix outdoors
- Check GNSS module power and connections
- Verify satellite visibility (clear sky view)
- Enable both GPS and GLONASS: `gnss.setGnss(eGPS_GLONASS)`

## Recent Changes (v3.0)

### Major Updates
- ✅ **AI Integration**: Edge Impulse machine learning for container classification
- ✅ **Event-Driven Architecture**: Transmission triggered by high-confidence detections
- ✅ **String Comparison Fix**: Resolved fillLevel classification bug
- ✅ **Sleep Optimization**: Default cycle ~100 seconds for better monitoring
- ✅ **Memory Management**: PSRAM utilization for AI inference
- ✅ **Continuous Monitoring**: Real-time camera feed with classification

### Previous Updates (v2.0)
- ✅ **BC95-GR Compatibility**: Complete migration to new AT command set
- ✅ **Network Attachment**: Proper PDP context and authentication setup
- ✅ **Socket API Overhaul**: Migration from AT+NSO* to AT+QI* commands
- ✅ **Registration Parsing**: Fixed `+CEREG: 0,5` detection (roaming support)
- ✅ **Power Optimization**: Improved sleep sequence with proper cleanup

### Breaking Changes
- **AI Model**: Requires Edge Impulse container classification model
- **Hardware**: Requires BC95-GR module (not compatible with original BC95)
- **Commands**: All socket operations use new AT+QI* command set
- **Network**: New PDP context setup required for attachment
- **Sleep Timing**: Reduced from 30-minute to 2-minute cycles

### Performance Improvements
- **AI Inference**: Real-time on-device classification with 80% confidence threshold (classification) / 60% (object detection)
- **Smart Transmission**: Only sends data when meaningful detections occur
- **Memory Optimization**: PSRAM allocation for large AI models
- **Power Efficiency**: Event-driven sleep reduces unnecessary wake cycles
- **Classification Accuracy**: Improved string comparison for reliable state detection

---

**📞 Support**: For issues or contributions, please open a GitHub issue with detailed logs and hardware configuration.

**🔗 Related Projects**: Check `src/nodeRed/` for complete server-side implementation examples.
