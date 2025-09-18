#include <Arduino.h>

// Tell ESP camera header to use a different name for sensor_t
#define sensor_t camera_sensor_t
#include "esp_camera.h"
#undef sensor_t

#include "DFRobot_AXP313A.h"
#include "DFRobot_GNSS.h"
#include "driver/rtc_io.h"
#include "driver/i2c.h"
#include "driver/periph_ctrl.h"
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <WiFi.h>

#include <DFRobot_SHT40.h>

// ---------------------------------------------------------------------------
// I2C bus configuration (explicit to avoid pin conflicts with camera)
// Override via build flags if your wiring uses different pins, e.g.:
//   -DI2C_SDA_PIN=xx -DI2C_SCL_PIN=yy -DI2C_CLOCK_HZ=100000
// ---------------------------------------------------------------------------
#ifndef I2C_CLOCK_HZ
#define I2C_CLOCK_HZ 100000UL
#endif

#ifndef PIN_WIRE_SDA
// Default to IO1/IO2 to match current hardware where I2C shares with camera SCCB
#define PIN_WIRE_SDA 1
#endif
#ifndef PIN_WIRE_SCL
#define PIN_WIRE_SCL 2
#endif

#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN PIN_WIRE_SDA
#endif
#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN PIN_WIRE_SCL
#endif

// ---------
#include <tflite_vasek_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
// ---------

// Override Edge Impulse memory allocation to force PSRAM usage for large allocations
__attribute__((weak)) void *ei_malloc(size_t size) {
    // For large allocations (>1KB), prefer PSRAM
    if (size > 1024) {
        void* ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
        if (ptr) return ptr;
    }
    // Fallback to regular heap
    return heap_caps_malloc(size, MALLOC_CAP_8BIT);
}

__attribute__((weak)) void ei_free(void* ptr) {
    heap_caps_free(ptr);
}


// ---------------------------------------------------------------------------
// Camera Pins
// ---------------------------------------------------------------------------
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    45
#define SIOD_GPIO_NUM    1
#define SIOC_GPIO_NUM    2
#define Y9_GPIO_NUM      48
#define Y8_GPIO_NUM      46
#define Y7_GPIO_NUM      8
#define Y6_GPIO_NUM      7
#define Y5_GPIO_NUM      4
#define Y4_GPIO_NUM      41
#define Y3_GPIO_NUM      40
#define Y2_GPIO_NUM      39
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    42
#define PCLK_GPIO_NUM    5

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3
/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; //points to the output of the capture

// ---------------------------------------------------------------------------
// SHT40 Sensor
// ---------------------------------------------------------------------------  
DFRobot_SHT40 SHT40(SHT40_AD1B_IIC_ADDR);
bool useSHT40 = true;

// ---------------------------------------------------------------------------
// GNSS I2C
// ---------------------------------------------------------------------------
DFRobot_GNSS_I2C gnss(&Wire ,GNSS_DEVICE_ADDR);
bool useGNSS = true;


// ---------------------------------------------------------------------------
// AXP Power Management
// ---------------------------------------------------------------------------
DFRobot_AXP313A axp;
#define VDD_POWER 13  // 

// ---------------------------------------------------------------------------
// Fill Level
// ---------------------------------------------------------------------------
float fillLevel = 0;

// ---------------------------------------------------------------------------
// Image Cropping Configuration
// ---------------------------------------------------------------------------
int topCropPercent = 40;  // Crop top 30% of the image before classification

// ---------------------------------------------------------------------------
// Mini OLED display
// ---------------------------------------------------------------------------
#define OLED_SCK    SCK
#define OLED_MOSI   MOSI
#define OLED_CS     D6
#define OLED_A0     D2
#define OLED_RST    D3

// Using the correct constructor for LD7032 display
U8G2_LD7032_60X32_F_4W_SW_SPI u8g2(U8G2_R0, OLED_SCK, OLED_MOSI, OLED_CS, OLED_A0, OLED_RST);

bool debugShowATcommands = false; // showing Quectel configuration details
bool showOLED = true;            // Show OLED only after reset, not after sleep wakeup 

void oled_display(const char* line1, const char* line2, const char* line3) {
  if (!showOLED) return;  // Skip OLED display if not needed
  
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  // Calculate x to center
  int screenWidth = 60;
  int y1 = 10, y2 = 20, y3 = 30; // 9 pixels apart for 8-pixel font height + 1 pixel spacing

  int x1 = (screenWidth - u8g2.getStrWidth(line1)) / 2;
  int x2 = (screenWidth - u8g2.getStrWidth(line2)) / 2;
  int x3 = (screenWidth - u8g2.getStrWidth(line3)) / 2;

  u8g2.drawStr(x1, y1, line1);
  u8g2.drawStr(x2, y2, line2);
  u8g2.drawStr(x3, y3, line3);

  u8g2.sendBuffer();
  // delay(1000);                    // Wait to show message
  // u8g2.clearBuffer();
  // u8g2.sendBuffer();
}


// ---------------------------------------------------------------------------
// AHT Sensor
// ---------------------------------------------------------------------------


// Simple I2C scanner for diagnostics
void scanI2CBus() {
  Serial.println("Scanning I2C bus...");
  uint8_t count = 0;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at 0x");
      Serial.println(String(address, HEX));
      count++;
    }
  }
  if (count == 0) Serial.println("No I2C devices found");
}

// ---------------------------------------------------------------------------
// BC95 Modem Setup
// (Adapt pins, debug macros, etc. as needed for your environment)
// ---------------------------------------------------------------------------

// Debugging option
#define DEBUG 1

#if DEBUG == 1
  #define debug(x)   Serial.print(x)
  #define debugln(x) Serial.println(x)
#else
  #define debug(x)
  #define debugln(x)
#endif

// RX/TX pins for the BC95
#define BC95_RX_PIN 44  // ESP32 RX pin connected to BC95 TX
#define BC95_TX_PIN 43  // ESP32 TX pin connected to BC95 RX


// Initialize HardwareSerial on UART2 (for BC95)
HardwareSerial bc95_modem(2);

// Server details
const String SERVER_IP = "34.10.203.180";  // Address of system sending the message.
// IP addresses can be specified in decimal, octal or hexadecimal notation.
const int    TCP_PORT  = 8009; 
const int    UDP_PORT  = 8094;
bool moduleInitialized = false;

// // ---------------------------------------------------------------------------
// // Battery Percentage
// // ---------------------------------------------------------------------------  
// const int analogPin = A0;
int batteryPercentage = 42;            // Example battery status
// float batteryAnalogValue;
// float batteryInputVoltage;


// ---------------------------------------------------------------------------
// Camera Frame Buffer
// We'll store the last captured frame here.
// ---------------------------------------------------------------------------
bool doCameraTCPJob = true;           // Takes a photo and sends it to server
camera_fb_t *capturedFrame = nullptr;
uint8_t *classificationPhoto = nullptr;  // Store the photo used for classification
size_t classificationPhotoSize = 0;       // Size of the classification photo


// Add global variables to store temp and humidity
struct SensorData {
    float temperature = 0.0;
    float humidity = 0.0;
} sensorData;
bool sensorDataValid = false;

// GNSS coordinates cache for UDP payload
double gnssLatitude = 0.0;
double gnssLongitude = 0.0;
bool gnssCoordinatesValid = false;

// Sleep configuration
#define uS_TO_S_FACTOR 1000000ULL // Conversion factor for micro seconds to seconds
#define DEFAULT_SLEEP_TIME 600  // seconds

// GNSS configuration
#define TIMEOUT_DURATION 120000   // Timeout duration in milliseconds (2 min) for GPS 
#define TIMEZONE_OFFSET 0         // Local timezone offset from network time (+2 hours)
bool gpsFixObtained = false;      // Track if GPS fix was successful


// RTC memory variables to store
RTC_DATA_ATTR int customSleepDuration = DEFAULT_SLEEP_TIME; // seconds
RTC_DATA_ATTR int  normalSleepDuration = DEFAULT_SLEEP_TIME;
RTC_DATA_ATTR bool wasNightSleep = false;

// ---------------------------------------------------------------------------
// Sleep configuration
// ---------------------------------------------------------------------------
int napTimeHour = 21;                  // Time when the unit goes to sleep for the whole night
int wakeUpHour = 6;                    // Time when the unit will wake up and follow normal work/sleep cycles

// Add near the top with other defines
#define ANALOG_PIN A5  // ADC2_0

// Add this function to read and display analog value
void readAndDisplayAnalog() {
  int analogValue = analogRead(ANALOG_PIN);
  float dividerVoltage = analogValue * (3.3 / 4095.0);  // Convert to voltage (ESP32 has 12-bit ADC)
  
  // Calculate actual battery voltage using linear regression from measurements:
  // Point 1: 3.089V divider → 3.9V battery
  // Point 2: 3.250V divider → 4.125V battery
  // Formula: battery_voltage = 1.3975 * divider_voltage - 0.4168
  float batteryVoltage = 1.3975 * dividerVoltage - 0.4168;
  
  // Calculate battery percentage (4.2V = 100%, 3.0V = 0%) and update global variable
  batteryPercentage = map(batteryVoltage * 100, 300, 420, 0, 100);
  batteryPercentage = constrain(batteryPercentage, 0, 100);
  
  // Debug: confirm global variable is updated
  debugln("Global batteryPercentage updated to: " + String(batteryPercentage));
  
  // Display on Serial
  debug("Analog Value: ");
  debug(analogValue);
  debug(" (");
  debug(dividerVoltage);
  debug("V) Battery: ");
  debug(batteryVoltage);
  debug("V (");
  debug(batteryPercentage);
  debugln("%)");
  
  // Display on OLED
  char voltStr[10];
  char percStr[10];
  snprintf(voltStr, sizeof(voltStr), "%.2fV", batteryVoltage);
  snprintf(percStr, sizeof(percStr), "%d%%", batteryPercentage);
  oled_display("Battery", voltStr, percStr);
}

// ---------------------------------------------------------------------------
// Function Prototypes
// ---------------------------------------------------------------------------

// bool initCamera();
void capturePhoto();
String sendATCommand(const String &cmd, unsigned long waitMs);
void initializeModule();
String bufferToHex(const uint8_t *data, size_t length);
bool sendPhotoOverTCP(const uint8_t *buffer, size_t length);
void sendUDPMessage();
void receiveUDPMessage(int connectID);
void readSHT40Sensor();
void enterSleepMode();
void getGNSS();
bool parseNetworkTime(String timeResponse, int &hour, int &minute);
bool checkNapTime(int hour, int minute);

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;
// High-confidence result function
void checkHighConfidenceAndSleep(ei_impulse_result_t* result);

void getGNSS(){
  unsigned long startTime = millis();
  while (millis() - startTime < TIMEOUT_DURATION){
    uint8_t numSatellites = gnss.getNumSatUsed();
    if (numSatellites > 0) {
      sTim_t utc = gnss.getUTC();
      sTim_t date = gnss.getDate();
      sLonLat_t lat = gnss.getLat();
      sLonLat_t lon = gnss.getLon();
      double high = gnss.getAlt();
      uint8_t starUserd = gnss.getNumSatUsed();
      double sog = gnss.getSog();
      double cog = gnss.getCog();

      debugln("");
      debug(date.year);
      debug("/");
      debug(date.month);
      debug("/");
      debug(date.date);
      debug("/");
      debug(utc.hour);
      debug(":");
      debug(":");
      debug(utc.minute);
      debug(utc.second);
      debugln();
      debugln((char)lat.latDirection);
      debugln((char)lon.lonDirection);
      
      // Serial.print("lat DDMM.MMMMM = ");
      // Serial.println(lat.latitude, 5);
      // Serial.print(" lon DDDMM.MMMMM = ");
      // Serial.println(lon.lonitude, 5);
      Serial.print("lat degree = ");
      Serial.println(lat.latitudeDegree,6);
      Serial.print("lon degree = ");
      Serial.println(lon.lonitudeDegree,6);

      Serial.print("star userd = ");
      Serial.println(starUserd);
      Serial.print("alt high = ");
      Serial.println(high);
      Serial.print("sog =  ");
      Serial.println(sog);
      Serial.print("cog = ");
      Serial.println(cog);
      Serial.print("gnss mode =  ");
      Serial.println(gnss.getGnssMode());
      oled_display("Satelites", "used", String(starUserd).c_str());
      delay(1000);

      oled_display("GNSS", "fix", "OK");
      // Store coordinates for later UDP payload (apply sign based on N/S/E/W)
      gnssLatitude = lat.latitudeDegree;
      if ((char)lat.latDirection == 'S') {
        gnssLatitude = -gnssLatitude;
      }
      gnssLongitude = lon.lonitudeDegree;
      if ((char)lon.lonDirection == 'W') {
        gnssLongitude = -gnssLongitude;
      }
      gnssCoordinatesValid = true;
      gpsFixObtained = true;  // Set flag when GPS fix is successful
      return;
    }
    delay(5000); // Wait 1 second before next check
    oled_display("GNSS", "time", String(millis() - startTime).c_str());
  }
  // If we reach here, timeout occurred without GPS fix
  oled_display("GNSS", "timeout", "no fix");
  gpsFixObtained = false;  // Ensure flag is set to false on timeout
  gnssCoordinatesValid = false;
}


// Camera config
static camera_config_t config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 10, //0-63 lower number means higher quality (slightly higher number for smaller size)
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

    if (is_initialised) return true;

    //initialize the camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x\n", err);
      return false;
    }

    is_initialised = true;
    return true;
}

// ---------------------------------------------------------------------------
// capturePhoto()
//    Grabs a single frame and stores in global 'capturedFrame'.
// ---------------------------------------------------------------------------
void capturePhoto() {
  // Capture and discard initial frames
  for (int i = 0; i < 2; i++) {
      camera_fb_t * temp_fb = esp_camera_fb_get();
      if (temp_fb) {
          esp_camera_fb_return(temp_fb);
          debugln("Camera frame discard...");
      }
      delay(150); // Wait between frames
  }
  // Now capture the actual frame you want to use
  capturedFrame = esp_camera_fb_get();
  debugln("Camera frame stored...");
  delay(200); // Short settle

  if (!capturedFrame) {
    debugln("Camera capture failed!");
    oled_display("camera", "capture", "ERROR");
    return;
  }
  oled_display("camera", "capture", "OK");
  debug("Captured image size: ");
  debugln(capturedFrame->len);
}

// ---------------------------------------------------------------------------
// BC95 HELPER FUNCTIONS
// ---------------------------------------------------------------------------

/**
 * Drain any pending input from the modem for up to drainMs milliseconds.
 */
void modemFlushInput(unsigned long drainMs = 0) {
  unsigned long start = millis();
  while (bc95_modem.available() || (drainMs > 0 && (millis() - start) < drainMs)) {
    while (bc95_modem.available()) {
      (void)bc95_modem.read();
    }
    if (drainMs == 0) break;
  }
}

/**
 * Wait for a URC containing any of the expected tokens. Returns index (1-based) of match, or 0 on timeout.
 * Captured response is returned via out.
 */
int waitForAnyOf(const char* token1, const char* token2, unsigned long timeoutMs, String &out) {
  out = "";
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (bc95_modem.available()) {
      char c = (char)bc95_modem.read();
      out += c;
      if (token1 && strstr(out.c_str(), token1)) return 1;
      if (token2 && strstr(out.c_str(), token2)) return 2;
    }
  }
  return 0;
}

/**
 * Wait specifically for +QIOPEN: <connectID>,0 indicating socket open success.
 */
bool waitForQIOPEN(int connectID, unsigned long timeoutMs) {
  String needle = String("+QIOPEN: ") + String(connectID) + ",0";
  String resp;
  int which = waitForAnyOf(needle.c_str(), nullptr, timeoutMs, resp);
  if (which == 1) return true;
  debugln(String("Timeout or error while waiting for QIOPEN URC. Last resp: ") + resp);
  return false;
}

/**
 * Wait for SEND OK / SEND FAIL after QISENDEX.
 */
bool waitForSendResult(unsigned long timeoutMs) {
  String resp;
  int which = waitForAnyOf("SEND OK", "SEND FAIL", timeoutMs, resp);
  if (which == 1) return true;
  if (which == 2) {
    debugln(String("Send reported FAIL. Resp: ") + resp);
    return false;
  }
  debugln(String("Timeout waiting SEND OK/FAIL. Resp: ") + resp);
  return false;
}

/**
 * Send an AT command and capture its response.
 *
 * @param cmd The AT command to send.
 * @param waitMs How long to wait for a response (ms).
 * @return The response from the modem (trimmed).
 */
String sendATCommand(const String &cmd, unsigned long waitMs = 1000) {
  // debugln("[BC95 SEND] " + cmd);
  // Drain any stale URCs before issuing a new command to avoid mixing responses
  modemFlushInput(5);
  bc95_modem.print(cmd + "\r\n");

  String response;
  bool seenTerminator = false;
  unsigned long start = millis();
  while (millis() - start < waitMs) {
    while (bc95_modem.available()) {
      char c = (char)bc95_modem.read();
      response += c;
      // Early exit when 'OK' or 'ERROR' appears and a CRLF has been seen
      if (response.indexOf("\r\nOK") != -1 || response.indexOf("\r\nERROR") != -1) {
        seenTerminator = true;
      }
    }
    if (seenTerminator) break;
  }
  response.trim();

  if (response.length() > 0) {
    debugln("[BC95 RSP] " + response) + '\n';
  }

  return response;
}

// parseSocketID function removed - no longer needed for BC95-GR AT+QI* commands


/**
 * Convert an ASCII string to a hex string.
 */
String asciiToHex(const String &ascii) {
  String hex;
  hex.reserve(ascii.length() * 2);
  for (int i = 0; i < ascii.length(); i++) {
    char buffer[3];
    sprintf(buffer, "%02X", (uint8_t)ascii[i]);
    hex += buffer;
  }
  return hex;
}

/**
 * Query network time using AT+CCLK? and parse hour/minute safely via URC wait.
 */
bool queryNetworkTime(int &hour, int &minute, String &rawOut) {
  rawOut = "";
  modemFlushInput(10);
  // First try to get full response via the standard command path
  String resp = sendATCommand("AT+CCLK?", 1500);
  // If +CCLK: not present in the immediate response, wait for URC async
  if (resp.indexOf("+CCLK:") == -1) {
    String extra;
    int which = waitForAnyOf("+CCLK:", nullptr, 4000, extra);
    if (which == 0) {
      debugln("AT+CCLK? timed out waiting for +CCLK:");
      return false;
    }
    resp += extra;
  }
  // Extract the line containing +CCLK:
  int idx = resp.lastIndexOf("+CCLK:");
  if (idx < 0) idx = resp.indexOf("+CCLK:");
  if (idx < 0) {
    debugln("Could not locate +CCLK: in buffer");
    return false;
  }
  int lineEnd = resp.indexOf('\n', idx);
  String line = (lineEnd > idx) ? resp.substring(idx, lineEnd) : resp.substring(idx);
  rawOut = line;
  // Find quoted time
  int q1 = line.indexOf('"');
  int q2 = (q1 >= 0) ? line.indexOf('"', q1 + 1) : -1;
  String quoted = (q1 >= 0 && q2 > q1) ? line.substring(q1, q2 + 1) : line; // fallback
  // Reuse existing parser; it expects a comma between date and time
  String toParse = quoted;
  if (!parseNetworkTime(toParse, hour, minute)) {
    debugln("Failed to parse +CCLK: line: " + line);
    return false;
  }
  return true;
}

/**
 * Prepare a HEX-encoded message in Telegraf line protocol format.
 *
 * Builds a line protocol string from mock data, then converts it to HEX.
 */
String prepareHexMessage() {
  // Use cached sensor values captured earlier to avoid SCCB/I2C contention
  readAndDisplayAnalog();
  String lineProtocol;
  lineProtocol.reserve(160);
  lineProtocol = "gps_data,device=esp32 ";
  lineProtocol += "gpsFIX=" + String(gpsFixObtained ? 1 : 0) + ",";
  // Use cached values if valid; otherwise emit NaN-safe defaults
  lineProtocol += "temperature=" + String(sensorDataValid ? sensorData.temperature : NAN, 1) + ",";
  lineProtocol += "humidity=" + String(sensorDataValid ? sensorData.humidity : NAN, 1) + ",";
  lineProtocol += "latitude=" + String(gnssCoordinatesValid ? gnssLatitude : NAN, 6) + ",";
  lineProtocol += "longitude=" + String(gnssCoordinatesValid ? gnssLongitude : NAN, 6) + ",";
  lineProtocol += "battery=" + String(batteryPercentage) + ",";
  lineProtocol += "fillLevel=" + String(fillLevel);

  debugln("Line Protocol as ASCII:");
  debugln(lineProtocol);

  // Convert the ASCII line protocol to HEX
  return asciiToHex(lineProtocol);
}


/**
 * Initialize the BC95 module (reset, set to full functionality, attach).
 */
void initializeModule() {
  oled_display("init", "module", "start");
  sendATCommand("AT"); // Check basic communication
  delay(100);
  sendATCommand("ATE0"); // Disable command echo for cleaner response parsing
  delay(100);
  // Verbose error codes
  sendATCommand("AT+CMEE=2", 1000);
  delay(50);

  if (debugShowATcommands){
    // Check firmware version
    debugln("Checking firmware version...");
    oled_display("check", "firmware", "ver");
    String fwVersion = sendATCommand("AT+CGMR", 1000);
    debugln("Firmware version: " + fwVersion);
    
    // Check module configuration
    debugln("Checking module configuration...");
    oled_display("check", "module", "config");
    String moduleConfig = sendATCommand("AT+NCONFIG?", 1000);
    debugln("Module configuration: " + moduleConfig);

    // Check SIM Card Status
    oled_display("check", "SIM", "card");
    String cimiResponse = sendATCommand("AT+CIMI", 500);
    if (cimiResponse.indexOf("ERROR") != -1 || cimiResponse.length() < 5) {
      debugln("ERROR: Failed to read SIM card (AT+CIMI). Check SIM insertion/activation.");
      oled_display("SIM", "card", "error");
      // Optional: halt or implement retry logic
      // return; // Consider stopping if SIM is essential
    } else {
      debugln("SIM IMSI: " + cimiResponse);
      oled_display("SIM", "card", "OK");
    }
    delay(100);
  }

  // BC95-GR Network Attachment Sequence (from manual)

  debugln("Rebooting module...");
  oled_display("module", "reboot", "wait");
  sendATCommand("AT+NRB", 1000);
  
  // Wait for reboot to complete (module should respond with REBOOTING and then Neul)
  debugln("Waiting for module reboot to complete...");
  delay(5000); // Wait 5 seconds for reboot
  
  // Re-establish communication after reboot
  sendATCommand("AT", 1000); // Check basic communication
  // Disable sleep/PSM to avoid registration stalls
  oled_display("sleep", "QSCLK", "0");
  sendATCommand("AT+QSCLK=0", 2000);
  delay(100);
  oled_display("PSM", "disable", "...");
  sendATCommand("AT+CPSMS=0", 2000);
  delay(100);

  // Ensure full functionality so SIM interface is powered
  debugln("Setting full functionality (CFUN=1)...");
  oled_display("set", "CFUN", "1");
  sendATCommand("AT+CFUN=1", 5000);
  delay(300);
  
  // Wait for SIM to be ready
  debugln("Waiting for SIM ready (CPIN)...");
  oled_display("SIM", "wait", "CPIN");
  bool simReady = false;
  for (int attempt = 0; attempt < 60 && !simReady; attempt++) {
    String cpin = sendATCommand("AT+CPIN?", 1000);
    if (cpin.indexOf("READY") != -1) {
      simReady = true;
      break;
    }
    delay(1000);
  }
  if (!simReady) {
    debugln("ERROR: SIM not ready");
    oled_display("SIM", "not", "ready");
  }

  // Configure PDP context with correct context and PDP type
  // debugln("Setting PDP context (APN)...");
  // oled_display("set", "APN", "ctx0");
  // String apnResponse = sendATCommand("AT+CGDCONT=0,\"IPV4V6\",\"iot.1nce.net\"", 4000);
  // if (apnResponse.indexOf("OK") == -1) {
  //   debugln("APN ctx0 failed, trying ctx1...");
  //   oled_display("set", "APN", "ctx1");
  //   apnResponse = sendATCommand("AT+CGDCONT=1,\"IPV4V6\",\"iot.1nce.net\"", 4000);
  // }
  // if (apnResponse.indexOf("OK") == -1) {
  //   debugln("Failed to set APN");
  //   oled_display("set", "APN", "error");
  // } else {
  //   debugln("APN set successfully");
  //   oled_display("set", "APN", "OK");
  // }

  // // Step 6: Query IMSI to verify SIM is working
  // debugln("Querying IMSI...");
  // oled_display("query", "IMSI", "check");
  // String imiResponse = sendATCommand("AT+CIMI", 2000); // Increased timeout for SIM
  // debugln("IMSI response: " + imiResponse);
  
  // // Step 7: Query PDP context to verify it's set correctly
  // debugln("Verifying PDP context...");
  // oled_display("verify", "PDP", "context");
  // sendATCommand("AT+CGDCONT?", 2000);

  // // Avoid forcing band on BC95-GR; keep auto for better registration
  // delay(300);
  // String verifyBand = sendATCommand("AT+QBAND?", 1000);
  // debugln("Band capabilities: " + verifyBand);

  // Step 8: Trigger network attachment (final step in BC95-GR sequence)
  // Start attach and searching
  debugln("Attaching to network...");
  oled_display("attach", "CGATT", "1");
  sendATCommand("AT+CGATT=1", 1000);
  // Optional automatic operator selection
  sendATCommand("AT+COPS=0", 630);

  // Keep checking registration status
  String CEREGresponse;
  int ceregChecks = 0;
  const int MAX_CEREG_CHECKS = 60; // Increased checks for a total timeout of 10 minutes for initial registration
  bool registered = false;

  do {
    ceregChecks++;
    CEREGresponse = sendATCommand("AT+CEREG?", 5000); // Increased wait time for command response
    debugln("AT+CEREG? response (" + String(ceregChecks) + "/" + String(MAX_CEREG_CHECKS) + "): " + CEREGresponse);

      if (CEREGresponse.indexOf("+CEREG: 0,1") != -1 || CEREGresponse.indexOf("+CEREG: 0,5") != -1) {
     registered = true;
     oled_display("network", "registered", "OK");
     break; // Exit loop if registered (home or roaming)
    } else if (CEREGresponse.indexOf("+CEREG: 0,2") != -1) {
      debugln("Network searching...");
      oled_display("searching", "network", String(ceregChecks).c_str());
    } else if (CEREGresponse.indexOf("+CEREG: 0,0") != -1) {
      debugln("Not registered, not searching.");
      oled_display("not", "registered", "error");
    } else if (CEREGresponse.indexOf("+CEREG: 0,3") != -1) {
      debugln("Registration denied!");
      oled_display("registration", "denied", "error");
      // Consider stopping or specific error handling
    } else {
       debugln("Waiting for registration...");
       oled_display("waiting", "for", "network");
    }
    delay(5000); // Increased delay between checks to be less aggressive

  } while (!registered && ceregChecks < MAX_CEREG_CHECKS);

  if (registered) {
      if (CEREGresponse.indexOf("+CEREG: 0,1") != -1) {
        debugln("Network registered successfully (home network)");
        oled_display("home", "network", "OK");
      } else {
        debugln("Network registered successfully (roaming)");
        oled_display("roaming", "network", "OK");
      }
      // sendATCommand("AT+CGPADDR", 1000);    // Query IP address

      // debugln("Signal statistics (AT+NUESTATS)...");
      // sendATCommand("AT+NUESTATS", 4000);
      // oled_display("signal", "stats", "check");

      // Band query already printed above

  } else {
      debugln("ERROR: Failed to register on network after " + String(MAX_CEREG_CHECKS * 2) + " seconds.");
      oled_display("network", "error", "timeout");
      
      // --- Check Signal Quality HERE, even if not registered ---
      debugln("Running comprehensive diagnostics...");
      oled_display("running", "network", "diag");
      
      debugln("1. Signal statistics (AT+NUESTATS)...");
      sendATCommand("AT+NUESTATS", 2000);
      oled_display("signal", "stats", "check");
      
      debugln("2. Extended signal quality (AT+CSQ)...");
      sendATCommand("AT+CSQ", 1000);
      oled_display("signal", "quality", "check");
      
      debugln("3. Network operator settings (AT+COPS?)...");
      sendATCommand("AT+COPS?", 1000);
      oled_display("operator", "settings", "check");
      
      debugln("4. Available frequency bands (AT+QBAND?)...");
      sendATCommand("AT+QBAND?", 1000);
      oled_display("frequency", "bands", "check");
      
      debugln("5. Preferred selection mode (AT+NPTWEDRXS?)...");
      sendATCommand("AT+NPTWEDRXS?", 1000);
      oled_display("selection", "mode", "check");
      
      debugln("6. Module functionality level (AT+CFUN?)...");
      sendATCommand("AT+CFUN?", 1000);
      oled_display("module", "function", "check");
      
      debugln("7. Radio access technology (AT+NCONFIG?AUTOCONNECT)...");
      sendATCommand("AT+NCONFIG?AUTOCONNECT", 1000);
      oled_display("radio", "access", "check");
      
      debugln("8. IMEI number (AT+CGSN=1)...");
      sendATCommand("AT+CGSN=1", 1000);
      oled_display("IMEI", "check", "done");
      // return; // Consider stopping
  }

  // Check attach status again
  String cgattResponse = sendATCommand("AT+CGATT?", 1000);
  debugln("AT+CGATT? response: " + cgattResponse);

  // sendATCommand("AT+QREGSWT=2");  // Disable Huawei platform registration (Optional)

  moduleInitialized = registered; // Only set true if registered
  debugln("Module initialization complete. Status: " + String(moduleInitialized ? "Initialized" : "Failed"));
}

/**
 * Convert a binary buffer to HEX string for BC95's NSOSD command.
 */
String bufferToHex(const uint8_t *data, size_t length) {
  String hex;
  hex.reserve(length * 2);
  
  for (size_t i = 0; i < length; i++) {
    char tmp[3];
    sprintf(tmp, "%02X", data[i]);
    hex += tmp;
  }
  return hex;
}

/**
 * Send the photo buffer via TCP (chunked).
 *
 * 1) Initialize the BC95 module.
 * 2) Create a STREAM socket.
 * 3) Connect to the server.
 * 4) Send data in manageable chunks (e.g., 200 bytes).
 * 5) Send termination sequence (4 bytes: 0xFF 0xFF 0xFF 0xFF).
 * 6) Close the socket.
 *
 * @param buffer Pointer to the image data.
 * @param length Size of the image data in bytes.
 */
bool sendPhotoOverTCP(const uint8_t *buffer, size_t length) {
  debugln("sendPhotoOverTCP() starting...");
  oled_display("TCP", "send", "start");

  // 1) Initialize the module
  if (!moduleInitialized) {
    oled_display("init", "module", "TCP");
    initializeModule();
  }

  // 2) Open TCP connection using BC95-GR command: AT+QIOPEN=<contextID>,<connectID>,"TCP",<IP>,<port>
  oled_display("open", "TCP", "connection");
  int contextID = 1;  // Context ID (1-3)
  int connectID = 0;  // Connect ID (0-5)
  
  // Make sure input buffer is clean before opening
  modemFlushInput(20);

  String openCmd = "AT+QIOPEN=" + String(contextID) + "," + String(connectID) + 
                   ",\"TCP\",\"" + SERVER_IP + "\"," + String(TCP_PORT);
  
  String response = sendATCommand(openCmd, 1000);
  if (response.indexOf("OK") == -1) {
    debugln("Failed to issue QIOPEN!");
    oled_display("TCP", "open", "error");
    return false;
  }
  // Wait explicitly for URC confirming the socket is open
  if (!waitForQIOPEN(connectID, 20000)) {
    oled_display("TCP", "open", "timeout");
    return false;
  }
  debugln("TCP connection opened with connect ID: " + String(connectID));

  // 3) Send data in chunks using AT+QISEND
  const size_t CHUNK_SIZE = 1024;  // Smaller chunks for BC95-GR
  size_t offset = 0;
  int totalChunks = (length + CHUNK_SIZE - 1) / CHUNK_SIZE;

  // Send initiation marker (4 bytes: 0x00 0x00 0x00 0x00)
  oled_display("send", "start", "marker");
  const uint8_t BGN_MARKER[] = {0x00, 0x00, 0x00, 0x00};
  String bgnMarkerHex = bufferToHex(BGN_MARKER, 4);
  String bgnCmd = "AT+QISENDEX=" + String(connectID) + ",4,\"" + bgnMarkerHex + "\"";
  
  // Send, then wait for SEND OK URC
  (void)sendATCommand(bgnCmd, 200);
  if (!waitForSendResult(10000)) {
    debugln("Start marker SEND FAIL");
    oled_display("marker", "send", "error");
    return false;
  }

  // Send data chunks
  oled_display("send", "data", "chunks");
  while (offset < length) {
    size_t thisChunkSize = min(CHUNK_SIZE, length - offset);
    int currentChunk = (offset / CHUNK_SIZE) + 1;
    String hexChunk = bufferToHex(buffer + offset, thisChunkSize);
    String sendCmd = "AT+QISENDEX=" + String(connectID) + "," + 
                     String(thisChunkSize) + ",\"" + hexChunk + "\"";
    
    bool sent = false;
    for (int attempt = 0; attempt < 3 && !sent; attempt++) {
      modemFlushInput(5);
      (void)sendATCommand(sendCmd, 200);
      sent = waitForSendResult(15000);
      if (!sent) {
        debugln("QISENDEX SEND FAIL, retrying chunk " + String(currentChunk));
        delay(150);
      }
    }
    if (!sent) {
      debugln("Failed to send chunk " + String(currentChunk));
      oled_display("chunk", "error", String(currentChunk).c_str());
      return false;
    }
    
    // Show progress
    char progress[10];
    snprintf(progress, sizeof(progress), "%d/%d", currentChunk, totalChunks);
    oled_display("chunk", progress, "sent");
    
    offset += thisChunkSize;
    delay(120); // Small pacing between chunks
  }

  // Send termination marker (4 bytes: 0xFF 0xFF 0xFF 0xFF)
  oled_display("send", "end", "marker");
  const uint8_t END_MARKER[] = {0xFF, 0xFF, 0xFF, 0xFF};
  String endMarkerHex = bufferToHex(END_MARKER, 4);
  String endCmd = "AT+QISENDEX=" + String(connectID) + ",4,\"" + endMarkerHex + "\"";
  (void)sendATCommand(endCmd, 200);
  (void)waitForSendResult(10000);

  // 4) Close the connection
  oled_display("close", "TCP", "socket");
  sendATCommand("AT+QICLOSE=" + String(connectID), 3000);
  oled_display("TCP", "send", "done");
  return true;
}

// Calculate sleep duration until wake-up time
int calculateSleepDuration(int currentHour, int currentMinute) {
    int currentTotalMinutes = currentHour * 60 + currentMinute;
    int targetTotalMinutes = wakeUpHour * 60;
    return ((targetTotalMinutes - currentTotalMinutes) + (24 * 60)) % (24 * 60);
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  // Release the hold on VDD_POWER pin
  rtc_gpio_hold_dis((gpio_num_t)VDD_POWER);
  
  // Set VDD_POWER LOW (turn ON) for normal operation
  digitalWrite(VDD_POWER, LOW);

  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : debugln("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : debugln("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : debugln("Wakeup caused by timer"); break;
    default : debugln("Wakeup not caused by deep sleep"); break;
  }
}

void i2cBusRecover(int sdaPin, int sclPin) {
  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, INPUT_PULLUP);
  delay(2);
  // Generate up to 9 clock pulses on SCL to release a stuck slave holding SDA low
  pinMode(sclPin, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(sclPin, LOW);
    delayMicroseconds(5);
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(5);
    if (digitalRead(sdaPin) == HIGH) break;
  }
  // Create a STOP condition
  pinMode(sdaPin, OUTPUT);
  digitalWrite(sdaPin, LOW);
  delayMicroseconds(5);
  digitalWrite(sclPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(sdaPin, HIGH);
  delay(2);
}

uint8_t axpReadReg(uint8_t reg) {
  Wire.beginTransmission(0x36);
  Wire.write(reg);
  if (Wire.endTransmission(true) != 0) return 0xFF;  // send STOP
  delay(2);  // small settle time
  if (Wire.requestFrom((uint8_t)0x36, (uint8_t)1, (uint8_t)true) != 1) return 0xFF;
  return Wire.read();
}

void enterSleepMode() {
  debugln("Preparing for sleep...");
  
  // 1) Stop radios/services first
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  // 2) Deinit camera and stop XCLK so no GPIO drives the sensor
  Serial.println("Deinitializing camera & stopping XCLK...");
  esp_camera_deinit();
  ledcDetachPin(XCLK_GPIO_NUM);
  pinMode(XCLK_GPIO_NUM, INPUT);

  // 3) Tri-state all camera related GPIOs to avoid back-powering
  if (PWDN_GPIO_NUM >= 0) pinMode(PWDN_GPIO_NUM, INPUT);
  if (RESET_GPIO_NUM >= 0) pinMode(RESET_GPIO_NUM, INPUT);
  pinMode(PCLK_GPIO_NUM, INPUT);
  pinMode(VSYNC_GPIO_NUM, INPUT);
  pinMode(HREF_GPIO_NUM, INPUT);
  pinMode(Y2_GPIO_NUM, INPUT);
  pinMode(Y3_GPIO_NUM, INPUT);
  pinMode(Y4_GPIO_NUM, INPUT);
  pinMode(Y5_GPIO_NUM, INPUT);
  pinMode(Y6_GPIO_NUM, INPUT);
  pinMode(Y7_GPIO_NUM, INPUT);
  pinMode(Y8_GPIO_NUM, INPUT);
  pinMode(Y9_GPIO_NUM, INPUT);

  // --- Close All Connections ---
  debugln("Attempting to close potential connections (0, 1, 2)...");
  sendATCommand("AT+QICLOSE=0", 300); // Try closing connect ID 0
  sendATCommand("AT+QICLOSE=1", 300); // Try closing connect ID 1
  sendATCommand("AT+QICLOSE=2", 300); // Try closing connect ID 2
  delay(500); // Give time for connections to close

  // --- Detach from Network ---
  debugln("Detaching from network (AT+CGATT=0)...");
  sendATCommand("AT+CGATT=0", 5000); // Detach from GPRS service
  delay(1000); // Wait after detaching

  // --- Set Minimum Functionality ---
  debugln("Setting minimum functionality (AT+CFUN=0)...");
  sendATCommand("AT+CFUN=0", 90000); // Set minimum functionality to power down RF
  delay(1000); // Wait after CFUN=0

  // --- Configure Sleep Timer ---
  esp_sleep_enable_timer_wakeup(customSleepDuration * uS_TO_S_FACTOR);

  debugln("Setup ESP32 to sleep for " + String(customSleepDuration) + " Seconds");
  debugln("Going to sleep now");

  oled_display("going", "to", "sleep");
  
  // Set VDD_POWER HIGH and hold it
  digitalWrite(VDD_POWER, HIGH);   // Turn VDD line OFF
  rtc_gpio_hold_en((gpio_num_t)VDD_POWER);    // Hold the pin state during sleep
  
  Serial.flush(); // Ensure all debug messages are sent before sleeping

  // Clean up classification photo if it exists
  if (classificationPhoto != nullptr) {
    free(classificationPhoto);
    classificationPhoto = nullptr;
    classificationPhotoSize = 0;
    debugln("Cleaned up classification photo");
  }

  // 4) Re-initialize I2C for AXP on the shared bus to avoid SCCB side effects
  Wire.end();
  i2cBusRecover(SIOD_GPIO_NUM, SIOC_GPIO_NUM);
  Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM, 100000);
  Wire.setClock(100000);

  // 5) Disable camera power rails on AXP313A by clearing DLDO1 (b4) and ALDO1 (b3)
  Serial.println("Disabling AXP camera rails (clear DLDO1/ALDO1 bits in 0x10)...");
  uint8_t reg10 = axpReadReg(0x10);
  // Only keep DCDC1 (bit 0), clear all other bits (DLDO1, ALDO1, DLDO2, ALDO2)
  uint8_t new10 = reg10 & (1 << 0);
  if (new10 != reg10) {
    axp.writeReg(0x10, &new10, 1);
  }
  delay(20);
  uint8_t confirm10 = axpReadReg(0x10);
  Serial.printf("AXP 0x10 before=0x%02X after=0x%02X\n", reg10, confirm10);

  // 6) Allow rails to discharge to avoid residual current/back-power
  delay(150);
  // --- Enter Deep Sleep ---
  esp_deep_sleep_start();
}

void receiveUDPMessage(int connectID) { // Pass the correct connect ID
    // Wait and check for incoming messages using BC95-GR commands
    const int MAX_ATTEMPTS = 5;
    const int WAIT_TIME = 2000; // 2 seconds between checks

    debugln("Attempting to receive UDP data on connect ID " + String(connectID) + "...");

    for (int i = 0; i < MAX_ATTEMPTS; i++) {
        // Read from the connection using AT+QIRD=<connectID>,<req_length>
        String readCmd = "AT+QIRD=" + String(connectID) + ",512";
        String response = sendATCommand(readCmd, 2000);
        debugln("UDP Read Response (attempt " + String(i + 1) + "): " + response);

        if (response.indexOf("ERROR") == -1) {
            // Check if we received actual data
            // BC95-GR response format: +QIRD: <length><CR><LF><data>
            if (response.indexOf("+QIRD:") != -1) {
                // Parse the length and data
                int colonPos = response.indexOf(':');
                if (colonPos != -1) {
                    String lengthStr = response.substring(colonPos + 1);
                    lengthStr.trim();
                    int dataLength = lengthStr.toInt();
                    
                    if (dataLength > 0) {
                        debugln("Received UDP data length: " + String(dataLength));
                        debugln("Received UDP data: " + response);
                        // TODO: Add parsing logic here if you need to process the received data
                        // customSleepDuration = ... // parse and set based on data
                        break; // Exit loop after receiving data
                    } else {
                        debugln("No data available yet on connect ID " + String(connectID));
                    }
                }
            } else if (response.indexOf("OK") != -1 && response.length() <= 4) {
                debugln("No data available yet on connect ID " + String(connectID));
            } else {
                debugln("Unexpected UDP read response: " + response);
            }
        } else {
            debugln("Error reading UDP connect ID " + String(connectID));
        }
        debugln("Waiting for UDP response...");
        delay(WAIT_TIME);
    }

    // Close the connection *after* attempting to receive
    debugln("Closing UDP connect ID " + String(connectID) + " after receive attempts.");
    sendATCommand("AT+QICLOSE=" + String(connectID), 3000);
}

/**
 * Send a UDP message.
 *
 * Initializes the module, creates a UDP socket, sends a HEX-encoded
 * Telegraf line protocol message, and closes the socket. If the UDP
 * send command returns an error, it retries up to 5 times.
 */
void sendUDPMessage() {
  if (!moduleInitialized) {
    // Attempt to initialize if not already done
    initializeModule();
    if (!moduleInitialized) {
        debugln("Failed to initialize module for UDP. Aborting send.");
        return; // Can't send if module isn't ready
    }
  }

  // Prepare message in HEX
  String hexMessage = prepareHexMessage();
  int hexMessageBytes = hexMessage.length() / 2;

  debugln("\nLine Protocol as HEX:");
  debugln(hexMessage);

  // Open UDP connection using BC95-GR command: AT+QIOPEN=<contextID>,<connectID>,"UDP",<IP>,<port>,<local_port>
  int contextID = 1;  // Context ID (1-3)
  int connectID = 1;  // Connect ID (0-5), use different ID from TCP
  int localPort = 3014; // Local port for UDP
  
  String openCmd = "AT+QIOPEN=" + String(contextID) + "," + String(connectID) + 
                   ",\"UDP\",\"" + SERVER_IP + "\"," + String(UDP_PORT) + "," + String(localPort);
  
  String response = sendATCommand(openCmd, 10000);
  if (response.indexOf("OK") == -1) {
    debugln("Failed to open UDP connection!");
    return;
  }
  
  // Wait for connection to establish
  delay(3000);
  debugln("UDP connection opened with connect ID: " + String(connectID));

  // Send the UDP message using AT+QISENDEX
  String sendCmd = "AT+QISENDEX=" + String(connectID) + "," + 
                   String(hexMessageBytes) + ",\"" + hexMessage + "\"";

  // Attempt sending the message up to 5 times if an error is received
  const int MAX_ATTEMPTS = 5;
  int attempt = 0;
  bool sentSuccessfully = false;
  
  while (attempt < MAX_ATTEMPTS) {
    String sendResponse = sendATCommand(sendCmd, 5000);
    if (sendResponse.indexOf("SEND OK") != -1 || sendResponse.indexOf("OK") != -1) {
      debugln("UDP message sent successfully on attempt " + String(attempt + 1));
      sentSuccessfully = true;
      break;
    } else {
      attempt++;
      debugln("UDP message send error. Attempt " + String(attempt) + " failed. Retrying...");
      delay(500);
    }
  }
  
  if (!sentSuccessfully) {
    debugln("UDP message failed to send after " + String(MAX_ATTEMPTS) + " attempts.");
  }

  // Close the UDP connection
  debugln("Closing UDP connection " + String(connectID));
  sendATCommand("AT+QICLOSE=" + String(connectID), 3000);
}

/**
 * Parse network time from AT+CCLK response
 * Expected format: +CCLK: "25/06/08,07:29:51+08"
 * Returns hour and minute by reference
 */
bool parseNetworkTime(String timeResponse, int &hour, int &minute) {
  // Find the start of the time part (after the comma)
  int commaIndex = timeResponse.indexOf(',');
  if (commaIndex == -1) {
    debugln("Error: No comma found in time response");
    return false;
  }
  
  // Extract time part: "07:29:51+08"
  String timePart = timeResponse.substring(commaIndex + 1);
  
  // Find the hour and minute
  int colonIndex = timePart.indexOf(':');
  if (colonIndex == -1) {
    debugln("Error: No colon found in time part");
    return false;
  }
  
  // Extract hour and minute
  hour = timePart.substring(0, colonIndex).toInt();
  minute = timePart.substring(colonIndex + 1, colonIndex + 3).toInt();
  
  debugln("Parsed time - Hour: " + String(hour) + ", Minute: " + String(minute));
  return true;
}

/**
 * Check if it's night time and handle sleep if needed
 */
bool checkNapTime(int hour, int minute) {
  int localHour = (hour + TIMEZONE_OFFSET) % 24;  // Adjust for local timezone
  
  debugln("Network time: " + String(hour) + ":" + String(minute));
  debugln("Local time: " + String(localHour) + ":" + String(minute));
  
  bool isNapTime = false;
  if (napTimeHour < wakeUpHour) {
    // Nap time does NOT cross midnight
    isNapTime = (localHour >= napTimeHour && localHour < wakeUpHour);
  } else {
    // Nap time crosses midnight (e.g., 22 to 6)
    isNapTime = (localHour >= napTimeHour || localHour < wakeUpHour);
  }
  if (isNapTime) {
    debugln("Nap time detected! Going to extended sleep...");
    oled_display("Nap", "time", "sleep");
    
    int minutesUntilWake = calculateSleepDuration(localHour, minute);
    customSleepDuration = minutesUntilWake * 60;  // Convert to seconds
    wasNightSleep = true;
    
    debugln("Sleeping for " + String(minutesUntilWake) + " minutes");
    enterSleepMode();
    return true;  // This won't be reached due to deep sleep
  }
  
  debugln("Awake time - continuing normal operation");
  return false;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        // Swap BGR to RGB here
        // due to https://github.com/espressif/esp32-camera/issues/379
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}

void runMainJob(){
    // Read and display initial analog value
  // readAndDisplayAnalog();
  // delay(1000);  // Show the value for a second
  // Initialize BC95 modem regardless of camera job
  bc95_modem.begin(9600, SERIAL_8N1, BC95_RX_PIN, BC95_TX_PIN);
  debugln("BC95 modem RX TX pins set.");

  // Initialize module first to ensure network connection
  initializeModule();
  
  if (moduleInitialized) {
    // Get current time from network after successful connection
    debugln("Checking network time...");
    int networkHour, networkMinute;
    String rawClock;
    if (queryNetworkTime(networkHour, networkMinute, rawClock)) {
      debugln("Network time: " + rawClock);
      if (checkNapTime(networkHour, networkMinute)) {
        // Device will go to sleep in checkNapTime() if it's night
      }
    } else {
      debugln("Failed to parse network time, continuing with normal operation");
    }
  } else {
    debugln("Module not initialized, skipping time check");
  }

  if (doCameraTCPJob) {
    // Use the classification photo instead of taking a new one
    if (classificationPhoto != nullptr && classificationPhotoSize > 0) {
      debugln("Using classification photo for transmission");
      oled_display("send", "photo", "TCP");
      sendPhotoOverTCP(classificationPhoto, classificationPhotoSize);
      
      // Free the classification photo after transmission
      free(classificationPhoto);
      classificationPhoto = nullptr;
      classificationPhotoSize = 0;
      debugln("Classification photo transmitted and freed");
    } else {
      debugln("No classification photo available, taking new photo");
      // Fallback: Initialize camera and take new photo
      if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
        oled_display("camera", "init", "ERROR");
        return;
      }
      
      // Capture a single photo in 'capturedFrame'
      capturePhoto();
      
      // Send the captured photo over TCP (if available)
      if (capturedFrame) {
        sendPhotoOverTCP(capturedFrame->buf, capturedFrame->len);
        
        // IMPORTANT: Release the frame buffer when you're done
        esp_camera_fb_return(capturedFrame);
        capturedFrame = nullptr;
        axp.disablePower();
        debugln("camera power down");
      }
    }
  }
  // Send UDP message regardless of camera job
  sendUDPMessage();
}


void readSHT40Sensor() {
  // Read temperature and humidity using high precision
  float temperatureC = SHT40.getTemperature(/*mode = */PRECISION_MID);
  float humidityRH   = SHT40.getHumidity(/*mode = */PRECISION_MID);

  // Validate readings
  if (isnan(temperatureC) || isnan(humidityRH)) {
    debugln("SHT40 read invalid (NaN)");
    sensorDataValid = false;
    return;
  }

  // Clamp humidity to physical range
  if (humidityRH < 0.0f) humidityRH = 0.0f;
  if (humidityRH > 100.0f) humidityRH = 100.0f;

  sensorData.temperature = temperatureC;
  sensorData.humidity = humidityRH;
  sensorDataValid = true;

  debug("SHT40 T/H: ");
  debug(sensorData.temperature);
  debug(" C, ");
  debug(sensorData.humidity);
  debugln(" %");
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  // Start Serial for debug
  Serial.begin(115200);
  delay(2000);


  if (useGNSS){
    // Initialize GNSS module
    while(!gnss.begin()){
      debugln("NO Devices !");
      oled_display("GNSS", "not", "found");
      delay(1000);
    }
    gnss.enablePower();      
    gnss.setGnss(eGPS_BeiDou_GLONASS); // USE gps + glonass
    gnss.setRgbOn();
    getGNSS(); // Simple function to show gps fix if obtained
  }

  // // Initialize I2C explicitly on chosen pins and clock
  // Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  // Wire.setClock(I2C_CLOCK_HZ);
  // Wire.setTimeOut(50);

  if (useSHT40){
    SHT40.begin();
    readSHT40Sensor();
  }
  // // Optional: scan bus once at boot for diagnostics
  // scanI2CBus();

  // Get power to the camera before initialization, set shutdown key
  while(axp.begin() != 0){
    Serial.println("init error");
    delay(1000);
  }
  axp.enableCameraPower(axp.eOV2640);

  // Initialize VDD_POWER pin and set it LOW (turn ON)
  rtc_gpio_hold_dis((gpio_num_t)VDD_POWER); // Release any previous hold
  pinMode(VDD_POWER, OUTPUT);
  digitalWrite(VDD_POWER, LOW);  // Turn VDD line ON

  // Initialize analog pin
  pinMode(ANALOG_PIN, INPUT);
  
  // Check wake-up reason and determine if we should show OLED
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) {
    // This is a reset/power-on, show OLED messages
    showOLED = true;
    debugln("Reset/Power-on detected - OLED enabled");
  } else {
    // This is a wake-up from deep sleep, skip OLED messages
    showOLED = false;
    debugln("Wake-up from deep sleep detected - OLED disabled");
  }

  u8g2.begin();
  oled_display("main", "setup", "begin");
  delay(1000);

  // After waking up, reset customSleepDuration to default unless night sleep is triggered again
  customSleepDuration = DEFAULT_SLEEP_TIME;
  wasNightSleep = false;

  ////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////
  // Debug memory information
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());
  
  // Force PSRAM to be used for all allocations and defragment memory
  heap_caps_malloc_extmem_enable(100000); // Enable external memory for allocations > 20KB

  if (ei_camera_init() == false) {
      ei_printf("Failed to initialize Camera!\r\n");
  }
  else {
      ei_printf("Camera initialized\r\n");
  }

  ei_printf("\nStarting continious inference in 2 seconds...\n");
  ei_sleep(2000);
  ////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  // enterSleepMode();
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop(){
    // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
    if (ei_sleep(50) != EI_IMPULSE_OK) {
        return;
    }

    // Try to allocate snapshot buffer in PSRAM first, then fallback to heap
    snapshot_buf = (uint8_t*)heap_caps_malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE, MALLOC_CAP_SPIRAM);
    
    // If PSRAM allocation fails, try heap
    if(snapshot_buf == nullptr) {
        snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
    }

    // check if allocation was successful
    if(snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        ei_printf("Free heap: %d, Free PSRAM: %d\n", ESP.getFreeHeap(), ESP.getFreePsram());
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    // Debug memory before classification
    ei_printf("Memory before classification - Free heap: %d, Free PSRAM: %d\n", 
              ESP.getFreeHeap(), ESP.getFreePsram());

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        ei_printf("Memory after failed classification - Free heap: %d, Free PSRAM: %d\n", 
                  ESP.getFreeHeap(), ESP.getFreePsram());
        return;
    }

    // print the predictions
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }

    // Print the prediction results (classification)
#else
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }
#endif

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
    ei_printf("Visual anomalies:\r\n");
    for (uint32_t i = 0; i < result.visual_ad_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }
#endif

    // Check for high-confidence results and sleep if found
    checkHighConfidenceAndSleep(&result);

    // Free the snapshot buffer using the appropriate function
    if (snapshot_buf) {
        heap_caps_free(snapshot_buf);
        snapshot_buf = nullptr;
    }

}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {

    //deinitialize the camera
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK)
    {
        ei_printf("Camera deinit failed\n");
        return;
    }

    is_initialised = false;
    return;
}


/**
 * @brief      Crop the top portion of an RGB888 image
 *
 * @param[in]  input_buf     input image buffer
 * @param[in]  input_width   width of input image
 * @param[in]  input_height  height of input image
 * @param[out] output_buf    output image buffer (can be same as input)
 * @param[in]  crop_percent  percentage of top to crop (0-100)
 *
 * @retval     true if cropping successful
 *
 */
bool crop_top_portion_rgb888(uint8_t *input_buf, uint32_t input_width, uint32_t input_height, 
                            uint8_t *output_buf, int crop_percent) {
    if (input_buf == nullptr || output_buf == nullptr) {
        ei_printf("ERR: Null buffer in crop_top_portion_rgb888\n");
        return false;
    }
    if (crop_percent <= 0 || crop_percent >= 100) {
        // No cropping needed, just copy the image
        memcpy(output_buf, input_buf, input_width * input_height * 3);
        return true;
    }
    
    // Calculate crop parameters
    uint32_t crop_pixels = (input_height * crop_percent) / 100;
    uint32_t new_height = input_height - crop_pixels;
    
    if (new_height <= 0) {
        ei_printf("ERR: Crop too large, no image left\n");
        return false;
    }
    
    ei_printf("Cropping top %d%% (%d pixels), new height: %d\n", crop_percent, crop_pixels, new_height);
    
    // Copy the contiguous bottom portion (skip top crop_pixels rows)
    // Use memmove to safely handle overlapping input/output buffers
    size_t row_bytes = (size_t)input_width * 3;
    size_t bytes_to_copy = (size_t)new_height * row_bytes;
    uint8_t *src_ptr = input_buf + ((size_t)crop_pixels * row_bytes);
    memmove(output_buf, src_ptr, bytes_to_copy);
    
    return true;
}

/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

   // Copy original JPEG for transmission BEFORE releasing the frame buffer
   if (classificationPhoto != nullptr) {
       heap_caps_free(classificationPhoto);
       classificationPhoto = nullptr;
   }
   classificationPhotoSize = fb->len;
   classificationPhoto = (uint8_t*)heap_caps_malloc(classificationPhotoSize, MALLOC_CAP_SPIRAM);
   if (classificationPhoto != nullptr) {
       memcpy(classificationPhoto, fb->buf, classificationPhotoSize);
       ei_printf("Stored classification photo: %d bytes\n", classificationPhotoSize);
   } else {
       ei_printf("ERR: Failed to allocate memory for classification photo (%d bytes)\n", (int)classificationPhotoSize);
   }

   // Convert JPEG to RGB888 while we still own the frame buffer
   bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

   // Now it's safe to return the frame buffer
   esp_camera_fb_return(fb);

   if(!converted){
       ei_printf("Conversion failed\n");
       return false;
   }

    // Apply top cropping before any other processing
    if (topCropPercent > 0) {
        ei_printf("Applying top crop: %d%%\n", topCropPercent);
        if (!crop_top_portion_rgb888(snapshot_buf, EI_CAMERA_RAW_FRAME_BUFFER_COLS, 
                                    EI_CAMERA_RAW_FRAME_BUFFER_ROWS, snapshot_buf, topCropPercent)) {
            ei_printf("ERR: Top cropping failed\n");
            return false;
        }
        
        // Update the effective height for subsequent processing
        uint32_t crop_pixels = (EI_CAMERA_RAW_FRAME_BUFFER_ROWS * topCropPercent) / 100;
        uint32_t new_height = EI_CAMERA_RAW_FRAME_BUFFER_ROWS - crop_pixels;
        
        // Adjust the image processing parameters for the cropped height
        if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (img_height != new_height)) {
            do_resize = true;
        }
    } else {
        if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
            || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
            do_resize = true;
        }
    }

    if (do_resize) {
        // Calculate the actual height after cropping
        uint32_t actual_height = EI_CAMERA_RAW_FRAME_BUFFER_ROWS;
        if (topCropPercent > 0) {
            uint32_t crop_pixels = (EI_CAMERA_RAW_FRAME_BUFFER_ROWS * topCropPercent) / 100;
            actual_height = EI_CAMERA_RAW_FRAME_BUFFER_ROWS - crop_pixels;
        }
        
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        actual_height,
        out_buf,
        img_width,
        img_height);
    }

    return true;
}

/**
 * @brief Check for high-confidence results and enter deep sleep if found
 */
void checkHighConfidenceAndSleep(ei_impulse_result_t* result) {
    float maxConfidence = 0.0f;
    std::string bestLabel = "";
    bool foundHighConfidence = false;

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    // Object detection mode
    for (uint32_t i = 0; i < result->bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result->bounding_boxes[i];
        if (bb.value >= 0.6f && bb.value > maxConfidence) {
            maxConfidence = bb.value;
            bestLabel = std::string(bb.label);
            foundHighConfidence = true;
        }
    }
#else
    // Classification mode
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result->classification[i].value >= 0.8f && result->classification[i].value > maxConfidence) {
            maxConfidence = result->classification[i].value;
            bestLabel = std::string(ei_classifier_inferencing_categories[i]);
            foundHighConfidence = true;
        }
    }
#endif

    if (foundHighConfidence) {
        ei_printf("*** HIGH CONFIDENCE DETECTION: %s (%.3f) - Going to send data over NbIoT ***\r\n", 
                  bestLabel.c_str(), maxConfidence);
        
        // Map level-based classification to fill levels
        if (bestLabel == "level_0"){
            fillLevel = 0;    // Empty (0%)
        }
        else if (bestLabel == "level_1"){
            fillLevel = 0.25;   // Quarter full (25%)
        }
        else if (bestLabel == "level_2"){
            fillLevel = 0.50;   // Half full (50%)
        }
        else if (bestLabel == "level_3"){
            fillLevel = 0.75;   // Three-quarters full (75%)
        }
        else if (bestLabel == "level_4"){
            fillLevel = 100;  // Full (100%)
        }
        else{
            fillLevel = -1; // Unknown state
        }

        runMainJob(); // Sending the photo and other values to the server
        enterSleepMode();        
        
        // Clean up camera
        // ei_camera_deinit();
        // // Enter deep sleep for 60 seconds (60 * 1000000 microseconds)
        // ei_printf("Entering deep sleep for 60 seconds...\r\n");
        // esp_deep_sleep(60ULL * 1000000ULL);
    }
}


#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif