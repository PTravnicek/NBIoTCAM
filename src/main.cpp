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
#include <Adafruit_AHTX0.h>
#include <U8g2lib.h>

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


// ---------------------------------------------------------------------------
// GNSS
// ---------------------------------------------------------------------------
DFRobot_GNSS_I2C gnss(&Wire ,GNSS_DEVICE_ADDR);
bool useGNSS = false;

// ---------------------------------------------------------------------------
// AXP Power Management
// ---------------------------------------------------------------------------
DFRobot_AXP313A axp;
#define VDD_POWER 13  // 


// ---------------------------------------------------------------------------
// Mini OLED display
// ---------------------------------------------------------------------------
#define OLED_SCK    SCK
#define OLED_MOSI   MOSI
#define OLED_CS     18
#define OLED_A0     3
#define OLED_RST    38

// Using the correct constructor for LD7032 display
U8G2_LD7032_60X32_F_4W_SW_SPI u8g2(U8G2_R0, OLED_SCK, OLED_MOSI, OLED_CS, OLED_A0, OLED_RST);

bool debugShowATcommands = false; // showing Quectel configuration details
bool showOLED = false;            // Show OLED only after reset, not after sleep wakeup 

void oled_display(const char* line1, const char* line2, const char* line3) {
  if (!showOLED) return;  // Skip OLED display if not needed
  
  u8g2.begin();
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
Adafruit_AHTX0 aht;

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
const String SERVER_IP = "www.iot-magic.com";  // tohle je stara IP:"35.231.115.19"<-pred utokem, "34.75.62.225";
// node-red http://35.211.34.129:1880/
// iot-magic.com odkazuje na DNS namecheap na IP 35.211.34.129
// IP not hardcoded, because it might change even though its static
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


// Add global variables to store temp and humidity
struct SensorData {
    float temperature = 0.0;
    float humidity = 0.0;
} sensorData;

// Sleep configuration
#define uS_TO_S_FACTOR 1000000ULL // Conversion factor for micro seconds to seconds
#define DEFAULT_SLEEP_TIME 3600  // seconds

// GNSS configuration
#define TIMEOUT_DURATION 120000   // Timeout duration in milliseconds (2 min) for GPS 
#define TIMEZONE_OFFSET 2         // Local timezone offset from network time (+2 hours)
bool gpsFixObtained = false;      // Track if GPS fix was successful


// RTC memory variables to store
RTC_DATA_ATTR int customSleepDuration = DEFAULT_SLEEP_TIME; // seconds
RTC_DATA_ATTR int  normalSleepDuration = DEFAULT_SLEEP_TIME;
RTC_DATA_ATTR bool wasNightSleep = false;

// ---------------------------------------------------------------------------
// Sleep configuration
// ---------------------------------------------------------------------------
int napTimeHour = 22;                  // Time when the unit goes to sleep for the whole night
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

bool initCamera();
void capturePhoto();
String sendATCommand(const String &cmd, unsigned long waitMs);
int parseSocketID(const String &rawResponse);
void initializeModule();
String bufferToHex(const uint8_t *data, size_t length);
bool sendPhotoOverTCP(const uint8_t *buffer, size_t length);
void sendUDPMessage();
void receiveUDPMessage();
void readAHTSensor();
void enterSleepMode();
void getGNSS();
bool parseNetworkTime(String timeResponse, int &hour, int &minute);
bool checkNightTime(int hour, int minute);

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  // Start Serial for debug
  Serial.begin(115200);
  delay(1000);

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

  // Initialize VDD_POWER pin and set it LOW (turn ON)
  rtc_gpio_hold_dis((gpio_num_t)VDD_POWER); // Release any previous hold
  pinMode(VDD_POWER, OUTPUT);
  digitalWrite(VDD_POWER, LOW);  // Turn VDD line ON

  // Initialize analog pin
  pinMode(ANALOG_PIN, INPUT);
  
  u8g2.begin();
  oled_display("main", "setup", "begin");
  delay(1000);


  // Read and display initial analog value
  // readAndDisplayAnalog();
  // delay(1000);  // Show the value for a second

  // Initialize AHT sensor
  if (!aht.begin()) {
    debugln("Failed to find AHT20");
  }  
  readAHTSensor();

  if (useGNSS){
    // Initialize GNSS module
    while(!gnss.begin()){
      debugln("NO Devices !");
      oled_display("GNSS", "not", "found");
      delay(1000);
    }
    gnss.enablePower();      
    gnss.setGnss(eGPS_GLONASS); // USE gps + glonass
    gnss.setRgbOff();
    getGNSS(); // Simple function to show gps fix if obtained
  }
  // Initialize BC95 modem regardless of camera job
  bc95_modem.begin(9600, SERIAL_8N1, BC95_RX_PIN, BC95_TX_PIN);
  debugln("BC95 modem RX TX pins set.");

  // Initialize module first to ensure network connection
  initializeModule();
  
  if (moduleInitialized) {
    // Get current time from network after successful connection
    debugln("Checking network time...");
    String timeResponse = sendATCommand("AT+CCLK?", 2000);
    debugln("Network time: " + timeResponse);
    
    // Parse time and check if it's night time
    int networkHour, networkMinute;
    if (parseNetworkTime(timeResponse, networkHour, networkMinute)) {
      if (checkNightTime(networkHour, networkMinute)) {
        // Device will go to sleep in checkNightTime() if it's night
        // This code won't be reached if it's night time
      }
    } else {
      debugln("Failed to parse network time, continuing with normal operation");
    }
  } else {
    debugln("Module not initialized, skipping time check");
  }

  if (doCameraTCPJob) {
    // Initialize camera only if we need it
    // enterSleepMode();//going to sleep here resulted in 865uA consumption, GREAT
    if (!initCamera()) {
      debugln("Camera init failed. Check connections/settings.");
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
  // Send UDP message regardless of camera job
  sendUDPMessage();

  enterSleepMode();
}


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
      gpsFixObtained = true;  // Set flag when GPS fix is successful
      return;
    }
    delay(1000); // Wait 1 second before next check
    oled_display("GNSS", "time", String(millis() - startTime).c_str());
  }
  // If we reach here, timeout occurred without GPS fix
  oled_display("GNSS", "timeout", "no fix");
  gpsFixObtained = false;  // Ensure flag is set to false on timeout
}

// ---------------------------------------------------------------------------
// AHT Sensor Helper Functions
// ---------------------------------------------------------------------------
void readAHTSensor() {
  sensors_event_t humidity, temp;  
  aht.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
  
  // Store in global variables
  sensorData.temperature = temp.temperature;
  sensorData.humidity = humidity.relative_humidity;
  
  debug("Temperature: ");debug(temp.temperature);debugln(" degrees C");
  debug("Humidity: ");debug(humidity.relative_humidity);debugln("%");
}


// ---------------------------------------------------------------------------
// initCamera()
//    Powers on camera via AXP and initializes the ESP camera driver.
// ---------------------------------------------------------------------------
bool initCamera() {
  // Power on the camera
  while (axp.begin() != 0) {
    debugln("AXP313A init error, retrying...");
    delay(1000);
  }
  axp.enableCameraPower(axp.eOV2640);
  axp.setShutdownKeyLevelTime(axp.eTime6s);
  // enterSleepMode(); //going to sleep here resulted in 864uA consumption, GREAT

  // Camera config
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;  // new
  config.pin_sccb_scl = SIOC_GPIO_NUM;  // new
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;

    // JPEG for compressed capture
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_SVGA;  
  config.jpeg_quality = 4;
  config.fb_count     = 1;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  // enterSleepMode(); //going to sleep here resulted in 864uA consumption, GREAT
  // If PSRAM is found, allow higher quality and double buffering
  if (psramFound()) {
    debugln("psramFound");
    config.jpeg_quality = 4; // 4 is the best quality, 63 is the worst  
    config.fb_count     = 2;   
    config.grab_mode    = CAMERA_GRAB_LATEST;
  } else {
    // Without PSRAM, reduce frame size
    debugln("psram WAS NOT Found!!!");
    config.jpeg_quality = 20; // Dont want higher quality
    config.frame_size  = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }
  // enterSleepMode(); //going to sleep here resulted in 864uA consumption, GREAT

  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  // enterSleepMode(); //going to sleep here resulted in 19.56mA consumption, PROBLEM WITH CAMERA
  if (err != ESP_OK) {
    debug("Camera init failed with error 0x");
    debugln(String(err, HEX));
    return false;
  }

  // Sensor adjustments - use camera_sensor_t instead of sensor_t
  camera_sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    // s->set_brightness(s, 0);     // -2 to 2
    // s->set_contrast(s, 0);       // -2 to 2
    // s->set_saturation(s, 0);     // -2 to 2
    // s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    // s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    // s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    // s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    // s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    // s->set_ae_level(s, 0);       // -2 to 2
    // s->set_aec_value(s, 300);    // 0 to 1200
    // s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
    // s->set_agc_gain(s, 0);       // 0 to 30
    // s->set_gainceiling(s, (gainceiling_t)2);  // 0 to 6
    s->set_bpc(s, 1);            // 0 = disable , 1 = enable
    // s->set_wpc(s, 1);            // 0 = disable , 1 = enable
    // s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
    // s->set_lenc(s, 1);           // 0 = disable , 1 = enable
    // s->set_dcw(s, 1);            // 0 = disable , 1 = enable
    // s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
    s->set_aec2(s, 1);           // 0 = disable , 1 = enable


    s->set_vflip(s, 0);          // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
    s->set_gain_ctrl(s, 1);         //auto gain 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);          // Auto White Balance ENABLE(0 = disable , 1 = enable)
    s->set_exposure_ctrl(s, 1);     // auto exposure ON
    // s->set_brightness(s, 0);        // (-2 to 2) - set brightness
    // s->set_agc_gain(s, 0);          // set gain manually (0 - 30)
    // s->set_aec_value(s, 110);         // set exposure manually  (0-1200)
  }  
  return true;
}

// ---------------------------------------------------------------------------
// capturePhoto()
//    Grabs a single frame and stores in global 'capturedFrame'.
// ---------------------------------------------------------------------------
void capturePhoto() {
  // Capture and discard initial frames
  for (int i = 0; i < 6; i++) {
      camera_fb_t * temp_fb = esp_camera_fb_get();
      if (temp_fb) {
          esp_camera_fb_return(temp_fb);
          debugln("Camera frame discard...");
      }
      delay(500); // Wait between frames
  }
  // Now capture the actual frame you want to use
  capturedFrame = esp_camera_fb_get();
  debugln("Camera frame stored...");
  delay(1000); // Wait for 1 second

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
 * Send an AT command and capture its response.
 *
 * @param cmd The AT command to send.
 * @param waitMs How long to wait for a response (ms).
 * @return The response from the modem (trimmed).
 */
String sendATCommand(const String &cmd, unsigned long waitMs = 1000) {
  // debugln("[BC95 SEND] " + cmd);
  bc95_modem.print(cmd + "\r\n");

  String response;

  // Collect response until timeout
  unsigned long start = millis();
  while (millis() - start < waitMs) {
    while (bc95_modem.available()) {
      response += (char)bc95_modem.read();
    }
  }
  response.trim();

  if (response.length() > 0) {
    debugln("[BC95 RSP] " + response) + '\n';
  }

  return response;
}

/**
 * Parse the socket ID from the raw response.
 */
int parseSocketID(const String &rawResponse) {
  String temp = rawResponse;
  temp.trim();

  int startIndex = 0;
  while (true) {
    int endIndex = temp.indexOf('\n', startIndex);
    if (endIndex == -1) {
      endIndex = temp.length();
    }

    String line = temp.substring(startIndex, endIndex);
    line.trim();

    // If the line is non-empty and not "OK", convert to int
    if (line.length() > 0 && line != "OK") {
      return line.toInt();
    }

    if (endIndex >= temp.length()) {
      break;
    }
    startIndex = endIndex + 1;
  }
  return -1; // Not found
}


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
 * Prepare a HEX-encoded message in Telegraf line protocol format.
 *
 * Builds a line protocol string from mock data, then converts it to HEX.
 */
String prepareHexMessage() {
  // Update battery reading before preparing message
  readAndDisplayAnalog();
  
  String lineProtocol = "gps_data,device=esp32 ";
  lineProtocol += "gpsFIX=" + String(gpsFixObtained ? "true" : "false") + ",";
  lineProtocol += "temperature=" + String(sensorData.temperature, 1) + ",";
  lineProtocol += "humidity=" + String(sensorData.humidity, 1) + ",";
  lineProtocol += "battery=" + String(batteryPercentage);

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

  // Set Full Functionality (Monitor closely for reboots here)
  String cfunResponse;
  int cfunRetries = 0;
  const int MAX_CFUN_RETRIES = 3;
  do {
    debugln("Attempting AT+CFUN=1...");
    oled_display("set", "function", "mode");
    cfunResponse = sendATCommand("AT+CFUN=1", 5000); // Increased timeout for CFUN
    debugln("AT+CFUN=1 response: " + cfunResponse);
    if (cfunResponse.indexOf("OK") == -1) {
      cfunRetries++;
      debugln("AT+CFUN=1 failed, retry " + String(cfunRetries));
      oled_display("function", "mode", "retry");
      delay(2000); // Wait before retrying
    }
  } while (cfunResponse.indexOf("OK") == -1 && cfunRetries < MAX_CFUN_RETRIES);

  if (cfunResponse.indexOf("OK") == -1) {
     debugln("ERROR: Failed to set AT+CFUN=1 after retries.");
     oled_display("function", "mode", "error");
     // return; // Consider stopping
  }

  // --- Add significant delay after CFUN=1 ---
  debugln("Waiting 2 seconds after AT+CFUN=1 for radio stabilization...");
  oled_display("radio", "stabilize", "wait");
  delay(2000);
  // --- End Delay ---

  // Check and set the NB-IoT band if needed
  debugln("Checking current band configuration...");
  oled_display("check", "band", "config");
  String currentBand = sendATCommand("AT+NBAND?", 1000);
  debugln("Current band configuration: " + currentBand);
  
  // Only set band if it's not already configured properly
  if (currentBand.indexOf("255") != -1 || currentBand.indexOf("ERROR") != -1) {
    debugln("Setting NB-IoT band to 8 (900MHz)...");
    oled_display("set", "band", "8");
    String bandResponse = sendATCommand("AT+NBAND=8", 1000);
    debugln("Band setting response: " + bandResponse);
    
    // Verify band setting
    delay(500);
    String verifyBand = sendATCommand("AT+NBAND?", 1000);
    debugln("Verified band setting: " + verifyBand);
  }

  // Trigger network attachment
  debugln("Attempting network attach (AT+CGATT=1)...");
  oled_display("attach", "to", "network");
  sendATCommand("AT+CGATT=1", 5000); // Command to attach
  // delay(2000); // Wait after attach command

  // Keep checking registration status
  String CEREGresponse;
  int ceregChecks = 0;
  const int MAX_CEREG_CHECKS = 30; // Check for up to 60 seconds
  bool registered = false;

  do {
    ceregChecks++;
    CEREGresponse = sendATCommand("AT+CEREG?", 1500);
    debugln("AT+CEREG? response (" + String(ceregChecks) + "/" + String(MAX_CEREG_CHECKS) + "): " + CEREGresponse);

    if (CEREGresponse.indexOf("+CEREG:0,1") != -1 || CEREGresponse.indexOf("+CEREG:0,5") != -1) {
       registered = true;
       oled_display("network", "registered", "OK");
       break; // Exit loop if registered (home or roaming)
    } else if (CEREGresponse.indexOf("+CEREG:0,2") != -1) {
      debugln("Network searching...");
      oled_display("searching", "network", String(ceregChecks).c_str());
    } else if (CEREGresponse.indexOf("+CEREG:0,0") != -1) {
      debugln("Not registered, not searching.");
      oled_display("not", "registered", "error");
    } else if (CEREGresponse.indexOf("+CEREG:0,3") != -1) {
      debugln("Registration denied!");
      oled_display("registration", "denied", "error");
      // Consider stopping or specific error handling
    } else {
       debugln("Waiting for registration...");
       oled_display("waiting", "for", "network");
    }
    delay(2000); // Wait 2 seconds between checks

  } while (!registered && ceregChecks < MAX_CEREG_CHECKS);

  if (registered) {
      if (CEREGresponse.indexOf("+CEREG:0,1") != -1) {
        debugln("Network registered successfully (home network)");
        oled_display("home", "network", "OK");
      } else {
        debugln("Network registered successfully (roaming)");
        oled_display("roaming", "network", "OK");
      }
      sendATCommand("AT+CGPADDR", 1000);    // Query IP address

      // --- Query the current band ---
      debugln("Querying current operating band (AT+NBAND?)...");
      sendATCommand("AT+NBAND?", 1000); // Add this line to query the band
      // --- End Band Query ---

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
      
      debugln("4. Available frequency bands (AT+NBAND?)...");
      sendATCommand("AT+NBAND?", 1000);
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

  // 2) Create a TCP socket: "AT+NSOCR=STREAM,6,0,1"
  oled_display("create", "TCP", "socket");
  String socketResp = sendATCommand("AT+NSOCR=STREAM,6,0,1", 300);
  int socketID = parseSocketID(socketResp);
  if (socketID < 0) {
    debugln("Failed to create TCP socket!");
    oled_display("TCP", "socket", "error");
    return false;
  }
  debugln("Got TCP socket ID: " + String(socketID));

  // 3) Connect to server: "AT+NSOCO=<socket>,<ip>,<port>"
  oled_display("connect", "to", "server");
  String connectCmd = "AT+NSOCO=" + String(socketID) + "," + SERVER_IP + "," + String(TCP_PORT);
  String response;
  
  // Attempt to connect, retrying on ERROR with a maximum of 10 attempts
  const int MAX_RETRIES = 10;
  int retryCount = 0;

  do {
      response = sendATCommand(connectCmd, 6000);
      debug("[BC95 RSP] " + response + "\n");
      
      if (response.indexOf("ERROR") != -1) {
          retryCount++;
          oled_display("TCP", "retry", String(retryCount).c_str());
          if (retryCount >= MAX_RETRIES) {
              debugln("Max retries reached. Exiting connection loop.");
              oled_display("TCP", "max", "retries");
              break;
          }
          debugln("Received ERROR. Retrying connection...");
      }
  } while (response.indexOf("ERROR") != -1);

  // 4) Send data in chunks
  const size_t CHUNK_SIZE = 1300;
  size_t offset = 0;
  int totalChunks = (length + CHUNK_SIZE - 1) / CHUNK_SIZE;  // Ceiling division

  // Send initiation sequence (4 bytes: 0x00 0x00 0x00 0x00)
  oled_display("send", "start", "marker");
  const uint8_t BGN_MARKER[] = {0x00, 0x00, 0x00, 0x00};
  String bgnMarkerHex = bufferToHex(BGN_MARKER, 4);
  String bgnCmd = "AT+NSOSD=" + String(socketID) + ",4," + bgnMarkerHex;
  
  String responsebgnCmd;
  bool markerSent = false;
  int retries = 0;

  while (!markerSent && retries < MAX_RETRIES) {
    responsebgnCmd = sendATCommand(bgnCmd);
    if (responsebgnCmd.indexOf("ERROR") == -1) {
      markerSent = true;
    } else {
      retries++;
      oled_display("marker", "retry", String(retries).c_str());
      debugln("Error sending start marker, retry " + String(retries));
    }
  }

  if (!markerSent) {
    debugln("Failed to send start marker after " + String(MAX_RETRIES) + " retries");
    oled_display("marker", "send", "error");
    return false;
  }

  // Send data chunks
  oled_display("send", "data", "chunks");
  while (offset < length) {
    size_t thisChunkSize = min(CHUNK_SIZE, length - offset);
    int currentChunk = (offset / CHUNK_SIZE) + 1;
    String hexChunk = bufferToHex(buffer + offset, thisChunkSize);
    String sendCmd = "AT+NSOSD=" + String(socketID) + "," 
                             + String(thisChunkSize) + ","
                             + hexChunk;
    
    String response;
    bool chunkSent = false;
    const int MAX_RETRIES = 10;
    int retries = 0;

    while (!chunkSent && retries < MAX_RETRIES) {
      response = sendATCommand(sendCmd);
      if (response.indexOf("ERROR") == -1) {
        chunkSent = true;
        // Show progress
        char progress[10];
        snprintf(progress, sizeof(progress), "%d/%d", currentChunk, totalChunks);
        oled_display("chunk", progress, "sent");
      } else {
        retries++;
        oled_display("chunk", "retry", String(retries).c_str());
        debugln("Error sending chunk, retry " + String(retries));
      }
    }

    if (!chunkSent) {
      debugln("Failed to send chunk after " + String(MAX_RETRIES) + " retries");
      oled_display("chunk", "send", "error");
      return false;
    }

    offset += thisChunkSize;
  }

  // Send termination sequence (4 bytes: 0xFF 0xFF 0xFF 0xFF)
  oled_display("send", "end", "marker");
  const uint8_t END_MARKER[] = {0xFF, 0xFF, 0xFF, 0xFF};
  String endMarkerHex = bufferToHex(END_MARKER, 4);
  String endCmd = "AT+NSOSD=" + String(socketID) + ",4," + endMarkerHex;
  sendATCommand(endCmd);

  // Close socket and cleanup
  oled_display("close", "TCP", "socket");
  sendATCommand("AT+NSOCL=" + String(socketID), 300);
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


void enterSleepMode() {
  debugln("Preparing for sleep...");

  // --- Power Down Camera and AXP Power Management ---
  debugln("Resetting camera sensor...");
  
  // Get camera sensor for reset commands
  camera_sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    // Put camera sensor into software standby/powerdown mode
    s->set_reg(s, 0x3008, 0xff, 0x42); // Software standby mode for OV2640
    delay(10);
  }
  
  debugln("Deinitializing camera...");
  esp_camera_deinit(); // Always deinitialize camera
  delay(100); // 
  
  // Stop LEDC timer used for XCLK
  debugln("Stopping camera clock...");
  ledc_timer_rst(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
  ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
  
  // Reset camera GPIO pins to reduce power consumption
  debugln("Resetting camera GPIO pins...");
  gpio_reset_pin((gpio_num_t)XCLK_GPIO_NUM);
  gpio_reset_pin((gpio_num_t)SIOD_GPIO_NUM);
  gpio_reset_pin((gpio_num_t)SIOC_GPIO_NUM);
  gpio_reset_pin((gpio_num_t)Y9_GPIO_NUM);
  gpio_reset_pin((gpio_num_t)Y8_GPIO_NUM);
  gpio_reset_pin((gpio_num_t)Y7_GPIO_NUM);
  gpio_reset_pin((gpio_num_t)Y6_GPIO_NUM);
  gpio_reset_pin((gpio_num_t)Y5_GPIO_NUM);
  gpio_reset_pin((gpio_num_t)Y4_GPIO_NUM);
  gpio_reset_pin((gpio_num_t)Y3_GPIO_NUM);
  gpio_reset_pin((gpio_num_t)Y2_GPIO_NUM);
  gpio_reset_pin((gpio_num_t)VSYNC_GPIO_NUM);
  gpio_reset_pin((gpio_num_t)HREF_GPIO_NUM);
  gpio_reset_pin((gpio_num_t)PCLK_GPIO_NUM);

  // Reset I2C/SCCB peripheral
  debugln("Resetting I2C peripheral...");
  i2c_driver_delete(I2C_NUM_0);
  
  // Disable camera power domains in ESP32
  debugln("Disabling camera power domains...");
  // Reset camera peripheral completely
  periph_module_disable(PERIPH_I2S0_MODULE);
  periph_module_disable(PERIPH_I2S1_MODULE);
  
  debugln("Disabling camera power...");
  
  // Method 1: Use the standard function first
  axp.disablePower(); // Disable AXP power
  delay(100);
  
  // Method 2: Manually ensure complete power shutdown by setting voltage registers to 0
  debugln("Manual camera power shutdown...");
  
  // Create a private access method to write registers directly
  // We'll access the AXP313A registers directly to ensure complete shutdown
  uint8_t powerDisable = 0x01;  // Disable power enable register
  uint8_t voltageZero = 0x00;   // Set voltage to 0
  
  // Disable power enable register (0x10)
  Wire.beginTransmission(0x36); // AXP313A I2C address
  Wire.write(0x10);
  Wire.write(powerDisable);
  Wire.endTransmission();
  delay(10);
  
  // Set ALDO voltage to 0 (register 0x16) 
  Wire.beginTransmission(0x36);
  Wire.write(0x16);
  Wire.write(voltageZero);
  Wire.endTransmission();
  delay(10);
  
  // Set DLDO voltage to 0 (register 0x17)
  Wire.beginTransmission(0x36);
  Wire.write(0x17);
  Wire.write(voltageZero);
  Wire.endTransmission();
  delay(10);
  
  debugln("Camera power rails set to zero voltage");
  
  // Try multiple approaches to ensure camera is completely off
  debugln("Multiple camera power-down attempts...");
  
  // Method 1: Reset AXP313A completely
  axp.begin(); // Re-initialize AXP313A without camera power
  
  // Method 2: If there's a camera power pin, set it to disable camera
  // (This might be board-specific - check your schematic)
  
  // Method 3: Set camera control pins to known safe states
  digitalWrite(SIOD_GPIO_NUM, LOW);
  digitalWrite(SIOC_GPIO_NUM, LOW);
  
  delay(1000); // Longer delay to allow camera to fully power down

  debugln("Stopping Bluetooth...");
  btStop();

  // --- Close All Sockets ---
  debugln("Attempting to close potential sockets (0, 1, 2)...");
  sendATCommand("AT+NSOCL=0", 300); // Try closing socket 0
  sendATCommand("AT+NSOCL=1", 300); // Try closing socket 1
  sendATCommand("AT+NSOCL=2", 300); // Try closing socket 2
  delay(500); // Give time for sockets to close

  // --- Detach from Network ---
  debugln("Detaching from network (AT+CGATT=0)...");
  sendATCommand("AT+CGATT=0", 5000); // Detach from GPRS service
  delay(1000); // Wait after detaching

  // --- Set Minimum Functionality ---
  debugln("Setting minimum functionality (AT+CFUN=0)...");
  sendATCommand("AT+CFUN=0", 5000); // Set minimum functionality to power down RF
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

  // --- Enter Deep Sleep ---
  esp_deep_sleep_start();
}

void receiveUDPMessage(int socketIDUDP) { // Pass the correct socket ID
    // Wait and check for incoming messages
    const int MAX_ATTEMPTS = 5;
    const int WAIT_TIME = 2000; // 2 seconds between checks

    // Removed incorrect AT commands like QREGSWT, CGPADDR, NSOCR, NSOST
    // We only need to read from the existing socket.

    debugln("Attempting to receive UDP data on socket " + String(socketIDUDP) + "...");

    for (int i = 0; i < MAX_ATTEMPTS; i++) {
        // Read from the correct socket ID passed to the function
        // Format: AT+NSORF=<socket>,<req_length>
        String readCmd = "AT+NSORF=" + String(socketIDUDP) + ",512";
        String response = sendATCommand(readCmd, 2000); // Increased timeout
        debugln("UDP Read Response (attempt " + String(i + 1) + "): " + response);

        if (response.indexOf("ERROR") == -1) {
            // Check if we received actual data (not just OK)
            // The response format is <socket>,<remote_ip>,<remote_port>,<length>,<data_hex>,<remaining_length>
            // Example: 0,192.168.1.100,5000,10,48656C6C6F...,0
            // Look for the comma separators to confirm data presence
            int firstComma = response.indexOf(',');
            if (firstComma != -1 && response.length() > firstComma + 1) {
                 // Basic check: Does the response look like it contains data fields?
                 // A more robust parsing could be added here if needed.
                 int secondComma = response.indexOf(',', firstComma + 1);
                 if (secondComma != -1) {
                    debugln("Received UDP data: " + response);
                    // TODO: Add parsing logic here if you need to process the received HEX data
                    // String hexData = ... // extract hex data part
                    // customSleepDuration = ... // parse and set based on data
                    break; // Exit loop after receiving data
                 } else {
                     debugln("Received response, but not in expected data format.");
                 }

            } else if (response.indexOf("OK") != -1 && response.length() <= 4) {
                 // Received only "OK" or similar short response, likely no data
                 debugln("No data available yet on socket " + String(socketIDUDP));
            } else {
                // Received something other than ERROR or simple OK
                debugln("Unexpected UDP read response: " + response);
            }
        } else {
            debugln("Error reading UDP socket " + String(socketIDUDP));
            // No need to break on error, maybe it works next time
        }
        debugln("Waiting for UDP response...");
        delay(WAIT_TIME);
    }

    // Close the socket *after* attempting to receive
    debugln("Closing UDP socket " + String(socketIDUDP) + " after receive attempts.");
    sendATCommand("AT+NSOCL=" + String(socketIDUDP), 300);
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

  // Create a UDP socket with specific local port
  String respSocketIDUDP = sendATCommand("AT+NSOCR=DGRAM,17,3014,1"); // Enable URC for received data
  debugln("Socket creation response: " + respSocketIDUDP);
  int socketIDUDP = parseSocketID(respSocketIDUDP);
  if (socketIDUDP < 0) {
      debugln("Failed to create UDP socket!");
      return; // Cannot proceed without a socket
  }
  debugln("Created UDP socket " + String(socketIDUDP) + " on port 3014");


    // Prepare the command to send the UDP message
    String cmdSendUDPMessage = "AT+NSOST=" + String(socketIDUDP) + "," +
                               SERVER_IP + "," +
                               String(UDP_PORT) + "," +
                               String(hexMessageBytes) + "," +
                               hexMessage;

    // Attempt sending the message up to 5 times if an error is received
    const int MAX_ATTEMPTS = 5;
    int attempt = 0;
    String response;
    bool sentSuccessfully = false;
    while (attempt < MAX_ATTEMPTS) {
      response = sendATCommand(cmdSendUDPMessage, 3000);
      if (response.indexOf("ERROR") == -1) {
        debugln("UDP message sent successfully on attempt " + String(attempt + 1));
        sentSuccessfully = true;
        break;
      } else {
        attempt++;
        debugln("UDP message send error. Attempt " + String(attempt) + " failed. Retrying...");
        delay(500);  // Optional: add a small delay before retrying
      }
    }
    if (!sentSuccessfully) {
      debugln("UDP message failed to send after " + String(MAX_ATTEMPTS) + " attempts.");
      // Close the socket even if send failed, to clean up
      sendATCommand("AT+NSOCL=" + String(socketIDUDP), 300);
      return; // Don't proceed to receive if send failed
    }

    // --- Wait for and Receive UDP Response ---
    // no receiving now
    // receiveUDPMessage(socketIDUDP); // This function now handles reading and closing

    // The socket closing is now handled inside receiveUDPMessage

    // Close the socket
    debugln("Closing UDP socket " + String(socketIDUDP) + " after receive attempts.");
    sendATCommand("AT+NSOCL=" + String(socketIDUDP), 300);
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
bool checkNightTime(int hour, int minute) {
  int localHour = (hour + TIMEZONE_OFFSET) % 24;  // Adjust for local timezone
  
  debugln("Network time: " + String(hour) + ":" + String(minute));
  debugln("Local time: " + String(localHour) + ":" + String(minute));
  
  if (localHour >= napTimeHour || localHour < wakeUpHour) {
    debugln("Night time detected! Going to extended sleep...");
    oled_display("Night", "time", "sleep");
    
    int minutesUntilWake = calculateSleepDuration(localHour, minute);
    customSleepDuration = minutesUntilWake * 60;  // Convert to seconds
    wasNightSleep = true;
    
    debugln("Sleeping for " + String(minutesUntilWake) + " minutes");
    enterSleepMode();
    return true;  // This won't be reached due to deep sleep
  }
  
  debugln("Day time - continuing normal operation");
  return false;
}


// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
  // code will not get to this point, sleep at the end of the setup
  delay(1000);  // Read every second
}