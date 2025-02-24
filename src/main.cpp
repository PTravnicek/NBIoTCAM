#include <Arduino.h>
#include "esp_camera.h"
#include "DFRobot_AXP313A.h"
#include "DFRobot_GNSS.h"
#include "driver/rtc_io.h"

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
// AXP Power Management
// ---------------------------------------------------------------------------
DFRobot_AXP313A axp;

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
const String SERVER_IP = "34.75.62.225";
const int    TCP_PORT  = 8009; 
const int    UDP_PORT  = 8094;

// GNSS Mode selection: Choose either INDOOR or OUTDOOR
enum Mode { INDOOR, OUTDOOR };
Mode currentMode = INDOOR;  // Set your desired mode here: INDOOR or OUTDOOR
DFRobot_GNSS_I2C gnss(&Wire, GNSS_DEVICE_ADDR);
bool gnssConnected = false;

// Mock data (in a real scenario, you'd fetch from GPS or sensors)
const double mockLatitude = 50.1234;   // Example latitude
const double mockLongitude = 14.1234;  // Example longitude
const double mockAltitude = 123.4;     // Example altitude (meters)
int batteryPercentage = 42;            // Example battery status

bool doCameraTCPJob = true;           // Takes a photo and sends it to server
// ---------------------------------------------------------------------------
// Camera Frame Buffer
// We'll store the last captured frame here.
// ---------------------------------------------------------------------------
camera_fb_t *capturedFrame = nullptr;

// ---------------------------------------------------------------------------
// GPS Global Variables
// ---------------------------------------------------------------------------
struct GPSData {
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double speed = 0.0;     // Speed over ground
    double course = 0.0;    // Course over ground
    uint8_t satellites = 0; // Number of satellites used
    struct {
        int year = 0;
        int month = 0;
        int day = 0;
        int hour = 0;
        int minute = 0;
        int second = 0;
    } timestamp;
} gpsData;

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
void getGPS();

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  // Start Serial for debug
  Serial.begin(115200);
  delay(1000);
  while(!gnss.begin()){
    Serial.println("NO Deivces !");
    delay(1000);
  }
  gnss.enablePower();    
  gnss.setGnss(eGPS_GLONASS);
  gnss.setRgbOn();

  getGPS();
  delay(1000);

  if (doCameraTCPJob) {
    // 1) Initialize the camera
    if (!initCamera()) {
      debugln("Camera init failed. Check connections/settings.");
      return;
    }
    
    // 2) Initialize the BC95 modem
    bc95_modem.begin(9600, SERIAL_8N1, BC95_RX_PIN, BC95_TX_PIN);
    debugln("BC95 modem initialized.");

    // 3) Capture a single photo in 'capturedFrame'
    capturePhoto();
    
    // 4) Send the captured photo over TCP (if available)
    if (capturedFrame) {
      sendPhotoOverTCP(capturedFrame->buf, capturedFrame->len);
      
      // IMPORTANT: Release the frame buffer when you're done
      esp_camera_fb_return(capturedFrame);
      capturedFrame = nullptr;
    }
  }

  sendUDPMessage();
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
  // You could periodically capture/send another photo here if desired
  // ...
  delay(1000);
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
  config.frame_size   = FRAMESIZE_UXGA;  
  config.jpeg_quality = 63;
  config.fb_count     = 1;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  // If PSRAM is found, allow higher quality and double buffering
  if (psramFound()) {
    config.jpeg_quality = 10;  
    config.fb_count     = 2;   
    config.grab_mode    = CAMERA_GRAB_LATEST;
  } else {
    // Without PSRAM, reduce frame size
    config.frame_size  = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    debug("Camera init failed with error 0x");
    debugln(String(err, HEX));
    return false;
  }

  // Sensor adjustments
  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  
  return true;
}

// ---------------------------------------------------------------------------
// capturePhoto()
//    Grabs a single frame and stores in global 'capturedFrame'.
// ---------------------------------------------------------------------------
void capturePhoto() {
  capturedFrame = esp_camera_fb_get();
  if (!capturedFrame) {
    debugln("Camera capture failed!");
    return;
  }
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
String sendATCommand(const String &cmd, unsigned long waitMs = 3000) {
  debugln("[BC95 SEND] " + cmd);
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
    debugln("[BC95 RSP] " + response);
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
  String lineProtocol = "gps_data,device=esp32 ";
  lineProtocol += "latitude=" + String(gpsData.latitude, 6) + ",";
  lineProtocol += "longitude=" + String(gpsData.longitude, 6) + ",";
  lineProtocol += "altitude=" + String(gpsData.altitude, 1) + ",";
  lineProtocol += "speed=" + String(gpsData.speed, 2) + ",";
  lineProtocol += "course=" + String(gpsData.course, 2) + ",";
  lineProtocol += "satellites=" + String(gpsData.satellites) + ",";
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
  sendATCommand("AT+NRB");       // Restart module
  // sendATCommand("AT+CFUN=0");    // Deactivate radio before setting multitone
  // sendATCommand("AT+NCONFIG=MULTITONE,TRUE"); // Configure UE Behaviour
  sendATCommand("AT+CFUN=1");    // Full functionality mode
  sendATCommand("AT+CGATT=1");   // Attach to network
  sendATCommand("AT+QREGSWT=2"); // Disable Huawei platform registration
  // sendATCommand("AT+NCONFIG?"); // Check UE Behaviour settings
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

  // 1) Initialize the module
  initializeModule();

  // 2) Create a TCP socket: "AT+NSOCR=STREAM,6,0,1"
  String socketResp = sendATCommand("AT+NSOCR=STREAM,6,0,1", 300);
  int socketID = parseSocketID(socketResp);
  if (socketID < 0) {
    debugln("Failed to create TCP socket!");
    return false;
  }
  debugln("Got TCP socket ID: " + String(socketID));

  // 3) Connect to server: "AT+NSOCO=<socket>,<ip>,<port>"
  String connectCmd = "AT+NSOCO=" + String(socketID) + "," + SERVER_IP + "," + String(TCP_PORT);
  String response;
  
  // Attempt to connect, retrying on ERROR with a maximum of 5 attempts
  const int MAX_RETRIES = 10;
  int retryCount = 0;

  do {
      response = sendATCommand(connectCmd, 6000);
      debug("[BC95 RSP] " + response + "\n");
      
      if (response.indexOf("ERROR") != -1) {
          retryCount++;
          if (retryCount >= MAX_RETRIES) {
              debugln("Max retries reached. Exiting connection loop.");
              break;
          }
          debugln("Received ERROR. Retrying connection...");
      }
  } while (response.indexOf("ERROR") != -1);

  // 4) Send data in chunks
  const size_t CHUNK_SIZE = 1300;
  size_t offset = 0;
  

  // Send initiation sequence (4 bytes: 0x00 0x00 0x00 0x00)
  const uint8_t BGN_MARKER[] = {0x00, 0x00, 0x00, 0x00};
  String bgnMarkerHex = bufferToHex(BGN_MARKER, 4);
  String bgnCmd = "AT+NSOSD=" + String(socketID) + ",4," + bgnMarkerHex;
  sendATCommand(bgnCmd);
  while (offset < length) {
    size_t thisChunkSize = min(CHUNK_SIZE, length - offset);
    String hexChunk = bufferToHex(buffer + offset, thisChunkSize);
    String sendCmd = "AT+NSOSD=" + String(socketID) + "," 
                             + String(thisChunkSize) + ","
                             + hexChunk;
    sendATCommand(sendCmd);
    offset += thisChunkSize;
  }

  // Send termination sequence (4 bytes: 0xFF 0xFF 0xFF 0xFF)
  const uint8_t END_MARKER[] = {0xFF, 0xFF, 0xFF, 0xFF};
  String endMarkerHex = bufferToHex(END_MARKER, 4);
  String endCmd = "AT+NSOSD=" + String(socketID) + ",4," + endMarkerHex;
  sendATCommand(endCmd);

  // Close socket and cleanup
  sendATCommand("AT+NSOCL=" + String(socketID), 300);
  return true;
}


/**
 * Send a UDP message.
 *
 * Initializes the module, creates a UDP socket, sends a HEX-encoded
 * Telegraf line protocol message, and closes the socket. If the UDP
 * send command returns an error, it retries up to 5 times.
 */
void sendUDPMessage() {
  initializeModule();

  // Prepare message in HEX
  String hexMessage = prepareHexMessage();
  int hexMessageBytes = hexMessage.length() / 2;

  debugln("\nLine Protocol as HEX:");
  debugln(hexMessage);

  // Create a UDP socket
  String respSocketIDUDP = sendATCommand("AT+NSOCR=DGRAM,17,0,1");
  int socketIDUDP = parseSocketID(respSocketIDUDP);
  debugln("Parsed UDP socket ID: " + String(socketIDUDP));

  if (socketIDUDP >= 0) {
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
    while (attempt < MAX_ATTEMPTS) {
      response = sendATCommand(cmdSendUDPMessage, 3000);
      if (response.indexOf("ERROR") == -1) {
        debugln("UDP message sent successfully on attempt " + String(attempt + 1));
        break;
      } else {
        attempt++;
        debugln("UDP message send error. Attempt " + String(attempt) + " failed. Retrying...");
        delay(500);  // Optional: add a small delay before retrying
      }
    }
    if (attempt >= MAX_ATTEMPTS) {
      debugln("UDP message failed to send after " + String(MAX_ATTEMPTS) + " attempts.");
    }

    // Close the UDP socket
    sendATCommand("AT+NSOCL=" + String(socketIDUDP), 100);
  } else {
    debugln("Failed to parse UDP socket ID!");
  }
}


/**
 * getGPS()
 *
 * Attempts to acquire a GNSS fix.
 */
void getGPS() {
  if (currentMode == OUTDOOR) {  
    sTim_t utc = gnss.getUTC();
    sTim_t date = gnss.getDate();
    sLonLat_t lat = gnss.getLat();
    sLonLat_t lon = gnss.getLon();
    
    // Store GPS data in global structure
    gpsData.latitude = lat.latitudeDegree;
    gpsData.longitude = lon.lonitudeDegree;
    gpsData.altitude = gnss.getAlt();
    gpsData.satellites = gnss.getNumSatUsed();
    gpsData.speed = gnss.getSog();
    gpsData.course = gnss.getCog();
    
    // Store timestamp
    gpsData.timestamp.year = date.year;
    gpsData.timestamp.month = date.month;
    gpsData.timestamp.day = date.date;
    gpsData.timestamp.hour = utc.hour;
    gpsData.timestamp.minute = utc.minute;
    gpsData.timestamp.second = utc.second;

    Serial.println("");
    Serial.print(date.year);
    Serial.print("/");
    Serial.print(date.month);
    Serial.print("/");
    Serial.print(date.date);
    Serial.print("/");
    Serial.print(utc.hour);
    Serial.print(":");
    Serial.print(utc.minute);
    Serial.print(":");
    Serial.print(utc.second);
    Serial.println();
    Serial.println((char)lat.latDirection);
    Serial.println((char)lon.lonDirection);

    // Serial.print("lat DDMM.MMMMM = ");
    // Serial.println(lat.latitude, 5);
    // Serial.print(" lon DDDMM.MMMMM = ");
    // Serial.println(lon.lonitude, 5);
    Serial.print("lat degree = ");
    Serial.println(lat.latitudeDegree,6);
    Serial.print("lon degree = ");
    Serial.println(lon.lonitudeDegree,6);

    Serial.print("star userd = ");
    Serial.println(gpsData.satellites);
    Serial.print("alt high = ");
    Serial.println(gpsData.altitude);
    Serial.print("sog =  ");
    Serial.println(gpsData.speed);
    Serial.print("cog = ");
    Serial.println(gpsData.course);
    Serial.print("gnss mode =  ");
    Serial.println(gnss.getGnssMode());
  }
  else if (currentMode == INDOOR) {
    // Store mock data in global structure
    gpsData.latitude = mockLatitude;
    gpsData.longitude = mockLongitude;
    gpsData.altitude = mockAltitude;
    gpsData.satellites = 0;
    gpsData.speed = 0;
    gpsData.course = 0;
    
    gnssConnected = false;
    
    Serial.println("Using mock coordinates as fallback:");
    Serial.print("Mock Latitude:  "); Serial.println(mockLatitude, 6);
    Serial.print("Mock Longitude: "); Serial.println(mockLongitude, 6);
    Serial.print("Mock Altitude:  "); Serial.println(mockAltitude);
  }
}