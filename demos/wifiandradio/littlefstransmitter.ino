#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <SPI.h>
#include <RH_RF95.h>
#include "FS.h"
#include <LittleFS.h>
#include <MS5837.h>
#include "time.h"
#include "config.h"   

//================================================================================================================================================
//                                                        Pin Definitions (XIAO ESP32-C6)

const int PIN_RF95_CS = D2;    // RFM95 CS
const int PIN_RF95_G0 = D3;    // RFM95 Interrupt
const int PIN_I2C_SDA = D4;
const int PIN_I2C_SCL = D5;
const int PIN_RF95_RST  = D6;    // RFM95 Reset
const int PIN_RF95_SCK  = D8;    // RFM95 Clock
const int PIN_RF95_MISO = D9;    // RFM95 Serial Out
const int PIN_RF95_MOSI = D10;   // RFM95 Serial In

//================================================================================================================================================
//                                                        Global Variables

const String COMPANY_NUMBER = "12345";   

// Depth control parameters (read_sensor() uses MAX_DEPTH)
const float MAX_DEPTH = 30.0;

// Competition / radio status
bool radio_available = false;

// Hardware objects
MS5837 pressureSensor;
WiFiServer server(80);
RH_RF95 rf95(PIN_RF95_CS, PIN_RF95_G0);

// NTP
const char* NTP_SERVER  = "pool.ntp.org";
const long GMT_OFFSET = -28800;
const int DAYLIGHT_OFFSET =  3600;

// LittleFS log file
const char* LOG_FILE = "/NanoFloat_datalog.csv";

// RFM95 frequency and TX interval
#define RF95_FREQ    900.0
#define TX_INTERVAL  5000

#define FORMAT_LITTLEFS_IF_FAILED true

// Demo timing
#define LOG_INTERVAL_MS        10000L   // one sensor reading every 10 s
#define LOGGING_DURATION_MS    180000L   // log for 3 minutes then transmit
#define WIFI_SERVE_DURATION_MS 300000L  // WiFi window: 5 minutes

//================================================================================================================================================
//                                                        Function Prototypes

void writeFile(fs::FS &fs, const char* path, const char* message);
void appendFile(fs::FS &fs, const char* path, const char* message);
bool sync_time();
void read_sensor(float &depth, float &pressure);
void save_data(float depth, float pressure);
void radiotransmit_data();
void wifitransmit_data();
void initialize_radio();

//================================================================================================================================================
//                                                        LittleFS Helpers

void writeFile(fs::FS &fs, const char* path, const char* message) {
  Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char* path, const char* message) {
  Serial.printf("Appending to file: %s\r\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("- failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("- message appended");
  } else {
    Serial.println("- append failed");
  }
  file.close();
}

//================================================================================================================================================
//                                                        Time Sync

bool sync_time() {
  Serial.println("\nSyncing time using NTP...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");

  configTime(GMT_OFFSET, DAYLIGHT_OFFSET, NTP_SERVER);

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("WARNING: Failed to obtain time.");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return false;
  }

  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  Serial.println("Time has been synced. WiFi disconnected.");
  return true;
}

//================================================================================================================================================
//                                                        Read Sensor

void read_sensor(float &depth, float &pressure) {
  pressureSensor.read();
  depth = pressureSensor.depth();
  pressure = pressureSensor.pressure() * 0.1;   // mbar -> kPa

  if (depth < 0) {
    depth = 0;
  }
  if (depth > MAX_DEPTH) {
    depth = MAX_DEPTH;
  }
}
//================================================================================================================================================
//                                                        Save Data

void save_data(float depth, float pressure) {
  struct tm timeinfo;
  String timestamp;

  if (getLocalTime(&timeinfo)) {
    char buf[30];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    timestamp = String(buf);
  } else {
    timestamp = String(millis());
  }

  String line = COMPANY_NUMBER + ", " + timestamp + ", " +
                String(depth, 2) + ", " + String(pressure, 2) + "\r\n";
  appendFile(LittleFS, LOG_FILE, line.c_str());

  Serial.print("Logged | Depth: ");
  Serial.print(depth, 2);
  Serial.print(" m | Pressure: ");
  Serial.print(pressure, 2);
  Serial.println(" kPA");
}

//================================================================================================================================================
//                                                        RFM9x Radio Transmit

void radiotransmit_data() {
  static unsigned long lastTX = 0;
  if (millis() - lastTX < TX_INTERVAL) return;
  lastTX = millis();

  if (!radio_available) {
    Serial.println("NanoFloat radio not available");
    return;
  }

  // Open log file from flash
  File file = LittleFS.open(LOG_FILE);
  if (!file || file.isDirectory()) {
    Serial.println("Failed to open log file");
    return;
  }

  Serial.println("NanoFloat transmitting log over radio...");
  static unsigned int packetnum = 0;

  // Skip the header line
  if (file.available()) {
    file.readStringUntil('\n');
  }

  // Transmit each data line as a separate packet
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    // Append a sequence number so the receiver can detect gaps
    String radioPacket = line + " | #" + String(packetnum++);

    char packetBuf[120];
    radioPacket.toCharArray(packetBuf, sizeof(packetBuf));

    Serial.print("Sending packet: ");
    Serial.println(packetBuf);

    delay(10);
    rf95.send((uint8_t *)packetBuf, strlen(packetBuf));
    rf95.waitPacketSent();
    delay(50);   // brief gap between packets 
  }

  file.close();
  Serial.println("NanoFloat radio transmission complete");
}

//================================================================================================================================================
//                                                        Fallback WiFi Transmit

void wifitransmit_data() {
  Serial.print("Connect to WiFi and open: ");
  Serial.println(WiFi.softAPIP());

  unsigned long start = millis();
  while (millis() - start < WIFI_SERVE_DURATION_MS) {
    WiFiClient client = server.available();
    if (client) {
      Serial.println("Client connected");
      String request = "";

      while (client.connected()) {
        if (client.available()) {
          char c = client.read();
          request += c;
          if (request.endsWith("\r\n\r\n")) break;
        }
      }

      if (request.indexOf("GET /download") >= 0) {
        File file = LittleFS.open(LOG_FILE);
        if (!file || file.isDirectory()) {
          Serial.println("Failed to open file for reading");
          client.println("HTTP/1.1 404 Not Found");
          client.println("Content-Type: text/plain");
          client.println();
          client.println("Log file not found.");
        } else {
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/csv");
          client.println("Content-Disposition: attachment; filename=\"NanoFloat_log.csv\"");
          client.println("Connection: close");
          client.println();
          while (file.available()) {
            client.write(file.read());
          }
          file.close();
          Serial.println("Log file sent to client");
        }
      } else {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-type:text/html");
        client.println();
        client.println("<!DOCTYPE html><html><head><title>NanoFloat</title>");
        client.println("<style>body{font-family:monospace;background:#0b1628;color:#dbeafe;padding:30px;}");
        client.println("h1{color:#2dd4bf;}a{color:#67e8f9;font-size:1.2em;}</style></head><body>");
        client.println("<h1>NanoFloat 2.0 — Data Download</h1>");
        client.println("<a href='/download'>Download Mission Log (CSV)</a>");
        client.println("</body></html>");
      }
      client.stop();
      Serial.println("Client disconnected");
    }
  }
  Serial.println("WiFi finished");
}

//================================================================================================================================================
//                                                        Initialize RFM9x Radio

void initialize_radio() {
  digitalWrite(PIN_RF95_RST, HIGH);
  Serial.println("\nInitializing NanoFloat Radio...");

  digitalWrite(PIN_RF95_RST, LOW);
  delay(10);
  digitalWrite(PIN_RF95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("NanoFloat radio init failed");
    radio_available = false;
    return;
  }
  Serial.println("NanoFloat radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    radio_available = false;
    return;
  }

  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);
  radio_available = true;
  Serial.println("NanoFloat radio initialized!");
}

//================================================================================================================================================
//                                                        Setup

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("|| NanoFloat — LittleFS / Radio / WiFi Demo ||");

  // I2C — explicit pins required on XIAO ESP32-C6
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // Radio reset pin
  pinMode(PIN_RF95_RST, OUTPUT);

  // MS5837 pressure sensor 
  Serial.println("\nInitializing MS5837 pressure sensor...");
  int attempts = 0;
  while (!pressureSensor.init() && attempts < 10) {
    Serial.println("Pressure sensor init failed! Retrying...");
    delay(2000);
    attempts++;
  }
  if (attempts >= 10) {
    Serial.println("ERROR: Could not initialize pressure sensor!");
    while (1) { delay(1000); }
  }
  pressureSensor.setModel(MS5837::MS5837_30BA);
  pressureSensor.setFluidDensity(997);   // freshwater; use 1029 for seawater
  Serial.println("Pressure sensor initialized!");

  // LittleFS 
  if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
    Serial.println("LittleFS Mount Failed");
    while (1) { delay(1000); }
  } else {
    Serial.println("Little FS Mounted Successfully");
  }

  // Match main sketch behaviour: create header only if file absent
  bool fileexists = LittleFS.exists(LOG_FILE);
  if (!fileexists) {
    Serial.println("Log file doesn't exist, creating...");
    writeFile(LittleFS, LOG_FILE, "Company, Timestamp, Depth (m), Pressure (kPA)\r\n");
  } else {
    Serial.println("Log file already exists, appending");
  }

  // NTP time sync 
  sync_time();

  // WiFi soft-AP 
  Serial.println("Configuring access point...");
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  server.begin();
  Serial.println("AP configured successfully!");

  // Radio
  initialize_radio();

  Serial.println("\n|| SYSTEM READY ||");
  Serial.println("Phase 1 — LOGGING starts (3 minutes)");
  Serial.println("Phase 2 — RADIO TX begins automatically at 3:00");
  Serial.println("Phase 3 — WiFi TX begins automatically after radio\n");
}

//================================================================================================================================================
//                                                             Main Loop 

enum Phase { PHASE_LOGGING, PHASE_RADIO, PHASE_WIFI, PHASE_DONE };
Phase phase = PHASE_LOGGING;
unsigned long phaseStart = 0;
unsigned long lastLog = 0;

void loop() {

  // Phase 1: log sensor data every 10 s for 3 minutes 
  if (phase == PHASE_LOGGING) {
    if (phaseStart == 0) phaseStart = millis();

    if (millis() - lastLog >= LOG_INTERVAL_MS) {
      float depth, pressure;
      read_sensor(depth, pressure);
      save_data(depth, pressure);
      lastLog = millis();

      unsigned long remaining = (LOGGING_DURATION_MS - (millis() - phaseStart)) / 1000;
      Serial.print("[DEMO] ~");
      Serial.print(remaining);
      Serial.println(" s until radio TX");
    }

    if (millis() - phaseStart >= LOGGING_DURATION_MS) {
      Serial.println("\n[DEMO] Logging complete — starting radio TX\n");
      phase = PHASE_RADIO;
    }
  }

  // Phase 2: transmit full log via radio 
  else if (phase == PHASE_RADIO) {
    radiotransmit_data();
    Serial.println("\n[DEMO] Radio TX done — starting WiFi server\n");
    phase = PHASE_WIFI;
  }

  // Phase 3: serve log over WiFi for 5 minutes
  else if (phase == PHASE_WIFI) {
    wifitransmit_data();
    phase = PHASE_DONE;
  }

  else {
    static bool printed = false;
    if (!printed) {
      Serial.println("\n|| LITTLEFS DEMO COMPLETE ||");
      Serial.println("All three subsystems tested:");
      Serial.println("LittleFS -> DONE");
      Serial.println("Radio TX -> DONE");
      Serial.println("WiFi TX  -> DONE");
      printed = true;
    }
    delay(5000);
  }
}
