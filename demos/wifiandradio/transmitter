#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <Wire.h>
#include "MS5837.h"
#include <SPI.h>
#include <RH_RF95.h>
#include "config.h"

// Define RFM95 frequency
#define RF95_FREQ 915.0

//================================================================================================================================================
//                                                              Pin Definitions (XIAO ESP32-C6)

// Direct GPIO pin assignments

const int PIN_RF95_CS  = D2;   // RFM95 CS
const int PIN_RF95_INT = D3;   //RFM95 Interrupt
const int PIN_RF95_RST = D6;   //RFM95 Reset
const int PIN_RF95_SCK  = D8;   // RFM95 Clock
const int PIN_RF95_MISO = D9;   // RFM95 Serial Out
const int PIN_RF95_MOSI = D10;  // RFM95 Serial In

//================================================================================================================================================
//                                                              Global Variables

// Safety limits
const float MAX_DEPTH = 30.0;

// Pressure sensor object (direct I2C)
MS5837 pressureSensor;

// Wifi web server
WiFiServer server(80);

// RFM95 Radio
RH_RF95 rf95(PIN_RF95_CS, PIN_RF95_INT);
bool radioAvailable = false;

int16_t packetnum = 0;  // For counting packets

// Transmission interval (ms)
const unsigned long TX_INTERVAL = 2000;

// Demo mode state machine
enum DemoMode { MODE_MENU, MODE_WIFI, MODE_RADIO };
DemoMode currentMode = MODE_MENU;

//================================================================================================================================================
//                                                              Function Prototypes

float read_depth();
void initialize_radio();
void initialize_sensor();
void initialize_wifi();
void transmitRadioData();
void handle_wifi_client();
void print_menu();

//================================================================================================================================================
//                                                              Setup Function

void setup() {
  // Initialize Serial
  Serial.begin(9600);
  delay(1000);
  Serial.println("\n\n╔════════════════════════════════════════════╗");
  Serial.println("║     NanoFloat Demo - Transmission Mode     ║");
  Serial.println("╚════════════════════════════════════════════╝");

  // Configure direct GPIO pins
  pinMode(PIN_RF95_RST, OUTPUT);

  // Initialize I2C bus
  Wire.begin();

  // Initialize pressure sensor
  initialize_sensor();

  // Initialize WiFi AP and web server
  initialize_wifi();

  // Initialize radio transmitter
  initialize_radio();

  print_menu();
}

//================================================================================================================================================
//                                                              Main Loop

void loop() {
  // Check for mode-change input at any time
  if (Serial.available() > 0) {
    char input = tolower((char)Serial.read());

    if (currentMode == MODE_MENU) {
      if (input == 'w') {
        Serial.println("\nStarting WIFI mode - open http://192.168.4.1 in a browser");
        Serial.println("Press 'X' to stop.\n");
        currentMode = MODE_WIFI;
      } else if (input == 'r') {
        Serial.println("\nStarting RADIO mode - transmitting LoRa packets");
        Serial.println("Press 'X' to stop.\n");
        currentMode = MODE_RADIO;
      }
    } else {
      // In an active mode — only 'X' exits back to menu
      if (input == 'x') {
        Serial.println("\nTransmission stopped.");
        currentMode = MODE_MENU;
        print_menu();
      }
    }
  }

  // Run the active mode
  switch (currentMode) {
    case MODE_WIFI:  handle_wifi_client(); break;
    case MODE_RADIO: transmitRadioData();  break;
    default: break;
  }

  delay(10);
}

// //================================================================================================================================================
// //                                                              RFM95 Radio Functions

void initialize_radio() {
  pinMode(PIN_RF95_RST, OUTPUT);
  digitalWrite(PIN_RF95_RST, HIGH);

  Serial.println("\nInitializing Nanofloat Radio...");

  // Manual reset
  digitalWrite(PIN_RF95_RST, LOW);
  delay(10);
  digitalWrite(PIN_RF95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("Nanofloat radio init failed");
    radioAvailable = false;
    return;
  }
  Serial.println("Nanofloat radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("ERROR: setFrequency failed!");
    while (1) { delay(1000); }
  }

  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);
  radioAvailable = true;
  Serial.println("Nanofloat radio ready!");
}

void transmitRadioData() {
  // Only transmit on TX_INTERVAL cadence
  static unsigned long lastTx = 0;
  if (millis() - lastTx < TX_INTERVAL) return;
  lastTx = millis();

  if (!radioAvailable) {
    Serial.println("Nanofloat radio not available");
    return;
  }

  pressureSensor.read();

  Serial.println("Sending to rf95_server");

  char packetSize[100];
  String radioPacket = "Depth: " + String(read_depth(), 2) +
                       "m | Temp: " + String(pressureSensor.temperature(), 1) +
                       " C | Pressure: " + String(pressureSensor.pressure(), 2) +
                       " mbar | #" + String(packetnum++);

  radioPacket.toCharArray(packetSize, sizeof(packetSize));

  Serial.print("Sending ");
  Serial.println(packetSize);
  Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)packetSize, strlen(packetSize));
  Serial.println("Waiting for packet to complete...");
  delay(10);
  rf95.waitPacketSent();
}

// //================================================================================================================================================
// //                                                              WiFi Web Server

void initialize_wifi() {
  Serial.println();
  Serial.println("Configuring access point...");

  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.begin();
  Serial.println("Web server started");
  Serial.println("Open browser to: http://192.168.4.1");

  Serial.println("\nAP configured successfully!");
}

void handle_wifi_client() {
  WiFiClient client = server.available();
  if (!client) return;

  Serial.println("New web client connected.");
  String currentLine = "";

  // Wait briefly for client to send request
  unsigned long timeout = millis();
  while (!client.available() && millis() - timeout < 2000) delay(1);

  while (client.connected()) {
    if (client.available()) {
      char c = client.read();
      if (c == '\n') {
        if (currentLine.length() == 0) {
          // Read fresh sensor data
          pressureSensor.read();
          float depth    = read_depth();
          float temp = pressureSensor.temperature();
          float pressure = pressureSensor.pressure();

          Serial.print("Depth: ");
          Serial.print(String(depth, 2));
          Serial.print(" m | Temp: ");
          Serial.print(String(temp, 1));
          Serial.print(" C | Pressure: ");
          Serial.print(String(pressure, 2));
          Serial.println(" mbar");

          // Send HTTP response
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();

          // HTML page with auto-refresh every 2 seconds
          client.println("<!DOCTYPE html><html>");
          client.println("<head>");
          client.println("<meta http-equiv='refresh' content='2'>");
          client.println("<title>NanoFloat Monitor</title>");
          client.println("<style>");
          client.println("body { font-family: monospace; background: #0b1628; color: #dbeafe; padding: 30px; }");
          client.println("h1 { color: #2dd4bf; } .card { background: #1e3a5f; padding: 15px; margin: 10px 0; border-radius: 8px; }");
          client.println(".val { font-size: 1.5em; color: #67e8f9; font-weight: bold; }");
          client.println("</style></head><body>");
          client.println("<h1>NanoFloat 2.0 Live Monitor</h1>");

          client.println("<div class='card'><p>Depth</p><p class='val'>" + String(depth, 2) + " m</p></div>");
          client.println("<div class='card'><p>Temperature</p><p class='val'>" + String(temp, 1) + " celcius</p></div>");
          client.println("<div class='card'><p>Pressure</p><p class='val'>" + String(pressure*0.1, 2) + " kpa</p></div>");

          client.println("</body></html>");
          client.println();
          break;
        } else {
          currentLine = "";
        }
      } else if (c != '\r') {
        currentLine += c;
      }
    }
  }

  client.stop();
  Serial.println("Web client disconnected.");
}

// //================================================================================================================================================
// //                                                              Pressure Sensor

void initialize_sensor() {
  Serial.println("\nInitializing MS5837 pressure sensor...");
  int attempts = 0;
  while (!pressureSensor.init() && attempts < 10) {
    Serial.println("Pressure sensor init failed! Retrying...");
    delay(2000);
    attempts++;
  }

  if (attempts >= 10) {
    Serial.println("ERROR: Could not initialize pressure sensor!");
    Serial.println("Check connections: White=SDA, Green=SCL");
    while(1) { delay(1000); }
  }

  pressureSensor.setModel(MS5837::MS5837_30BA);  // Bar30 explicit model set
  pressureSensor.setFluidDensity(997);            // Freshwater (use 1029 for seawater)
  Serial.println("Pressure sensor initialized!");
}

// //================================================================================================================================================
// //                                                              Depth Reading

float read_depth() {
  float depth = pressureSensor.depth();

  if (depth < 0) {
    depth = 0;
  }

  if (depth > MAX_DEPTH) {
    depth = MAX_DEPTH;
  }
  return depth;
}

// //================================================================================================================================================
// //                                                              Demo Menu

void print_menu() {
  Serial.println("\n|| SYSTEM READY ||");
  Serial.println("\nSelect transmission mode:");
  Serial.println("  W  ->  WiFi  (live web dashboard)");
  Serial.println("  R  ->  Radio (LoRa packet TX)");
  Serial.println("  X -> Exit");
}

