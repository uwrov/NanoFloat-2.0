#include <SPI.h>
#include <RH_RF95.h>

// Define RFM95 frequency — MUST match RF95_FREQ in nanofloat_storage_demo.ino
#define RF95_FREQ 915.0

//================================================================================================================================================
//                                                              Pin Definitions

const int PIN_RF95_CS  = 2;    // RFM95 Chip Select
const int PIN_RF95_INT = 21;   // RFM95 Interrupt
const int PIN_RF95_RST = 16;   // RFM95 Reset

//================================================================================================================================================
//                                                              Global Variables

RH_RF95 rf95(PIN_RF95_CS, PIN_RF95_INT);
bool radioAvailable = false;

// Packet tracking
int16_t packetnum = 0;

//================================================================================================================================================
//                                                              Function Prototypes

void initialize_radio();
void receive_packet();
void parse_and_print(const char* raw);
String extractField(const String& line, int fieldIndex);

//================================================================================================================================================
//                                                              Setup Function

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("NanoFloat LittleFS Reciever");

  pinMode(PIN_RF95_RST, OUTPUT);
  initialize_radio();

  Serial.println("\n|| RECEIVER READY — Waiting for transmission at 3 mins... ||\n");
  Serial.println("Packet format expected:");
  Serial.println("COMPANY, TIMESTAMP, DEPTH, PRESSURE | #N\n");
}

//================================================================================================================================================
//                                                              Main Loop

void loop() {
  receive_packet();
  delay(10);
}

//================================================================================================================================================
//                                                              Initialize Radio

void initialize_radio() {
  pinMode(PIN_RF95_RST, OUTPUT);
  digitalWrite(PIN_RF95_RST, HIGH);

  Serial.println("\nInitializing NanoFloat Radio...");

  // Manual reset — matches transmitter reset sequence
  digitalWrite(PIN_RF95_RST, LOW);
  delay(10);
  digitalWrite(PIN_RF95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("NanoFloat radio init failed! Retrying...");
    delay(2000);
  }
  Serial.println("NanoFloat radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("ERROR: setFrequency failed!");
    while (1) { delay(1000); }
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);
  radioAvailable = true;
  Serial.println("NanoFloat radio initialized!\n");
}

//================================================================================================================================================
//                                                              Receive Packet

void receive_packet() {
  if (!radioAvailable) return;
  if (!rf95.available()) return;

  uint8_t buf[130];
  uint8_t len = sizeof(buf);

  if (!rf95.recv(buf, &len)) return;

  if (len < sizeof(buf)) {
    buf[len] = '\0';          
  } else {
    buf[sizeof(buf) - 1] = '\0'; 
  }

  parse_and_print((const char*)buf);
}

//================================================================================================================================================
//                                                              Parse and Print

void parse_and_print(const char* raw) {
  String pkt = String(raw);
  pkt.trim();

  // Split sequence number off the end (" | #N")
  int16_t seqNum  = -1;
  String  csvPart = pkt;

  int seqSep = pkt.lastIndexOf(" | #");
  if (seqSep >= 0) {
    seqNum  = (int16_t) pkt.substring(seqSep + 4).toInt();
    csvPart = pkt.substring(0, seqSep);
  }

  // Parse the four CSV fields from save_data()
  // Order: Company, Timestamp, Depth (m), Pressure (kPA)
  String company   = extractField(csvPart, 0);
  String timestamp = extractField(csvPart, 1);
  String depthStr  = extractField(csvPart, 2);
  String pressStr  = extractField(csvPart, 3);

  float depth = depthStr.toFloat();
  float pressure = pressStr.toFloat();

  // Print everything on one line
  // Format: [#N] Company | Timestamp | Depth m | Pressure kPA
  Serial.print("[#"); Serial.print(seqNum >= 0 ? seqNum : packetnum); Serial.print("] ");
  Serial.print(company);     
  Serial.print(" | ");
  Serial.print(timestamp);   
  Serial.print(" | ");
  Serial.print(depth, 2);    
  Serial.print(" m | ");
  Serial.print(pressure, 2); 
  Serial.println(" kPA");

  packetnum++;
}

//================================================================================================================================================
//                                                              Extract CSV Field

String extractField(const String& line, int fieldIndex) {
  int count = 0;
  int start = 0;

  for (int i = 0; i <= (int)line.length(); i++) {
    if (i == (int)line.length() || line.charAt(i) == ',') {
      if (count == fieldIndex) {
        String field = line.substring(start, i);
        field.trim();
        return field;
      }
      count++;
      start = i + 1;
    }
  }
  return "";  // If field index out of range
}

