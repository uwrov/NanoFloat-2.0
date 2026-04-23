#include <SPI.h>
#include <RH_RF95.h>

// Define RFM95 frequency (REMEMBER: MUST MATCH TRANSMITTER)
#define RF95_FREQ 915.0

//================================================================================================================================================
//                                                              Pin Definitions

const int PIN_RF95_CS  = 2;   // RFM95 Chip Select
const int PIN_RF95_INT = 21;    // RFM95 Interrupt
const int PIN_RF95_RST = 16;    // RFM95 Reset

//================================================================================================================================================
//                                                              Global Variables

// RFM95 Radio
RH_RF95 rf95(PIN_RF95_CS, PIN_RF95_INT);
bool radioAvailable = false;

int16_t packetnum = 0;  // For counting received packets

//================================================================================================================================================
//                                                              Function Prototypes

void initialize_radio();
void receive_packet();

//================================================================================================================================================
//                                                              Setup Function

void setup() {
  // Initialize Serial
  Serial.begin(9600);
  delay(1000);
  Serial.println("\n\n╔════════════════════════════════════════════╗");
  Serial.println("║       NanoFloat Receiver — Radio Mode      ║");
  Serial.println("╚════════════════════════════════════════════╝");

  // Configure GPIO pins
  pinMode(PIN_RF95_RST, OUTPUT);
  
  // Initialize radio receiver
  initialize_radio();

  Serial.println("\n|| RECEIVER READY — Listening for packets... ||\n");
}

//================================================================================================================================================
//                                                              Main Loop

void loop() {
  receive_packet();
  delay(10);
}

// //================================================================================================================================================
// //                                                              RFM95 Radio Functions

void initialize_radio() {
  pinMode(PIN_RF95_RST, OUTPUT);
  digitalWrite(PIN_RF95_RST, HIGH);

  Serial.println("\nInitializing Receiver Radio...");

  // Manual reset
  digitalWrite(PIN_RF95_RST, LOW);
  delay(10);
  digitalWrite(PIN_RF95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("Receiver radio init failed! Retrying...");
    delay(2000);
  }
  Serial.println("Receiver radio init OK!");

  // Set frequency — must match transmitter exactly
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("ERROR: setFrequency failed!");
    while (1) { 
      delay(1000); 
    }
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);

  // Receiver does not need high TX power, but set to match link budget
  rf95.setTxPower(23, false);

  radioAvailable = true;
  Serial.println("Receiver radio ready!");
}

void receive_packet() {
  if (!radioAvailable) {
    return;
  } 

  // Check if a packet is available
  if (!rf95.available()) {
    return;
  } 

  // Buffer sized to match transmitter packetSize[100]
  uint8_t buf[100];
  uint8_t len = sizeof(buf);

  if (rf95.recv(buf, &len)) {
    packetnum++;

    // Terminate the packet string being printed to avoid any abnormal characters
    buf[len] = '\0'; 

    // Print raw packet contents
    Serial.println("-------");
    Serial.print("Packet received [");
    Serial.print(len);
    Serial.println(" bytes]");
    Serial.print("Data:   ");
    Serial.println((char *)buf);
  }
}
