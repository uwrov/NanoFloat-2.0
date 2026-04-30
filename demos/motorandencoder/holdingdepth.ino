#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <MS5837.h>
#include <EEPROM.h>
#include "FS.h"
#include <LittleFS.h>
#include "time.h"
#include "config.h"

// Pins
const int PIN_ENCODER_A  = D0;
const int PIN_ENCODER_B  = D1;
const int PIN_LIMIT_SW   = D7;
const int MCP_ADDR       = 0x20;
const int PIN_MOTOR_1    = 1;
const int PIN_MOTOR_2    = 0;

// Globals
volatile int encoder_delta = 0;
int piston_position        = 0;
float current_depth        = 0.0;
float sensordepth_tolerance = 0.2;
const float MAX_DEPTH      = 30.0;
const float MIN_DEPTH      = 0.0;
const unsigned long MAX_MOTOR_TIME = 120000;
const String COMPANY_NUMBER = "PLACEHOLDER";
const char* LOG_FILE       = "/NanoFloat_datalog.csv";
const int EEPROM_SIZE      = 512;
const int EEPROM_POSITION_ADDR = 0;

bool radio_available = false;

#define FORMAT_LITTLEFS_IF_FAILED true

Adafruit_MCP23X17 mcp;
MS5837 pressureSensor;

// Function Prototypes
void IRAM_ATTR encoder_isr();
void piston_out();
void piston_in();
void piston_stop();
void read_sensor(float &depth, float &pressure);
void save_data(float depth, float pressure);
void save_position();
void position_reset();
long depth_to_encoder(float depth_m);
bool move_to_depth(float target_depth_m);
bool hold_depth(float target_depth_m, unsigned long duration_ms, int readings);
bool surface();
void writeFile(fs::FS &fs, const char* path, const char* message);
void appendFile(fs::FS &fs, const char* path, const char* message);
bool piston_move(int steps, bool absolute);

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("|| NanoFloat Hold Depth Test ||");

  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_LIMIT_SW,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoder_isr, RISING);

  Wire.begin();

  if (!mcp.begin_I2C(MCP_ADDR)) {
    Serial.println("ERROR: MCP23017 not found.");
    while(1) { delay(1000); }
  }
  mcp.pinMode(PIN_MOTOR_1, OUTPUT);
  mcp.pinMode(PIN_MOTOR_2, OUTPUT);
  piston_stop();
  Serial.println("MCP23017 initialized!");

  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_POSITION_ADDR, piston_position);
  if (piston_position < 0) piston_position = 0;

  Serial.println("\nInitializing MS5837...");
  int attempts = 0;
  while (!pressureSensor.init() && attempts < 10) {
    Serial.println("Pressure sensor init failed! Retrying...");
    delay(2000);
    attempts++;
  }
  if (attempts >= 10) {
    Serial.println("ERROR: Could not initialize pressure sensor!");
    while(1) { delay(1000); }
  }
  pressureSensor.setModel(MS5837::MS5837_30BA);
  pressureSensor.setFluidDensity(997);
  Serial.println("Pressure sensor initialized!");

  if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
    Serial.println("LittleFS Mount Failed");
    while(1) { delay(1000); }
  } else {
    Serial.println("LittleFS Mounted Successfully");
  }

  bool fileexists = LittleFS.exists(LOG_FILE);
  if (!fileexists) {
    writeFile(LittleFS, LOG_FILE, "Company, Timestamp, Depth (m), Pressure (kPa)\r\n");
  }

  // Prompt for target depth
  Serial.println("\nEnter target depth in meters (e.g. 2.5):");
  while (!Serial.available()) { delay(50); }
  float target_depth = Serial.parseFloat();
  Serial.read(); // clear newline
  Serial.print("Target depth: ");
  Serial.print(target_depth, 2);
  Serial.println(" m");

  // Prompt for hold duration
  Serial.println("\nEnter hold duration in seconds (e.g. 30):");
  while (!Serial.available()) { delay(50); }
  unsigned long duration_s = Serial.parseInt();
  Serial.read();
  unsigned long duration_ms = duration_s * 1000;
  Serial.print("Hold duration: ");
  Serial.print(duration_s);
  Serial.println(" seconds");

  // Prompt for number of readings
  Serial.println("\nEnter number of readings (e.g. 7):");
  while (!Serial.available()) { delay(50); }
  int readings = Serial.parseInt();
  Serial.read();
  Serial.print("Readings: ");
  Serial.println(readings);

  Serial.println("\nPress any key to begin...");
  while (!Serial.available()) { delay(50); }
  Serial.read();

  // Dive to target depth
  Serial.print("\nDiving to ");
  Serial.print(target_depth, 2);
  Serial.println(" m...");
  if (!move_to_depth(target_depth)) {
    Serial.println("ERROR: Failed to reach target depth!");
    return;
  }

  // Hold at target depth
  Serial.println("\nStarting hold...");
  if (!hold_depth(target_depth, duration_ms, readings)) {
    Serial.println("ERROR: Hold failed!");
  } else {
    Serial.println("Hold complete!");
  }

  // Surface
  Serial.println("\nSurfacing...");
  surface();

  Serial.println("\n|| Test complete. ||");
  Serial.println("Check LittleFS log for saved readings.");
}

void loop() {}

//================================================================================================================================================
void IRAM_ATTR encoder_isr() {
  if (digitalRead(PIN_ENCODER_A) > digitalRead(PIN_ENCODER_B)) {
    encoder_delta++;
  } else {
    encoder_delta--;
  }
}

void piston_out() {
  mcp.digitalWrite(PIN_MOTOR_1, HIGH);
  mcp.digitalWrite(PIN_MOTOR_2, LOW);
}

void piston_in() {
  mcp.digitalWrite(PIN_MOTOR_1, LOW);
  mcp.digitalWrite(PIN_MOTOR_2, HIGH);
}

void piston_stop() {
  mcp.digitalWrite(PIN_MOTOR_1, LOW);
  mcp.digitalWrite(PIN_MOTOR_2, LOW);
}

void read_sensor(float &depth, float &pressure) {
  pressureSensor.read();
  depth    = pressureSensor.depth();
  pressure = pressureSensor.pressure() * 0.1;
  if (depth < 0)         depth = 0;
  if (depth > MAX_DEPTH) depth = MAX_DEPTH;
}

void save_position() {
  EEPROM.put(EEPROM_POSITION_ADDR, piston_position);
  EEPROM.commit();
}

void position_reset() {
  piston_position = 0;
  save_position();
  Serial.println("Piston position reset to 0.");
}

void writeFile(fs::FS &fs, const char* path, const char* message) {
  File file = fs.open(path, FILE_WRITE);
  if (!file) { Serial.println("- failed to open file for writing"); return; }
  if (file.print(message)) { Serial.println("- file written"); }
  else { Serial.println("- write failed"); }
  file.close();
}

void appendFile(fs::FS &fs, const char* path, const char* message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) { Serial.println("- failed to open file for appending"); return; }
  if (file.print(message)) { Serial.println("- message appended"); }
  else { Serial.println("- append failed"); }
  file.close();
}

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
  String line = COMPANY_NUMBER + ", " + timestamp + ", " + String(depth, 2) + ", " + String(pressure, 2) + "\r\n";
  appendFile(LittleFS, LOG_FILE, line.c_str());
  Serial.print("Logged | Depth: "); Serial.print(depth, 2);
  Serial.print(" m | Pressure: "); Serial.print(pressure, 2); Serial.println(" kPa");
}

long depth_to_encoder(float depth_m) {
  if (depth_m <= 0.4f) return ENCODER_COUNT_0_4M;
  if (depth_m <= 2.5f) return ENCODER_COUNT_2_5M;
  return ENCODER_COUNT_2_5M;
}

bool piston_move(int steps, bool absolute) {
  int target_position = absolute ? steps : piston_position + steps;
  encoder_delta = 0;
  unsigned long start_time = millis();

  if (target_position > piston_position) {
    piston_out();
    while (piston_position < target_position) {
      if (millis() - start_time > MAX_MOTOR_TIME) { piston_stop(); return false; }
      if (digitalRead(PIN_LIMIT_SW) == HIGH) { piston_stop(); position_reset(); return false; }
      noInterrupts(); piston_position += encoder_delta; encoder_delta = 0; interrupts();
      delay(10);
    }
  } else if (target_position < piston_position) {
    piston_in();
    while (piston_position > target_position) {
      if (millis() - start_time > MAX_MOTOR_TIME) { piston_stop(); return false; }
      if (digitalRead(PIN_LIMIT_SW) == HIGH) { piston_stop(); position_reset(); return false; }
      noInterrupts(); piston_position += encoder_delta; encoder_delta = 0; interrupts();
      delay(10);
    }
  }

  piston_stop();
  save_position();
  return true;
}

bool move_to_depth(float target_depth_m) {
  if (target_depth_m < MIN_DEPTH || target_depth_m > MAX_DEPTH) return false;

  long target_count = depth_to_encoder(target_depth_m);
  unsigned long start_time = millis();
  float depth, pressure;

  while (true) {
    noInterrupts(); piston_position += encoder_delta; encoder_delta = 0; interrupts();
    read_sensor(depth, pressure);
    current_depth = depth;

    if (digitalRead(PIN_LIMIT_SW) == HIGH) { piston_stop(); save_position(); return false; }
    if (millis() - start_time > MAX_MOTOR_TIME) { piston_stop(); save_position(); return false; }
    if (abs(depth - target_depth_m) <= sensordepth_tolerance) { piston_stop(); save_position(); return true; }
    if (abs(piston_position - target_count) <= ENCODER_TOLERANCE) { piston_stop(); save_position(); return true; }

    if (piston_position < target_count) { piston_out(); } else { piston_in(); }
  }
}

bool hold_depth(float target_depth_m, unsigned long duration_ms, int readings) {
  Serial.print("Holding at "); Serial.print(target_depth_m, 2);
  Serial.print(" m for "); Serial.print(duration_ms / 1000); Serial.println(" seconds...");

  unsigned long start_time    = millis();
  unsigned long read_interval = duration_ms / readings;
  unsigned long last_read_time = 0;
  int read_count              = 0;
  float depth, pressure;

  while (millis() - start_time < duration_ms) {
    noInterrupts(); piston_position += encoder_delta; encoder_delta = 0; interrupts();
    read_sensor(depth, pressure);
    current_depth = depth;

    if (digitalRead(PIN_LIMIT_SW) == HIGH) { piston_stop(); save_position(); return false; }

    if (depth < (target_depth_m - sensordepth_tolerance)) {
      piston_move(ENCODER_CORRECTION_STEP, false);
    } else if (depth > (target_depth_m + sensordepth_tolerance)) {
      piston_move(-ENCODER_CORRECTION_STEP, false);
    }

    if (millis() - last_read_time >= read_interval && read_count < readings) {
      save_data(depth, pressure);
      read_count++;
      last_read_time = millis();
    }

    Serial.print("Holding... depth: "); Serial.print(depth, 3);
    Serial.print(" m | readings: "); Serial.print(read_count);
    Serial.print("/"); Serial.println(readings);

    delay(500);
  }

  piston_stop();
  Serial.println("Hold complete.");
  save_position();
  return true;
}

bool surface() {
  Serial.println("Surfacing...");
  return move_to_depth(0.0);
}
