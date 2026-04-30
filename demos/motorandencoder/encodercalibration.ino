#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <MS5837.h>
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
int piston_position = 0;
Adafruit_MCP23X17 mcp;
MS5837 pressureSensor;

// Function Prototypes
void IRAM_ATTR encoder_isr();
void piston_out();
void piston_in();
void piston_stop();
void read_sensor(float &depth, float &pressure);
bool run_step(bool extend, float &depth, float &pressure);
void encoder_test();

void setup() {
  Serial.begin(9600);
  delay(1000);

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

  encoder_test();
}

void loop() {}

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
  if (depth < 0)    depth = 0;
  if (depth > 30.0) depth = 30.0;
}

bool run_step(bool extend, float &depth, float &pressure) {
  if (extend) {
    piston_out();
    Serial.println("Extending one step...");
  } else {
    piston_in();
    Serial.println("Retracting one step...");
  }

  int step_start = piston_position;

  while (abs(piston_position - step_start) < ENCODER_TEST_STEP) {
    noInterrupts();
    piston_position += encoder_delta;
    encoder_delta = 0;
    interrupts();

    if (digitalRead(PIN_LIMIT_SW) == HIGH) {
      piston_stop();
      Serial.println("Limit switch triggered.");
      return false;
    }
  }

  piston_stop();

  Serial.print("Stabilizing");
  float reading_a, reading_b, p;

  do {
    read_sensor(reading_a, p);
    delay(400);
    read_sensor(reading_b, p);
    delay(400);
    Serial.print(".");
  } while (abs(reading_a - reading_b) > 0.01f);

  depth    = reading_b;
  pressure = p;
  Serial.println();

  noInterrupts();
  piston_position += encoder_delta;
  encoder_delta = 0;
  interrupts();

  Serial.print("Encoder count: "); Serial.println(piston_position);
  Serial.print("Depth: ");         Serial.print(depth, 3);    Serial.println(" m");
  Serial.print("Pressure: ");      Serial.print(pressure, 2); Serial.println(" kPa");

  return true;
}

void encoder_test() {
  encoder_delta    = 0;
  piston_position  = 0;

  Serial.println("\n|--------------------------------------------|");
  Serial.println(  "|         ENCODER CALIBRATION TEST           |");
  Serial.println(  "|--------------------------------------------|");
  Serial.println("\nPlace the float at the surface before starting.");
  Serial.println("Press any key to zero the encoder and begin...");
  while (!Serial.available()) { delay(50); }
  Serial.read();

  noInterrupts();
  encoder_delta = 0;
  interrupts();
  piston_position = 0;

  float depth, pressure;
  read_sensor(depth, pressure);

  Serial.println("\nEncoder zeroed at surface.");
  Serial.print("Surface depth reading: ");
  Serial.print(depth, 3);
  Serial.println(" m\n");
  Serial.println("Controls:");
  Serial.println("  Any key -> extend one step");
  Serial.println("  'r'     -> retract one step");
  Serial.println("  'x'     -> abort test");

  int count_0_4m = -1;
  int count_2_5m = -1;

  Serial.println("\n|--------------------------------------------|");
  Serial.println(  "|    PHASE 1: Extend piston to reach 0.4 m   |");
  Serial.println(  "|--------------------------------------------|");

  while (count_0_4m < 0) {
    Serial.println("Press any key to extend, 'r' to retract, 'x' to abort.");
    while (!Serial.available()) { delay(50); }
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "x") { piston_stop(); Serial.println("Test aborted."); return; }

    bool ok = run_step(cmd != "r", depth, pressure);
    if (!ok) return;

    if (depth >= (0.4f - ENCODER_TOLERANCE) && depth <= (0.4f + ENCODER_TOLERANCE)) {
      count_0_4m = piston_position;
      Serial.println("\n0.4m TARGET CONFIRMED BY SENSOR");
      Serial.print("Encoder count: "); Serial.println(count_0_4m);
      Serial.print("Sensor depth: ");  Serial.print(depth, 3); Serial.println(" m");
      Serial.print("Pressure: ");      Serial.print(pressure, 2); Serial.println(" kPa");
      Serial.println("Proceeding to 2.5m.\n");
    }
  }

  Serial.println("\n|--------------------------------------------|");
  Serial.println(  "|    PHASE 2: Extend piston to reach 2.5 m   |");
  Serial.println(  "|--------------------------------------------|");

  while (count_2_5m < 0) {
    Serial.println("Press any key to extend, 'r' to retract, 'x' to abort.");
    while (!Serial.available()) { delay(50); }
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "x") { piston_stop(); Serial.println("Test aborted."); return; }

    bool ok = run_step(cmd != "r", depth, pressure);
    if (!ok) return;

    if (depth >= (2.5f - ENCODER_TOLERANCE) && depth <= (2.5f + ENCODER_TOLERANCE)) {
      count_2_5m = piston_position;
      Serial.println("\n2.5m TARGET CONFIRMED BY SENSOR");
      Serial.print("Encoder count: "); Serial.println(count_2_5m);
      Serial.print("Sensor depth: ");  Serial.print(depth, 3); Serial.println(" m");
      Serial.print("Pressure: ");      Serial.print(pressure, 2); Serial.println(" kPa");
      Serial.println("Encoders for 2.5m locked.\n");
    }
  }

    Serial.println("\n|--------------------------------------------|");
    Serial.println(  "|          CALIBRATION SUMMARY               |");
    Serial.println(  "|--------------------------------------------|");
    Serial.println("\nCopy these values into config.h:\n");

    Serial.print("  #define ENCODER_COUNT_0_4M    ");
    if (count_0_4m >= 0) {
        Serial.println(count_0_4m);
    } else {
        Serial.println("NOT REACHED");
    }

    Serial.print("  #define ENCODER_COUNT_2_5M    ");
    if (count_2_5m >= 0) {
        Serial.println(count_2_5m);
    } else {
        Serial.println("NOT REACHED");
    }

    Serial.println("\nEncoder calibration complete.");
}
