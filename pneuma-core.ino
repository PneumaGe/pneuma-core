// Copyright 2026 Shereef Sayed
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ==========================================================================
// PneumaGe: Double-Walled Static Accumulation Chamber Controller
//
// Firmware Version: 1.2.0 (Simulation Enabled)
// Project Lead: Shereef Sayed
//
// Nano 33 BLE Sense Pinout:
// - D1: PWM to BoxerPump 22KD
// - A2: Current Sensor (Shunt)
// - I2C: SprintIR-W, BMP390, SHT45
// - SPI: BMI270 IMU
// ==========================================================================

#include <ArduinoBLE.h>
// #include <Arduino_LSM9DS1.h>
#include <Arduino_BMI270_BMM150.h>
#include <ArduinoJson.h>

// -- Simulation Control --
// Set to 'true' to use simulated data for peripherals that are not yet connected.
// Set to 'false' to use real sensor libraries (once integrated).
#define SIMULATE_PERIPHERALS true

// Conditionally include real sensor libraries if not simulating
#if !SIMULATE_PERIPHERALS
  #include <Arduino_BMP390.h>  // For pressure on Rev2
  #include <Arduino_HS300x.h>  // For temp/humidity on Rev2
  // #include "SprintIR_W.h"    // Placeholder for actual library
#endif

// ==========================================================================
// BLE Service & Characteristics (UUIDs from GEMINI.md)
// =================================G=========================================
BLEService pneumaService("19B10000-E8F2-537E-4F6C-D104768A1214");

// UUID: 0001 - Real-time gas concentration
BLEFloatCharacteristic gasConcentrationChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
// UUID: 0002 - Packed environmental data
BLECharacteristic chamberStatsChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 18);
// UUID: 0003 - Command input from the app
BLECharacteristic systemCommandChar("19B10003-E8F2-537E-4F6C-D104768A1214", BLEWriteWithoutResponse, 1);
// UUID: 0004 - RANSAC flux calculation result (Disabled for Nano build)
// BLECharacteristic ransacResultChar("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 10);
// UUID: 0005 - System status flags (e.g., tilt, bump, errors)
BLECharacteristic systemStatusChar("19B10005-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 1);
// UUID: 0006 - Battery state of charge
BLECharacteristic batteryLifeChar("19B10006-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 4);
// UUID: 0007 - Static device properties (model, serial, etc.)
BLECharacteristic deviceInfoChar("19B10007-E8F2-537E-4F6C-D104768A1214", BLENotify, 20); // Streamed via notifications

// ==========================================================================
// Data Structures
// ==========================================================================
struct __attribute__((packed)) ChamberStats {
  int16_t  airTemperature; // Temperature (C * 100)
  int16_t  chamberTemperature; // Temperature (C * 100)
  uint32_t airPressure;   // Pressure (hPa * 1000)
  uint32_t chamberPressure;   // Pressure (hPa * 1000)
  uint16_t airHumidity;   // Humidity (% * 100)
  uint16_t chamberHumidity;   // Humidity (% * 100)
  uint16_t status;     // Bitmask for flags
};

struct __attribute__((packed)) RansacResult {
  float slope;
  float r_squared;
  uint8_t inlier_ratio; // Percentage of inliers
  uint8_t reserved;     // Padding
};

// ==========================================================================
// Global State & Configuration
// ==========================================================================
// System State
enum SystemCommand : uint8_t {
  CMD_STOP          = 0x00,
  CMD_START         = 0x01,
  CMD_PUMP_LOW      = 0x10,
  CMD_PUMP_MED      = 0x11,
  CMD_PUMP_HIGH     = 0x12,
  CMD_TARE_SENSORS  = 0x20,
  CMD_SET_LEVEL     = 0x30,
  CMD_HEARTBEAT     = 0xAA
};

bool isMeasuring = false;
bool systemShutdown = false;
unsigned long lastHeartbeat = 0;
static unsigned long lastUpdateTime = 0;
static bool wasConnected = false; // For tracking BLE connection state changes
const unsigned long HEARTBEAT_TIMEOUT_MS = 30000; // 30s timeout for easier debugging
uint8_t systemStatusFlags = 0x00;

// CO2 Simulation
float baselineCO2 = 415.0; // Typical atmospheric CO2 in ppm
float currentCO2 = 415.0;
float fluxRate = 1.5; // ppm per second during measurement

// Pump
const int PUMP_PIN = 1;
const int PUMP_SPEED_OFF = 0;
const int PUMP_SPEED_LOW = 77;   // ~30%
const int PUMP_SPEED_MED = 153;  // ~60%
const int PUMP_SPEED_HIGH = 255; // 100%
int currentPumpSpeed = PUMP_SPEED_OFF;

// IMU & Disturbance
float base_ax, base_ay, base_az; // Baseline for "level"
const float TILT_THRESHOLD = 3.0; // Degrees
const float BUMP_THRESHOLD_G = 1.5; // G-force

// System Status Flags (for BLE characteristic 0x0005)
const uint8_t STATUS_FLAG_TILT = 0x01;
const uint8_t STATUS_FLAG_BUMP = 0x02;
const uint8_t STATUS_FLAG_TIMEOUT = 0x07;

// ==========================================================================
// Peripheral Simulation
// ==========================================================================
#if SIMULATE_PERIPHERALS

// Simulate reading from the SprintIR-W CO2 Sensor
float readGasSensor() {
  if (isMeasuring) {
    currentCO2 += fluxRate * (random(80, 120) / 100.0); // Add some noise
  } else {
    // Decay back to baseline when not measuring
    if (abs(currentCO2 - baselineCO2) > 0.5) {
      currentCO2 -= (currentCO2 - baselineCO2) / 2.0;
    }
  }
  return currentCO2;
}

// Simulate reading the BoxerPump's current
float readPumpCurrent() {
  if (currentPumpSpeed == 0) return 0.0;
  // Simulate nominal current draw with some noise
  return 200.0 + (random(-10, 10)); // 200mA nominal
}

// Simulate reading the battery State of Charge (%)
float readBatterySoC() {
  static float batteryLevel = 100.0;
  if (batteryLevel > 0) {
    // Drain faster when pump is on
    float drainRate = (currentPumpSpeed > 0) ? 0.02 : 0.005;
    batteryLevel -= drainRate;
  }
  return max(0.0, batteryLevel);
}

// Simulate environmental sensors onboard the Nano
void readOnboardSensors(ChamberStats &stats) {
  // Use stable but slightly varying values for simulation
  stats.airTemperature = (int16_t)((25.0 + sin(millis() / 10000.0)) * 100);
  stats.airPressure = (uint32_t)((1013.25 + sin(millis() / 15000.0)) * 1000);
  stats.airHumidity = (uint16_t)((45.0 + cos(millis() / 12000.0) * 5) * 100);
  stats.chamberTemperature = (int16_t)((25.0 + sin(millis() / 10000.0)) * 100);
  stats.chamberPressure = (uint32_t)((1013.25 + sin(millis() / 15000.0)) * 1000);
  stats.chamberHumidity = (uint16_t)((45.0 + cos(millis() / 12000.0) * 5) * 100);
  stats.status = 0; // Clear status bits for now
}

#else
// ==========================================================================
// Real Hardware Integration (To be implemented)
// ==========================================================================
float readGasSensor() {
  // TODO: Add code to read from SprintIR-W sensor via UART/I2C
  return 0.0; // Placeholder
}

float readPumpCurrent() {
  // TODO: Read current from shunt on pin A2
  return 0.0; // Placeholder
}

float readBatterySoC() {
  // TODO: Read from LiPo fuel gauge IC
  return 0.0; // Placeholder
}

void readOnboardSensors(ChamberStats &stats) {
  // Read from the Rev2 onboard sensors
  stats.airTemperature = (int16_t)(HS300x.readTemperature() * 100);
  stats.airPressure = (uint32_t)(BMP390.readPressure() * 10.0); // BMP390 is Pa, convert to hPa*1000
  stats.airHumidity = (uint16_t)(HS300x.readHumidity() * 100);
  // TODO: integrate chamber sensors
  stats.chamberTemperature = 0;
  stats.chamberPressure = 0;
  stats.chamberHumidity = 0;
  stats.status = 0;
}
#endif

// ==========================================================================
// Core Logic & Command Handling
// ==========================================================================
// Get the unique 64-bit device ID from the nRF52's Factory Information
// Configuration Registers (FICR). This provides a stable serial number.
String getMCUSerialNumber() {
  char serialNumber[17]; // 8 bytes as hex + null terminator
  // Format each 32-bit part of the 64-bit ID as an 8-digit hex string.
  // The '08' ensures leading zeros are included for a fixed length.
  sprintf(serialNumber, "%08lX%08lX", 
          NRF_FICR->DEVICEID[1], // High word
          NRF_FICR->DEVICEID[0]  // Low word
  );
  serialNumber[16] = '\0';
  return String(serialNumber);
} // END getSerialNumber
// --- Pump Control ---
void setPumpSpeed(int speed) {
  if (currentPumpSpeed == 0 && speed > 0) {
    analogWrite(PUMP_PIN, 255); // 100% kickstart for 150ms
    delay(150);
  }
  analogWrite(PUMP_PIN, speed);
  currentPumpSpeed = speed;
}

// --- IMU & Disturbance Detection ---
void calibrateLevel() {
  const int num_readings = 10;
  float ax_sum = 0, ay_sum = 0, az_sum = 0;

  // Poll for acceleration data to be available, with a timeout.
  // This is crucial on boot, as the IMU may need a moment to stabilize
  // after IMU.begin() is called. A direct read can cause a crash.
  unsigned long start = millis();
  while (!IMU.accelerationAvailable()) {
    if (millis() - start > 500) { // 500ms timeout
      Serial.println("!!! WARN: IMU calibration failed. No acceleration data available.");
      return;
    }
    delay(10); // Don't spin-lock, yield to other tasks.
  }

  // Average a few readings for a more stable baseline
  for (int i = 0; i < num_readings; i++) {
    IMU.readAcceleration(base_ax, base_ay, base_az);
    ax_sum += base_ax;
    ay_sum += base_ay;
    az_sum += base_az;
    delay(10);
  }
  base_ax = ax_sum / num_readings;
  base_ay = ay_sum / num_readings;
  base_az = az_sum / num_readings;

  Serial.println("IMU level baseline calibrated.");
}

void checkDisturbance() {
  if (!IMU.accelerationAvailable()) return;
  float ax, ay, az;
  bool disturbanceDetected = false;
  IMU.readAcceleration(ax, ay, az);

  // Check for bumps
  float magnitude = sqrt(ax*ax + ay*ay + az*az);
  if (magnitude > BUMP_THRESHOLD_G) {
    systemStatusFlags |= STATUS_FLAG_BUMP;
    Serial.println("!!! ALERT: Bump Detected!");
    disturbanceDetected = true;
  }

  // Check for tilt
  float angle = acos( (ax*base_ax + ay*base_ay + az*base_az) /
                      (sqrt(ax*ax+ay*ay+az*az) * sqrt(base_ax*base_ax+base_ay*base_ay+base_az*base_az)) );
  if (degrees(angle) > TILT_THRESHOLD) {
    systemStatusFlags |= STATUS_FLAG_TILT;
    Serial.println("!!! ALERT: Chamber Tilted!");
    disturbanceDetected = true;
  }

  if (disturbanceDetected) {
    systemStatusChar.writeValue(systemStatusFlags);
  }
}

// --- RANSAC Placeholder (Disabled for Nano) ---
// void runRansac() {
//   // This is a placeholder for the actual RANSAC implementation on the Teensy.
//   // The Nano build offloads RANSAC to the app.
//   RansacResult result;
//   result.slope = isMeasuring ? (fluxRate + (random(-20, 20)/100.0)) : 0.0;
//   result.r_squared = isMeasuring ? (0.95 + (random(0, 5)/100.0)) : 0.0;
//   result.inlier_ratio = isMeasuring ? 98 : 0;
//   
//   // ransacResultChar.writeValue((uint8_t*)&result, sizeof(result));
// }

// --- BLE Command Handler ---
void handleCommand(uint8_t cmd) {
  Serial.print("[BLE CMD] Received: 0x");
  Serial.println(cmd, HEX);

  switch (cmd) {
    case CMD_STOP: // Stop all systems
      isMeasuring = false;
      setPumpSpeed(PUMP_SPEED_OFF);
      break;
    case CMD_START: // Start measurement session
      isMeasuring = true;
      break;
    case CMD_PUMP_LOW:  setPumpSpeed(PUMP_SPEED_LOW);  break;
    case CMD_PUMP_MED:  setPumpSpeed(PUMP_SPEED_MED);  break;
    case CMD_PUMP_HIGH: setPumpSpeed(PUMP_SPEED_HIGH); break;
    case CMD_TARE_SENSORS: // Tare gas sensor
      baselineCO2 = readGasSensor();
      break;
    case CMD_SET_LEVEL: // Set Level / Baseline Orientation
      calibrateLevel();
      break;
    // CMD_HEARTBEAT (0xAA) is handled directly in onCommandWritten
    default:
      Serial.println(" -> Unknown command.");
      break;
  }
}

// --- Main BLE Event Handler ---
void onCommandWritten(BLEDevice central, BLECharacteristic characteristic) {
    uint8_t cmd = 0;
    characteristic.readValue(&cmd, 1);
    
    if (cmd == CMD_HEARTBEAT) { // Handle Heartbeat
        lastHeartbeat = millis();
        Serial.println("[HEARTBEAT] Reset");
        if (systemShutdown) {
            systemShutdown = false;
            systemStatusFlags = 0x00;
            systemStatusChar.writeValue(systemStatusFlags); // System OK/Recovered
            Serial.println("[HEARTBEAT] System recovered.");
        }
    } else {
        handleCommand(cmd);
    }
}

// --- Device Info Streaming ---
// This function is triggered when the app subscribes to the Device Info characteristic.
// It generates the full JSON descriptor and streams it in small chunks (notifications).
void streamDeviceInfo(BLEDevice& central) {
  // Check if the central is still connected before proceeding.
  if (!central.connected()) {
    return;
  }

  // The JSON document size needs to be large enough for the entire object.
  // Using the ArduinoJson Assistant is recommended for production.
  StaticJsonDocument<1024> doc;

  // Get unique identifiers from the MCU
  String macAddress = BLE.address();
  macAddress.toUpperCase();
  String mcuSerial = getMCUSerialNumber();

  // Top-level device info
  doc["deviceId"] = mcuSerial; // Use the unique MCU serial as the persistent device ID
  doc["deviceName"] = "PneumaGe-Sim";
  doc["descriptorVersion"] = 1;
  doc["macAddress"] = macAddress;
  doc["processorMake"] = "Arduino";
  doc["processorModel"] = "Nano 33 BLE";
  doc["processorSerial"] = mcuSerial;
  doc["firmwareVersion"] = "1.2.0";

  // Capabilities
  JsonArray capabilities = doc.createNestedArray("capabilities");
  // Based on REQUIREMENTS.md, Nano offloads RANSAC
  capabilities.add("ransac_remote");

  // Pump Info
  JsonObject pump = doc.createNestedObject("pump");
  pump["make"] = "Boxer";
  pump["model"] = "22KD";
  pump["serialNumber"] = "PUMP-SIM-001";
  pump["flowRateMin"] = 0.1; // L/min, from datasheet
  pump["flowRateMax"] = 1.0; // L/min, from datasheet

  // Sensors array
  JsonArray sensors = doc.createNestedArray("sensors");

  // Sensor 1: CO2 Sensor (Simulated SprintIR-W)
  JsonObject co2Sensor = sensors.createNestedObject();
  co2Sensor["id"] = "co2_sensor";
  co2Sensor["make"] = "GSS";
  co2Sensor["model"] = "SprintIR-W";
  co2Sensor["serialNumber"] = "CO2-SIM-001";
  co2Sensor["sensorType"] = "NDIR";
  JsonArray co2Channels = co2Sensor.createNestedArray("channels");
  JsonObject co2Channel = co2Channels.createNestedObject();
  co2Channel["id"] = "co2";
  co2Channel["name"] = "CO2";
  co2Channel["unit"] = "ppm";
  co2Channel["gasType"] = "CO2";
  co2Channel["rangeMin"] = 0;
  co2Channel["rangeMax"] = 20000;
  co2Channel["resolution"] = 1.0;
  co2Channel["sampleRate"] = 2.0; // Hz

  // Sensor 2: Onboard Environmental Sensor
  JsonObject envSensor = sensors.createNestedObject();
  envSensor["id"] = "env_sensor";
  envSensor["make"] = "Onboard";
  envSensor["model"] = "BMP390/HS3003"; // Correct for Nano BLE Sense Rev 2
  envSensor["serialNumber"] = "N/A";
  envSensor["sensorType"] = "MEMS";
  JsonArray envChannels = envSensor.createNestedArray("channels");
  
  JsonObject tempChannel = envChannels.createNestedObject();
  tempChannel["id"] = "temp_chamber";
  tempChannel["name"] = "T_Chamber";
  tempChannel["unit"] = "°C";
  tempChannel["precision"] = 0.5;

  JsonObject pressChannel = envChannels.createNestedObject();
  pressChannel["id"] = "press_chamber";
  pressChannel["name"] = "P_Chamber";
  pressChannel["unit"] = "mBar";
  pressChannel["precision"] = 0.1;

  JsonObject rhChannel = envChannels.createNestedObject();
  rhChannel["id"] = "rh_chamber";
  rhChannel["name"] = "RH_Chamber";
  rhChannel["unit"] = "%";
  rhChannel["precision"] = 3.5;

  // Serialize JSON directly to a stack-allocated buffer to avoid heap fragmentation.
  char output[1024];
  size_t length = serializeJson(doc, output);

  // Stream the JSON data in chunks. The chunk size should be less than or
  // equal to the MTU size minus 3 bytes for overhead. 20 is a safe default.
  const int chunkSize = 20;
  size_t bytesSent = 0;

  Serial.print("Streaming Device Info... Total size: ");
  Serial.println(length);

  while (bytesSent < length) {
    if (!central.connected()) {
      Serial.println("Client disconnected during stream. Aborting.");
      break;
    }

    size_t chunk = min((size_t)chunkSize, length - bytesSent);
    deviceInfoChar.writeValue((const uint8_t*)(output + bytesSent), chunk);
    bytesSent += chunk;

    // A small delay can help some BLE client implementations process packets.
    delay(15);
  }

  // After the last data packet, send a zero-length packet to signal the
  // end of the transmission. This is a crucial part of the protocol.
  if (central.connected()) {
    deviceInfoChar.writeValue((const uint8_t*)nullptr, 0);
    Serial.println("Device Info stream complete.");
  }
}

void onDeviceInfoSubscribed(BLEDevice central, BLECharacteristic characteristic) {
  if (characteristic.subscribed()) {
    streamDeviceInfo(central);
  }
}

// ==========================================================================
// Setup & Main Loop
// ==========================================================================
void setup() {
  Serial.begin(9600);
  // while (!Serial); // Wait for serial connection
  Serial.println("PneumaGe Firmware Booting...");

  pinMode(PUMP_PIN, OUTPUT);
  // pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // --- Initialize BLE ---
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }
  
  BLE.setLocalName("PneumaGe-Sim");
  BLE.setAdvertisedService(pneumaService);
  
  // Add characteristics to the service
  pneumaService.addCharacteristic(gasConcentrationChar);
  pneumaService.addCharacteristic(chamberStatsChar);
  pneumaService.addCharacteristic(systemCommandChar);
  // pneumaService.addCharacteristic(ransacResultChar);
  pneumaService.addCharacteristic(systemStatusChar);
  pneumaService.addCharacteristic(batteryLifeChar);
  pneumaService.addCharacteristic(deviceInfoChar);
  
  BLE.addService(pneumaService);
  
    
  // Set the command handler
  systemCommandChar.setEventHandler(BLEWritten, onCommandWritten);
  deviceInfoChar.setEventHandler(BLESubscribed, onDeviceInfoSubscribed);

  // --- Initialize Sensors ---
  #if !SIMULATE_PERIPHERALS

    if (!BMP390.begin()) {
      Serial.println("Failed to initialize BMP390!");
    }
    if (!HS300x.begin()) {
      Serial.println("Failed to initialize HS300x!");
    }
  #endif
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
  }
  calibrateLevel();
  
  // NOTE: Device info is populated on first BLE connection to prevent a startup crash.

  // --- Finalize Setup ---
  BLE.advertise();
  lastHeartbeat = millis();
  digitalWrite(LED_BUILTIN, HIGH); // Signal setup complete
  Serial.println("Setup complete. Advertising...");
}

void handleConnectedState() {
  unsigned long currentTime = millis();

  // On the transition from disconnected to connected
  if (!wasConnected) {
    Serial.println("Client connected.");
    lastHeartbeat = currentTime; // Reset heartbeat timer
    wasConnected = true;
  }

  // Heartbeat timeout check
  if (!systemShutdown && (currentTime - lastHeartbeat > HEARTBEAT_TIMEOUT_MS)) {
    setPumpSpeed(PUMP_SPEED_OFF);
    isMeasuring = false;
    systemShutdown = true;
    systemStatusFlags = STATUS_FLAG_TIMEOUT;
    systemStatusChar.writeValue(systemStatusFlags);
    Serial.println("[HEARTBEAT] Timeout: System auto-shutdown.");
  }

  // Main update loop (runs every 1 second)
  if (currentTime - lastUpdateTime >= 1000) {
    lastUpdateTime = currentTime;

    if (!systemShutdown) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Blink LED

      ChamberStats stats;
      readOnboardSensors(stats);
      chamberStatsChar.writeValue((uint8_t*)&stats, sizeof(stats));

      float soc = readBatterySoC();
      batteryLifeChar.writeValue((uint8_t*)&soc, sizeof(soc));

      // Check for disturbances
      checkDisturbance();
      // After checking, clear non-latching flags if needed, or handle them.
      // A good addition would be a command to clear status flags.
      // For now, flags will persist until the next connection.
    }

    if(!systemShutdown && isMeasuring) {
      // Read sensors and update characteristics
      gasConcentrationChar.writeValue(readGasSensor());
    }
  }
}

void handleDisconnectedState() {
  // On the transition from connected to disconnected, reset system state
  if (wasConnected) {
    Serial.println("Client disconnected. Resetting state.");
    isMeasuring = false;
    setPumpSpeed(PUMP_SPEED_OFF);
    systemShutdown = false;
    systemStatusFlags = 0x00;
    currentCO2 = baselineCO2; // Reset CO2 to baseline
    wasConnected = false;
  }
  digitalWrite(LED_BUILTIN, HIGH); // Solid LED when disconnected
}

void loop() {
  BLE.poll();

  if (BLE.connected()) {
    handleConnectedState();
  } else {
    handleDisconnectedState();
  }
}
