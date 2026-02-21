#include <WiFi.h>
#include <HTTPClient.h>

// Mobile Hotspot Credentials
const char* ssid = "AndroidAP";
const char* password = "12345678";

// Server URL
const char* serverURL = "http://10.98.246.200:8000/api/bulb-data";

// Sensor Pins
const int voltagePin = 35;  // ZMPT101B Voltage Sensor
const int currentPin = 34;  // ZMCT103C Current Sensor

// CORRECTED CALIBRATION VALUES - ZMCT103C SPECIFIC
#define VOLTAGE_CALIBRATION 0.35      // ZMPT101B calibration factor
#define CURRENT_CALIBRATION 0.005     // ✅ CORRECTED: ZMCT103C calibration factor
#define SAMPLES_PER_CYCLE 200
#define MAINS_FREQUENCY 50

// IMPROVED CALIBRATION SETTINGS
#define VOLTAGE_NOISE_THRESHOLD 0.5   // 0.5V noise threshold
#define VOLTAGE_DEAD_ZONE 1.0         // 1.0V dead zone
#define CURRENT_NOISE_THRESHOLD 0.015 // 15mA noise level
#define MIN_REAL_CURRENT 0.030        // 30mA minimum real current
#define CURRENT_DEAD_ZONE 0.025       // 25mA dead zone
#define CALIBRATION_SAMPLES 3000      // More samples for better accuracy

// BULB SPECIFICATIONS - 100W EACH
#define BULB1_POWER 100.0
#define BULB2_POWER 100.0  
#define BULB3_POWER 100.0
#define BULB_CURRENT_THRESHOLD 0.08  // 80mA threshold for 100W bulb detection
#define POWER_TOLERANCE 20.0         // 20W tolerance for bulb detection

// Global variables
float voltageOffset = 0;
float currentOffset = 0;
float currentBaseline = 0;
float voltageBaseline = 0;
bool isCalibrated = false;
bool currentSensorConnected = false;
bool voltageSensorConnected = false;

// NEW: Dynamic calibration factors
float dynamicCurrentCalibration = CURRENT_CALIBRATION;
unsigned long lastCalibrationTime = 0;
bool calibrationStable = false;

// Bulb status tracking
struct BulbStatus {
  bool isOn;
  String name;
  float powerRating;
  float currentThreshold;
};

BulbStatus bulbs[3] = {
  {false, "Bulb 1 (100W)", BULB1_POWER, BULB_CURRENT_THRESHOLD},
  {false, "Bulb 2 (100W)", BULB2_POWER, BULB_CURRENT_THRESHOLD},
  {false, "Bulb 3 (100W)", BULB3_POWER, BULB_CURRENT_THRESHOLD}
};

void setup() {
  Serial.begin(115200);
  delay(3000);
  
  Serial.println("🎉 ESP32 ELECTRICITY MONITOR - CORRECTED CALIBRATION");
  Serial.println("===================================================");
  
  // Initialize sensors
  pinMode(voltagePin, INPUT);
  pinMode(currentPin, INPUT);
  
  // Configure ADC for maximum accuracy
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  // Test ADC reading first
  testSensorsInitial();
  
  // Improved calibration with multiple attempts
  if (!calibrateSensorsWithRetry(3)) {
    Serial.println("❌ CRITICAL: Calibration failed after multiple attempts!");
    Serial.println("🔧 Please check sensor connections and restart.");
  } else {
    isCalibrated = true;
  }
  
  // Connect to WiFi
  setupWiFi();
  
  Serial.println("🔧 System Ready!");
}

void testSensorsInitial() {
  Serial.println("🔍 INITIAL SENSOR TEST");
  Serial.println("======================");
  
  for(int i = 0; i < 5; i++) {
    int voltageRaw = analogRead(voltagePin);
    int currentRaw = analogRead(currentPin);
    float voltageVolts = (voltageRaw / 4095.0) * 3.3;
    float currentVolts = (currentRaw / 4095.0) * 3.3;
    
    Serial.print("📊 Voltage - Raw: ");
    Serial.print(voltageRaw);
    Serial.print(" | Volts: ");
    Serial.print(voltageVolts, 3);
    Serial.println("V");
    
    Serial.print("📊 Current - Raw: ");
    Serial.print(currentRaw);
    Serial.print(" | Volts: ");
    Serial.print(currentVolts, 3);
    Serial.println("V");
    Serial.println("---");
    
    delay(500);
  }
}

bool calibrateSensorsWithRetry(int maxAttempts) {
  for (int attempt = 1; attempt <= maxAttempts; attempt++) {
    Serial.println();
    Serial.print("🔄 CALIBRATION ATTEMPT ");
    Serial.println(attempt);
    Serial.println("======================");
    
    calibrateSensors();
    
    if (verifyCalibrationStrict()) {
      Serial.println("✅ CALIBRATION SUCCESSFUL!");
      return true;
    } else {
      Serial.println("❌ Calibration failed, retrying...");
      delay(2000);
    }
  }
  return false;
}

void calibrateSensors() {
  Serial.println("🔧 CALIBRATING SENSORS...");
  Serial.println("⚠  IMPORTANT: Ensure NO power to sensors!");
  Serial.println("⏳ Starting in 2 seconds...");
  delay(2000);
  
  calibrateCurrentZeroImproved();
  calibrateVoltageZero();
  calibrateSensorBaselines();
}

void calibrateCurrentZeroImproved() {
  Serial.println("🔋 CALIBRATING ZMCT103C CURRENT SENSOR...");
  
  // Phase 1: Quick calibration
  long quickSum = 0;
  for (int i = 0; i < 1000; i++) {
    quickSum += analogRead(currentPin);
    delay(1);
  }
  float quickOffset = quickSum / 1000.0;
  
  // Phase 2: Detailed calibration with stability check
  long detailedSum = 0;
  int stableSamples = 0;
  float lastReading = 0;
  
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    int rawValue = analogRead(currentPin);
    detailedSum += rawValue;
    
    // Stability check - reject fluctuating readings
    if (i > 0 && abs(rawValue - lastReading) < 5) {
      stableSamples++;
    }
    lastReading = rawValue;
    
    if(i % 300 == 0) {
      Serial.print(".");
    }
    delay(2);
  }
  
  currentOffset = detailedSum / (float)CALIBRATION_SAMPLES;
  
  Serial.println();
  Serial.print("📊 Quick Offset: "); Serial.println(quickOffset);
  Serial.print("📊 Detailed Offset: "); Serial.println(currentOffset);
  Serial.print("📊 Stable Samples: "); Serial.print(stableSamples);
  Serial.print("/"); Serial.println(CALIBRATION_SAMPLES);
  
  // If too unstable, use quick offset
  if (stableSamples < CALIBRATION_SAMPLES * 0.8) {
    Serial.println("⚠  Using quick offset due to instability");
    currentOffset = quickOffset;
  }
}

void calibrateVoltageZero() {
  Serial.println("📏 CALIBRATING VOLTAGE SENSOR...");
  
  long voltageSum = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    voltageSum += analogRead(voltagePin);
    
    if(i % 300 == 0) {
      Serial.print(".");
    }
    delay(2);
  }
  
  voltageOffset = voltageSum / (float)CALIBRATION_SAMPLES;
  Serial.println();
  Serial.print("✅ Voltage Offset: "); Serial.println(voltageOffset);
}

void calibrateSensorBaselines() {
  Serial.println("🎯 CALIBRATING SENSOR BASELINES...");
  
  // Current baseline
  float currentSum = 0;
  for (int i = 0; i < 500; i++) {
    currentSum += readCurrentRawImproved();
    delay(3);
  }
  currentBaseline = currentSum / 500.0;
  
  // Voltage baseline  
  float voltageSum = 0;
  for (int i = 0; i < 500; i++) {
    voltageSum += readVoltageRaw();
    delay(3);
  }
  voltageBaseline = voltageSum / 500.0;
  
  Serial.print("🎯 Current Baseline: "); Serial.print(currentBaseline, 6); Serial.println(" A");
  Serial.print("🎯 Voltage Baseline: "); Serial.print(voltageBaseline, 6); Serial.println(" V");
  
  // Sensor connection detection
  checkSensorConnections();
}

void checkSensorConnections() {
  int rawVoltage = analogRead(voltagePin);
  int rawCurrent = analogRead(currentPin);
  
  voltageSensorConnected = (rawVoltage > 50 && rawVoltage < 4000);
  currentSensorConnected = (rawCurrent > 50 && rawCurrent < 4000);
  
  Serial.print("🔌 Voltage Sensor: ");
  Serial.println(voltageSensorConnected ? "CONNECTED" : "DISCONNECTED");
  Serial.print("🔌 Current Sensor: ");
  Serial.println(currentSensorConnected ? "CONNECTED" : "DISCONNECTED");
}

bool verifyCalibrationStrict() {
  Serial.println("🔍 STRICT CALIBRATION VERIFICATION...");
  
  float voltageTests[10] = {0};
  float currentTests[10] = {0};
  bool voltageStable = true;
  bool currentStable = true;
  
  for(int j = 0; j < 10; j++) {
    voltageTests[j] = readVoltage();
    currentTests[j] = readCurrentImproved();
    
    // Check for stability during test
    if (j > 0) {
      if (abs(voltageTests[j] - voltageTests[j-1]) > 0.1) voltageStable = false;
      if (abs(currentTests[j] - currentTests[j-1]) > 0.01) currentStable = false;
    }
    delay(250);
  }
  
  float avgVoltage = 0, avgCurrent = 0;
  float maxVoltage = 0, maxCurrent = 0;
  
  for(int j = 0; j < 10; j++) {
    avgVoltage += voltageTests[j];
    avgCurrent += currentTests[j];
    if (abs(voltageTests[j]) > maxVoltage) maxVoltage = abs(voltageTests[j]);
    if (abs(currentTests[j]) > maxCurrent) maxCurrent = abs(currentTests[j]);
  }
  avgVoltage /= 10.0;
  avgCurrent /= 10.0;
  
  Serial.print("📊 Voltage - Avg: "); Serial.print(avgVoltage, 4);
  Serial.print(" V, Max: "); Serial.print(maxVoltage, 4); Serial.println(" V");
  Serial.print("📊 Current - Avg: "); Serial.print(avgCurrent, 4); 
  Serial.print(" A, Max: "); Serial.print(maxCurrent, 4); Serial.println(" A");
  Serial.print("📊 Stability - Voltage: "); Serial.print(voltageStable ? "STABLE" : "UNSTABLE");
  Serial.print(" | Current: "); Serial.println(currentStable ? "STABLE" : "UNSTABLE");
  
  // Strict criteria for success
  bool success = (abs(avgVoltage) < 0.15) && (abs(avgCurrent) < 0.008) && 
                 (maxVoltage < 0.3) && (maxCurrent < 0.015) &&
                 voltageStable && currentStable;
  
  if (success) {
    calibrationStable = true;
    lastCalibrationTime = millis();
  }
  
  return success;
}

// ✅ CORRECTED CURRENT READING WITH PROPER CALIBRATION
float readCurrentRawImproved() {
  unsigned long sumSquares = 0;
  int validSamples = 0;
  
  for (int i = 0; i < SAMPLES_PER_CYCLE; i++) {
    int rawValue = analogRead(currentPin);
    
    // Reject obviously bad readings
    if (rawValue > 100 && rawValue < 4000) {
      int correctedValue = rawValue - currentOffset;
      sumSquares += (unsigned long)(correctedValue * correctedValue);
      validSamples++;
    }
    
    delayMicroseconds(1000000 / (MAINS_FREQUENCY * SAMPLES_PER_CYCLE));
  }
  
  if (validSamples < SAMPLES_PER_CYCLE * 0.8) {
    return 0; // Too many bad samples
  }
  
  // ✅ CORRECTED: Use dynamic calibration factor
  float rmsValue = sqrt((float)sumSquares / validSamples);
  return rmsValue * dynamicCurrentCalibration;
}

float readVoltageRaw() {
  unsigned long sumSquares = 0;
  int validSamples = 0;
  
  for (int i = 0; i < SAMPLES_PER_CYCLE; i++) {
    int rawValue = analogRead(voltagePin);
    
    if (rawValue > 100 && rawValue < 4000) {
      int correctedValue = rawValue - voltageOffset;
      sumSquares += (unsigned long)(correctedValue * correctedValue);
      validSamples++;
    }
    
    delayMicroseconds(1000000 / (MAINS_FREQUENCY * SAMPLES_PER_CYCLE));
  }
  
  if (validSamples < SAMPLES_PER_CYCLE * 0.8) {
    return 0;
  }
  
  return sqrt((float)sumSquares / validSamples) * VOLTAGE_CALIBRATION;
}

float readCurrentImproved() {
  if(!isCalibrated || !currentSensorConnected) return 0;
  
  float currentRaw = readCurrentRawImproved();
  
  // Remove baseline noise with hysteresis
  float noiseCorrected = currentRaw - currentBaseline;
  
  // Apply dead zone with hysteresis
  static float lastValidCurrent = 0;
  
  if (abs(noiseCorrected) < CURRENT_DEAD_ZONE) {
    lastValidCurrent = 0;
    return 0.0;
  }
  
  // Only return current if it's above minimum real current threshold
  if (noiseCorrected < MIN_REAL_CURRENT) {
    lastValidCurrent = 0;
    return 0.0;
  }
  
  // Smooth transition using last valid value
  float filteredCurrent = (noiseCorrected + lastValidCurrent * 2) / 3.0;
  lastValidCurrent = filteredCurrent;
  
  // ✅ CORRECTED: Limit current to realistic range for 3 bulbs
  if (filteredCurrent > 2.0) { // Maximum expected current for 3 bulbs
    filteredCurrent = 1.5; // Safe limit
  }
  
  // Auto-recalibration if drift detected
  if (calibrationStable && millis() - lastCalibrationTime > 30000) {
    if (filteredCurrent > 0.5 && readVoltage() < 5.0) {
      Serial.println("⚠  Current drift detected, recalibrating...");
      calibrateCurrentZeroImproved();
      lastCalibrationTime = millis();
      return 0;
    }
  }
  
  return filteredCurrent;
}

float readVoltage() {
  if(!isCalibrated || !voltageSensorConnected) return 0;
  
  float voltageRaw = readVoltageRaw();
  float noiseCorrected = voltageRaw - voltageBaseline;
  
  if (abs(noiseCorrected) < VOLTAGE_DEAD_ZONE) return 0.0;
  if (abs(noiseCorrected) < VOLTAGE_NOISE_THRESHOLD) return 0.0;
  
  static float voltageBuffer[5] = {0};
  static int voltageIndex = 0;
  
  voltageBuffer[voltageIndex] = noiseCorrected;
  voltageIndex = (voltageIndex + 1) % 5;
  
  float filteredVoltage = 0;
  for (int i = 0; i < 5; i++) filteredVoltage += voltageBuffer[i];
  filteredVoltage /= 5.0;
  
  return (filteredVoltage < 0) ? 0 : filteredVoltage;
}

// ✅ IMPROVED BULB DETECTION
void detectBulbStatus(float current, float power) {
  // Reset all bulb status
  for(int i = 0; i < 3; i++) {
    bulbs[i].isOn = false;
  }
  
  // ✅ CORRECTED: Realistic current thresholds for 100W bulbs at 220V
  // 100W bulb at 220V = ~0.45A, but considering power factor ~0.45-0.5A
  float singleBulbCurrent = 0.45;    // 1 bulb ~ 0.45A
  float twoBulbsCurrent = 0.90;      // 2 bulbs ~ 0.90A  
  float threeBulbsCurrent = 1.35;    // 3 bulbs ~ 1.35A
  
  // Detect bulbs based on current with tolerance
  if (current >= 0.35 && current <= 0.60) {
    bulbs[0].isOn = true; // Only bulb 1 ON
  }
  else if (current >= 0.70 && current <= 1.10) {
    bulbs[0].isOn = true;
    bulbs[1].isOn = true; // Bulb 1 and 2 ON
  }
  else if (current >= 1.20) {
    bulbs[0].isOn = true;
    bulbs[1].isOn = true;
    bulbs[2].isOn = true; // All bulbs ON
  }
  
  // Additional verification based on power
  if (power >= 80.0 && power <= 120.0 && !bulbs[1].isOn && !bulbs[2].isOn) {
    bulbs[0].isOn = true;
  }
  else if (power >= 180.0 && power <= 220.0 && !bulbs[2].isOn) {
    bulbs[0].isOn = true;
    bulbs[1].isOn = true;
  }
  else if (power >= 280.0) {
    bulbs[0].isOn = true;
    bulbs[1].isOn = true;
    bulbs[2].isOn = true;
  }
}

void setupWiFi() {
  Serial.println("📡 CONNECTING TO WiFi...");
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while(WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }
  
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("✅ WiFi Connected!");
    Serial.print("📡 IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("❌ WiFi Failed!");
  }
}

// ✅ NEW FUNCTION: Auto-calibrate current sensor with known load
void autoCalibrateWithKnownLoad() {
  Serial.println("🎯 AUTO-CALIBRATION WITH KNOWN LOAD");
  Serial.println("⚠  Please turn ON ONE bulb and press any key...");
  
  while(!Serial.available()) {
    delay(100);
  }
  Serial.read(); // Clear buffer
  
  // Measure current with one known bulb
  float currentSum = 0;
  for(int i = 0; i < 100; i++) {
    currentSum += readCurrentRawImproved();
    delay(50);
  }
  float measuredCurrent = currentSum / 100.0;
  
  // Calculate correction factor (expected 0.45A for 100W bulb)
  float expectedCurrent = 0.45;
  if(measuredCurrent > 0.1) {
    dynamicCurrentCalibration = (expectedCurrent / measuredCurrent) * CURRENT_CALIBRATION;
    Serial.print("✅ Auto-calibration complete. New factor: ");
    Serial.println(dynamicCurrentCalibration, 6);
  }
}

void loop() {
  float voltage = readVoltage();
  float current = readCurrentImproved();
  float power = voltage * current;
  
  // Apply final dead zones
  if(abs(voltage) < VOLTAGE_DEAD_ZONE) voltage = 0;
  if(abs(current) < CURRENT_DEAD_ZONE) current = 0;
  if(abs(power) < 0.1) power = 0;
  
  // ✅ CORRECTED: If current is still too high, force recalibration
  if (current > 2.0 && isCalibrated) {
    Serial.println("⚠  Current too high, recalibrating...");
    calibrateCurrentZeroImproved();
    current = 0;
  }
  
  // Detect bulb status
  detectBulbStatus(current, power);
  
  // Display data
  displaySensorData(voltage, current, power);
  
  // Send to server
  if(WiFi.status() == WL_CONNECTED && isCalibrated) {
    sendToServer(voltage, current, power);
  }
  
  // ✅ Auto-calibration option (uncomment if needed)
  // if (millis() > 30000 && !calibrationStable) {
  //   autoCalibrateWithKnownLoad();
  // }
  
  Serial.println("=========================");
  delay(2000);
}

void displaySensorData(float v, float i, float p) {
  Serial.println("=== LIVE SENSOR DATA ===");
  
  Serial.print("⚡ Voltage: "); Serial.print(v, 2); Serial.print(" V");
  if (!voltageSensorConnected) Serial.println(" [SENSOR ERROR]");
  else if (v < 5.0) Serial.println(" [NO SUPPLY]");
  else Serial.println(" [LIVE]");
  
  Serial.print("🔋 Current: "); Serial.print(i, 3); Serial.print(" A");
  if (!currentSensorConnected) Serial.println(" [SENSOR ERROR]");
  else if (i < MIN_REAL_CURRENT) Serial.println(" [NO LOAD]");
  else Serial.println(" [REAL LOAD]");
  
  Serial.print("💡 Power: "); Serial.print(p, 2); Serial.println(" W");
  
  // Display detailed bulb status
  Serial.println("💡 BULB STATUS:");
  int bulbsOn = 0;
  for(int i = 0; i < 3; i++) {
    Serial.print("   ");
    Serial.print(bulbs[i].name);
    Serial.print(": ");
    if(bulbs[i].isOn) {
      Serial.println("🟢 ON");
      bulbsOn++;
    } else {
      Serial.println("🔴 OFF");
    }
  }
  
  Serial.print("📊 Total Bulbs ON: ");
  Serial.print(bulbsOn);
  Serial.print("/3 | Estimated Power: ");
  Serial.print(bulbsOn * 100.0);
  Serial.println("W");
  
  // Debug info
  int rawV = analogRead(voltagePin);
  int rawC = analogRead(currentPin);
  Serial.print("📊 Raw ADC - V:"); Serial.print(rawV);
  Serial.print(" C:"); Serial.println(rawC);
}

// ✅ CORRECTED: Fixed syntax error in sendToServer function
void sendToServer(float v, float i, float p) {
  if(WiFi.status() != WL_CONNECTED) return;
  
  HTTPClient http;
  http.begin(serverURL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(5000);
  
  String jsonData = "{";
  jsonData += "\"voltage\":" + String(v, 2) + ",";
  jsonData += "\"current\":" + String(i, 3) + ",";
  jsonData += "\"power\":" + String(p, 2) + ",";
  jsonData += "\"bulb1\":" + String(bulbs[0].isOn ? "true" : "false") + ",";
  jsonData += "\"bulb2\":" + String(bulbs[1].isOn ? "true" : "false") + ",";
  jsonData += "\"bulb3\":" + String(bulbs[2].isOn ? "true" : "false");
  jsonData += "}";
  
  int httpCode = http.POST(jsonData);
  
  if(httpCode > 0) {
    Serial.print("📡 Server response: ");
    Serial.println(httpCode);
  } else {
    Serial.print("❌ Send failed: ");
    Serial.println(httpCode);
    // Don't show "Data sent" message if failed
    return;
  }
  
  http.end();
  Serial.println("✅ Data sent to server!");
}
