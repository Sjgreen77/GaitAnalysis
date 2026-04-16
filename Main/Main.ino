#include <Wire.h>
#include <LSM6DS3.h>
#include "Config.h"
#include "GaitClassifier.h"
#include "SDManager.h"
#include "BLEManager.h"

// --- Global Instances ---
LSM6DS3 imu(I2C_MODE, 0x6A);
GaitClassifier classifier;
SDManager sdManager;
BLEManager bleManager;

unsigned long lastSampleTime = 0;
unsigned long lastBatteryTime = 0;
unsigned long correctCount = 0;
unsigned long incorrectCount = 0;

// Sync state (only used while connected)
bool isSyncing = false;

// --- Battery helpers (from battery_read_and_adc_reporting) ---
float readBatteryVoltage() {
    const int NUM_SAMPLES = 16;
    uint32_t sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += analogRead(PIN_VBAT);
        delay(5);
    }
    return (float)(sum / NUM_SAMPLES) * VBAT_SCALE;
}

int voltageToPercent(float voltage) {
    if (voltage >= VBAT_MAX) return 100;
    if (voltage <= VBAT_MIN) return 0;

    const float v[] = { 3.0f, 3.5f, 3.6f, 3.7f, 3.85f, 4.2f };
    const float p[] = {   0,   20,   40,   60,    80,   100  };

    for (int i = 0; i < 5; i++) {
        if (voltage <= v[i + 1]) {
            float t = (voltage - v[i]) / (v[i + 1] - v[i]);
            return (int)(p[i] + t * (p[i + 1] - p[i]));
        }
    }
    return 100;
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);

    // Enable battery ADC path
    pinMode(VBAT_ENABLE, OUTPUT);
    digitalWrite(VBAT_ENABLE, LOW);

    // Initialize Modules
    if (imu.begin() != 0) {
        Serial.println("IMU Error!");
        while(1);
    }

    sdManager.begin();
    bleManager.begin();

    Serial.println("System Ready.");
}

void loop() {
    if (bleManager.isConnected()) {
        // ========== CONNECTED TO MATLAB ==========
        // Report battery + handle sync requests. No step sampling.

        // --- Battery reporting every 1 second (skip during sync) ---
        unsigned long now = millis();
        if (!isSyncing && now - lastBatteryTime >= BATTERY_INTERVAL_MS) {
            lastBatteryTime = now;

            float vbat = readBatteryVoltage();
            int pct = voltageToPercent(vbat);

            char buf[64];
            snprintf(buf, sizeof(buf), "VBAT:%.3f,PCT:%d", vbat, pct);
            bleManager.sendData(buf, strlen(buf));
        }

        // --- Check for SYNC command ---
        if (globalSyncFlag && !isSyncing) {
            globalSyncFlag = false;
            if (sdManager.openForRead()) {
                isSyncing = true;
                delay(100); // Let any in-flight battery notification finish
                Serial.println("Starting BLE File Transfer...");
            }
        }

        // --- Stream file data if syncing ---
        if (isSyncing) {
            uint8_t chunk[20];
            int bytesRead = sdManager.readChunkRaw(chunk, sizeof(chunk));

            if (bytesRead > 0) {
                // Flow control: retry until BLE stack accepts the packet
                while (!bleManager.notifyData(chunk, bytesRead)) {
                    delay(1);
                }
            } else {
                // File finished — send EOF marker
                const uint8_t eof[] = {'E', 'O', 'F'};
                while (!bleManager.notifyData(eof, 3)) {
                    delay(1);
                }
                isSyncing = false;
                Serial.println("Transfer Complete.");
            }
        }

    } else {
        // ========== DISCONNECTED — STANDALONE SAMPLING ==========
        // Collect step data and log to SD card.
        isSyncing = false; // Reset if we were mid-sync and got disconnected

        unsigned long currentMillis = millis();
        if (currentMillis - lastSampleTime >= SAMPLE_INTERVAL_MS) {
            lastSampleTime = currentMillis;

            // 1. Read Force Sensors
            int heel = analogRead(PIN_ADC_HEEL);
            int toe  = analogRead(PIN_ADC_TOE);

            // 2. Process step detection (assumes walking state — hardcoded for now)
            // TODO: Re-add IMU-based motion state classification here.
            //       When re-added, only call processSample() when state == WALKING.
            MotionState result = classifier.processSample(heel, toe, currentMillis);

            // 3. If a step just completed, log it
            if (result == WALKING_CORRECT) {
                correctCount++;
                Serial.print("CORRECT STEP #");
                Serial.print(correctCount);
                Serial.print("  (heel="); Serial.print(heel);
                Serial.print(" toe="); Serial.print(toe);
                Serial.println(")");
                sdManager.logStep(result);
            } else if (result == WALKING_INCORRECT) {
                incorrectCount++;
                Serial.print("INCORRECT STEP #");
                Serial.print(incorrectCount);
                Serial.print("  (heel="); Serial.print(heel);
                Serial.print(" toe="); Serial.print(toe);
                Serial.println(")");
                sdManager.logStep(result);
            }
        }
    }
}
