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
unsigned long correctCount = 0;
unsigned long incorrectCount = 0;

// State Machine
enum SystemState { SAMPLING, SYNCING };
SystemState currentState = SAMPLING;

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);

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
    // Check for MATLAB Sync Command (e.g. "SYNC" was sent over BLE)
    if (globalSyncFlag && currentState != SYNCING) {
        globalSyncFlag = false;
        if (sdManager.openForRead()) {
            currentState = SYNCING;
            Serial.println("Starting BLE File Transfer...");
        }
    }

    // --- State Machine ---
    switch (currentState) {

        case SAMPLING: {
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
            break;
        }

        case SYNCING: {
            // Send data to MATLAB in non-blocking chunks
            char chunk[20];
            int bytesRead = sdManager.readChunk(chunk, 20);

            if (bytesRead > 0) {
                bleManager.sendChunk(chunk, bytesRead);
                delay(10); // Small delay to prevent flooding the BLE stack
            } else {
                // File transfer complete
                bleManager.sendChunk("EOF", 3); // Tell MATLAB we're done
                currentState = SAMPLING;
                Serial.println("Transfer Complete. Resuming sampling.");
            }
            break;
        }
    }
}
