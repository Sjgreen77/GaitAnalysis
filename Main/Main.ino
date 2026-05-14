#include <Wire.h>
#include <LSM6DS3.h>
#include "Config.h"

#include "GaitClassifier.h"
#include "MotionClassifier.h"
#include "SDManager.h"
#include "BLEManager.h"

// global instances
LSM6DS3 imu(I2C_MODE, 0x6A);
GaitClassifier   classifier;
MotionClassifier motionClassifier;
SDManager        sdManager;
BLEManager       bleManager;

// track previous motion state so we can reset the step classifier
// when leaving walking (prevents an old stance state from triggering
// a "ghost" step when walking continues).
MotionState prevMotion = STATIONARY;

unsigned long lastSampleTime = 0;
unsigned long lastBatteryTime = 0;
unsigned long correctCount = 0;
unsigned long incorrectCount = 0;

// sync state (only used while connected)
bool isSyncing = false;

// battery helper funcs
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

    // enable battery ADC path
    pinMode(VBAT_ENABLE, OUTPUT);
    digitalWrite(VBAT_ENABLE, LOW);

    // init modules
    if (imu.begin() != 0) {
        Serial.println("IMU Error!");
        while(1);
    }

    sdManager.begin();
    bleManager.begin();

    Serial.println("System Ready.");
}

// called when sync starts and each time MATLAB sends next.
// Writes next 240-byte chunk to characteristic value so MATLAB can read
void writeNextSyncChunk() {
    uint8_t chunk[240];
    int bytesRead = sdManager.readChunkRaw(chunk, sizeof(chunk));

    Serial.print("[WRITE_CHUNK] bytesRead=");
    Serial.print(bytesRead);

    if (bytesRead > 0) {
        Serial.print(" first_bytes=");
        for (int i = 0; i < (bytesRead < 10 ? bytesRead : 10); i++) {
            Serial.print((char)chunk[i]);
        }
        Serial.println("...");
        bleManager.writeCharValue(chunk, bytesRead);
    } else {
        Serial.println(" -> sending EOF marker");
        // send EOF marker that can't be confused with data
        bleManager.writeCharValue((const uint8_t*)"[EOF]", 5);
        delay(10);  // brief delay to ensure marker is written
        isSyncing = false;
        Serial.println("Transfer Complete.");
    }
}

void loop() {
    if (bleManager.isConnected()) {
        // now connected to matlab,
        // report battery and handle sync requests, no step sampling

        // battery reporting every second
        unsigned long now = millis();
        if (!isSyncing && now - lastBatteryTime >= BATTERY_INTERVAL_MS) {
            lastBatteryTime = now;

            float vbat = readBatteryVoltage();
            int pct = voltageToPercent(vbat);

            char buf[64];
            snprintf(buf, sizeof(buf), "VBAT:%.3f,PCT:%d", vbat, pct);
            bleManager.sendData(buf, strlen(buf));
        }

        // sync command, open all session files and stream the first chunk
        if (globalSyncFlag && !isSyncing) {
            Serial.println("[MAIN] SYNC received, opening all sessions for read...");
            globalSyncFlag = false;
            if (sdManager.openAllSessionsForRead()) {
                isSyncing = true;
                delay(100); // let any "in-flight" battery notification finish
                Serial.print("[MAIN] Transferring ");
                Serial.print(sdManager.getTotalSessions());
                Serial.println(" session(s).");
                writeNextSyncChunk();
            } else {
                Serial.println("[MAIN] No session data found on SD card.");
                // Tell matlab there's nothing to receive
                bleManager.writeCharValue((const uint8_t*)"[EOF]", 5);
            }
        }

        // next command: matlab has read the last chunk, send the next one
        if (globalNextFlag) {
            globalNextFlag = false;
            if (isSyncing) {
                Serial.println("[MAIN] NEXT received, writing next chunk...");
                writeNextSyncChunk();
            } else {
                // transfer already finished but MATLAB hasn't seen EOF yet,
                // resend it so matlab exits its polling loop.
                Serial.println("[MAIN] NEXT received after EOF — resending [EOF]");
                bleManager.writeCharValue((const uint8_t*)"[EOF]", 5);
            }
        }

        // done command: matlab confirmed successful save, clear all sessions
        if (globalDoneFlag) {
            globalDoneFlag = false;
            isSyncing = false;
            Serial.println("[MAIN] DONE received. Clearing all session data...");
            sdManager.deleteAllSessions();
            bleManager.writeCharValue((const uint8_t*)"[CLEARED]", 9);
            Serial.println("[MAIN] SD card cleared. Ready for new recording.");
        }

    } else {
        // disconnected from matlab,
        // collect step data and log to SD card.
        isSyncing = false; // reset if we were mid sync and got disconnected

        unsigned long currentMillis = millis();
        if (currentMillis - lastSampleTime >= SAMPLE_INTERVAL_MS) {
            lastSampleTime = currentMillis;

            // read all sensors
            float ax = imu.readFloatAccelX();
            float ay = imu.readFloatAccelY();
            float az = imu.readFloatAccelZ();
            float gx = imu.readFloatGyroX();
            float gy = imu.readFloatGyroY();
            float gz = imu.readFloatGyroZ();
            int heel = analogRead(PIN_ADC_HEEL);
            int toe  = analogRead(PIN_ADC_TOE);

            // top-level motion state fed every sample, decision refreshed once/sec
            MotionState motion = motionClassifier.processSample(ax, ay, az,
                                                                 gx, gy, gz,
                                                                 heel, toe);

            // reset step classifier when leaving walking so residual stance
            //    state doesn't fire a "ghost" step on the next walking window
            if (prevMotion == WALKING && motion != WALKING) {
                classifier.reset();
            }
            prevMotion = motion;

            // only classify steps and log to microSD when actively walking
            if (motion == WALKING) {
                StepType step = classifier.processSample(heel, toe, currentMillis);

                if (step == STEP_CORRECT) {
                    correctCount++;
                    Serial.print("CORRECT STEP #");
                    Serial.print(correctCount);
                    Serial.print("  (heel="); Serial.print(heel);
                    Serial.print(" toe="); Serial.print(toe);
                    Serial.println(")");
                    sdManager.logStep(step);
                } else if (step == STEP_INCORRECT) {
                    incorrectCount++;
                    Serial.print("INCORRECT STEP #");
                    Serial.print(incorrectCount);
                    Serial.print("  (heel="); Serial.print(heel);
                    Serial.print(" toe="); Serial.print(toe);
                    Serial.println(")");
                    sdManager.logStep(step);
                }
            }
        }
    }
}
