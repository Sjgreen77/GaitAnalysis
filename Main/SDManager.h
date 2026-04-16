#ifndef SD_MANAGER_H
#define SD_MANAGER_H

#include "Config.h"
#include <SPI.h>
#include "SdFat.h"

class SDManager {
private:
    SdFat SD;
    File dataFile;
    bool isInitialized = false;

public:
    void begin() {
        if (!SD.begin(PIN_SD_CS, SPI_HALF_SPEED)) {
            Serial.println("SD Init Failed!");
            return;
        }
        isInitialized = true;
        Serial.println("SD Init Success.");
    }

    void logStep(MotionState state) {
        if (!isInitialized) return;

        dataFile = SD.open("gait_log.txt", FILE_WRITE);
        if (dataFile) {
            unsigned long timestamp = millis();
            dataFile.print(timestamp);
            dataFile.print(",");
            dataFile.println(state == WALKING_CORRECT ? "CORRECT" : "INCORRECT");
            dataFile.close();
            Serial.print("[SD] logStep: wrote ");
            Serial.println(state == WALKING_CORRECT ? "CORRECT" : "INCORRECT");
        } else {
            Serial.println("[SD] logStep: ERROR - could not open file for writing");
        }
    }

    // Opens file for readback to MATLAB
    bool openForRead() {
        if (!isInitialized) return false;
        // Close any existing file handle first
        if (dataFile.isOpen()) {
            dataFile.close();
        }
        dataFile = SD.open("gait_log.txt", FILE_READ);
        Serial.print("[SD] openForRead: ");
        if (dataFile.isOpen()) {
            Serial.print("SUCCESS, file_size=");
            Serial.println(dataFile.size());
        } else {
            Serial.println("FAILED");
        }
        return dataFile.isOpen();
    }

    // Reads a raw chunk for BLE file transfer (no null terminator)
    int readChunkRaw(uint8_t* buffer, int maxLen) {
        if (!dataFile.isOpen()) {
            Serial.println("[SD] readChunkRaw: file not open!");
            return 0;
        }
        int bytesRead = dataFile.read(buffer, maxLen);
        Serial.print("[SD] readChunkRaw: bytesRead=");
        Serial.print(bytesRead);
        Serial.print(" first_4_bytes=");
        for (int i = 0; i < (bytesRead < 4 ? bytesRead : 4); i++) {
            Serial.print((char)buffer[i]);
        }
        Serial.println();

        if (bytesRead <= 0) {
            dataFile.close();
            Serial.println("[SD] readChunkRaw: EOF reached, file closed");
            return 0;
        }
        return bytesRead;
    }
};

#endif