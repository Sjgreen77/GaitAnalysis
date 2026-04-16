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
        }
    }

    // Opens file for readback to MATLAB
    bool openForRead() {
        if (!isInitialized) return false;
        dataFile = SD.open("gait_log.txt", FILE_READ);
        return dataFile.isOpen();
    }

    // Reads a raw chunk for BLE file transfer (no null terminator)
    int readChunkRaw(uint8_t* buffer, int maxLen) {
        if (!dataFile.isOpen()) return 0;
        int bytesRead = dataFile.read(buffer, maxLen);
        if (bytesRead <= 0) {
            dataFile.close();
            return 0;
        }
        return bytesRead;
    }
};

#endif