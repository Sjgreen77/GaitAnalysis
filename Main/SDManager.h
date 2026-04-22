#ifndef SD_MANAGER_H
#define SD_MANAGER_H

#include "Config.h"
#include "GaitClassifier.h"   // for StepType
#include <SPI.h>
#include "SdFat.h"

#define MAX_SESSIONS 99  // Safety cap on session count

class SDManager {
private:
    SdFat  SD;
    File   dataFile;
    bool   isInitialized      = false;
    int    currentSession     = 1;   // Session number being recorded right now
    int    currentReadSession = 1;   // Session being streamed during transfer
    int    totalSessions      = 0;   // How many sessions exist for transfer
    char   currentSessionFile[16];   // e.g. "s1.csv"
    bool   hasWrittenThisSession = false; // Bumps session.txt on first write only

    // --- Private helpers ---

    int readSessionFile() {
        if (!SD.exists("session.txt")) return 1;
        if (dataFile.isOpen()) dataFile.close();
        dataFile = SD.open("session.txt", FILE_READ);
        if (!dataFile.isOpen()) return 1;
        String s = "";
        while (dataFile.available()) s += (char)dataFile.read();
        dataFile.close();
        int n = s.toInt();
        return (n >= 1 && n <= MAX_SESSIONS) ? n : 1;
    }

    void writeSessionFile(int n) {
        // Remove then recreate to ensure clean overwrite (no partial appends)
        if (dataFile.isOpen()) dataFile.close();
        if (SD.exists("session.txt")) SD.remove("session.txt");
        dataFile = SD.open("session.txt", FILE_WRITE);
        if (dataFile.isOpen()) {
            dataFile.println(n);
            dataFile.close();
        } else {
            Serial.println("[SD] ERROR: could not write session.txt");
        }
    }

    bool openSessionForRead(int n) {
        char fname[16];
        snprintf(fname, sizeof(fname), "s%d.csv", n);
        if (!SD.exists(fname)) {
            Serial.print("[SD] Session file not found: ");
            Serial.println(fname);
            return false;
        }
        dataFile = SD.open(fname, FILE_READ);
        if (dataFile.isOpen()) {
            Serial.print("[SD] Opened: ");
            Serial.print(fname);
            Serial.print("  size=");
            Serial.println(dataFile.size());
        }
        return dataFile.isOpen();
    }

    // Determine how many session files actually exist on the card.
    // Includes the current session (i <= currentSession) so active data is transferred.
    int countExistingSessions() {
        int count = 0;
        for (int i = 1; i <= currentSession; i++) {
            char fname[16];
            snprintf(fname, sizeof(fname), "s%d.csv", i);
            if (SD.exists(fname)) count = i;  // Track highest valid index
        }
        return count;
    }

public:
    void begin() {
        if (!SD.begin(PIN_SD_CS, SPI_HALF_SPEED)) {
            Serial.println("[SD] Init Failed!");
            return;
        }
        isInitialized = true;
        Serial.println("[SD] Init Success.");
        beginSession();
    }

    // Called at startup (and after deleteAllSessions): reads session counter
    // and names the current session file. Does NOT advance session.txt — that
    // only happens on the first actual write (see logStep). This way a quick
    // boot-and-power-off without recording doesn't consume a session slot.
    void beginSession() {
        currentSession = readSessionFile();
        snprintf(currentSessionFile, sizeof(currentSessionFile), "s%d.csv", currentSession);
        hasWrittenThisSession = false;

        Serial.print("[SD] Starting session ");
        Serial.print(currentSession);
        Serial.print(" -> recording to: ");
        Serial.println(currentSessionFile);
    }

    // Append one step to the current session file.
    // On the FIRST successful write of this session, advance session.txt to
    // currentSession + 1 so the next power cycle lands on a fresh file.
    void logStep(StepType step) {
        if (!isInitialized) return;

        dataFile = SD.open(currentSessionFile, FILE_WRITE);
        if (dataFile) {
            dataFile.print(millis());
            dataFile.print(",");
            dataFile.println(step == STEP_CORRECT ? "CORRECT" : "INCORRECT");
            dataFile.close();

            if (!hasWrittenThisSession) {
                hasWrittenThisSession = true;
                writeSessionFile(currentSession + 1);
                Serial.print("[SD] First write to session ");
                Serial.print(currentSession);
                Serial.print(" -> session.txt advanced to ");
                Serial.println(currentSession + 1);
            }
        } else {
            Serial.println("[SD] logStep: ERROR opening session file");
        }
    }

    // Opens ALL existing session files for sequential BLE readback.
    // Sessions are streamed one after another via readChunkRaw().
    bool openAllSessionsForRead() {
        if (!isInitialized) return false;
        if (dataFile.isOpen()) dataFile.close();

        currentReadSession = 1;
        totalSessions = countExistingSessions();

        if (totalSessions < 1) {
            Serial.println("[SD] No session files to transfer.");
            return false;
        }

        Serial.print("[SD] Transferring ");
        Serial.print(totalSessions);
        Serial.println(" session(s).");
        return openSessionForRead(currentReadSession);
    }

    // Reads up to maxLen bytes. When the current session file is exhausted,
    // automatically advances to the next one. Returns 0 only when ALL sessions done.
    int readChunkRaw(uint8_t* buffer, int maxLen) {
        if (!dataFile.isOpen()) {
            Serial.println("[SD] readChunkRaw: no file open");
            return 0;
        }

        int bytesRead = dataFile.read(buffer, maxLen);

        // Log for debugging
        Serial.print("[SD] readChunkRaw: session=");
        Serial.print(currentReadSession);
        Serial.print(" bytesRead=");
        Serial.print(bytesRead);
        Serial.print(" first_4=");
        for (int i = 0; i < (bytesRead < 4 ? bytesRead : 4); i++) {
            Serial.print((char)buffer[i]);
        }
        Serial.println();

        if (bytesRead > 0) return bytesRead;

        // Current session exhausted — try the next one
        dataFile.close();
        Serial.print("[SD] Session ");
        Serial.print(currentReadSession);
        Serial.println(" fully read.");

        currentReadSession++;
        if (currentReadSession <= totalSessions) {
            if (openSessionForRead(currentReadSession)) {
                return readChunkRaw(buffer, maxLen);  // Recurse into new session
            }
        }

        Serial.println("[SD] All sessions streamed.");
        return 0;
    }

    // Called after MATLAB confirms receipt with "DONE".
    // Deletes every session file and resets the counter so the next boot
    // starts fresh at session 1.
    void deleteAllSessions() {
        if (!isInitialized) return;
        if (dataFile.isOpen()) dataFile.close();

        Serial.println("[SD] Clearing all session files...");
        for (int i = 1; i <= currentSession; i++) {  // <= includes active session
            char fname[16];
            snprintf(fname, sizeof(fname), "s%d.csv", i);
            if (SD.exists(fname)) {
                SD.remove(fname);
                Serial.print("[SD] Deleted: ");
                Serial.println(fname);
            }
        }

        // Reset session state so the next recording (or next boot) starts at s1.csv.
        // session.txt stays at 1 — it will only advance to 2 once data is actually
        // written (see logStep). This handles both cases correctly:
        //   - Record immediately after upload: logStep bumps session.txt to 2,
        //     so next boot opens s2.csv.
        //   - Power off immediately after upload: next boot reads 1, opens s1.csv,
        //     and session.txt stays at 1 until something is recorded.
        currentSession = 1;
        snprintf(currentSessionFile, sizeof(currentSessionFile), "s1.csv");
        writeSessionFile(1);
        hasWrittenThisSession = false;
        Serial.println("[SD] All sessions cleared. Ready to record into s1.csv.");
    }

    int getCurrentSession() const { return currentSession; }
    int getTotalSessions()   const { return totalSessions;  }
};

#endif
