#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include "Config.h"
#include <bluefruit.h>

// Forward declarations for the static BLE callback
volatile bool globalSyncFlag = false;
void triggerSync() { globalSyncFlag = true; }

class BLEManager {
private:
    BLEService gaitService;
    BLECharacteristic gaitChar;
    bool syncRequested = false;

public:
    BLEManager() : gaitService(SERVICE_UUID), gaitChar(CHAR_UUID) {}

    void begin() {
        // Configure for max throughput BEFORE begin()
        // Sets MTU=247, fast connection interval, large TX queue
        Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

        Bluefruit.begin(1, 1); // 1 peripheral, 1 central
        Bluefruit.setName("SensePlus");

        // Request fast connection interval: 7.5ms - 15ms (units of 1.25ms)
        Bluefruit.Periph.setConnInterval(6, 12);

        // LED off by default (HIGH = off on XIAO)
        pinMode(LED_BLUE, OUTPUT);
        digitalWrite(LED_BLUE, HIGH);

        // Connection callbacks for LED indicator
        Bluefruit.Periph.setConnectCallback(connect_callback);
        Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

        gaitService.begin();

        // Allow MATLAB to Read, Notify, and Write commands to this characteristic
        gaitChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ | CHR_PROPS_WRITE);
        gaitChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
        gaitChar.setWriteCallback(write_callback);
        gaitChar.setMaxLen(244); // Support up to MTU 247 (247 - 3 ATT header)
        gaitChar.begin();

        startAdvertising();
    }

    // For battery reporting (write + notify so MATLAB read() sees latest value)
    void sendData(const char* data, int len) {
        if (Bluefruit.connected()) {
            gaitChar.write(data, len);
            gaitChar.notify(data, len);
        }
    }

    // Fast notification for file transfer — returns false if BLE queue is full
    bool notifyData(const uint8_t* data, uint16_t len) {
        if (!Bluefruit.connected()) return false;
        return gaitChar.notify(data, len);
    }

    bool isConnected() {
        return Bluefruit.connected();
    }

    bool isSyncRequested() {
        if (syncRequested) {
            syncRequested = false;
            return true;
        }
        return false;
    }

    // Static callback wrapper required by Bluefruit library
    static void write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
        String command = "";
        for (int i = 0; i < len; i++) command += (char)data[i];

        if (command.indexOf("SYNC") >= 0) {
            triggerSync();
        }
    }

    // Static callbacks for BLE connection events
    static void connect_callback(uint16_t conn_handle) {
        (void)conn_handle;
        digitalWrite(LED_BLUE, LOW);  // LOW = on for XIAO
    }

    static void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
        (void)conn_handle;
        (void)reason;
        digitalWrite(LED_BLUE, HIGH); // HIGH = off
    }

private:
    void startAdvertising() {
        Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
        Bluefruit.Advertising.addService(gaitService);
        Bluefruit.ScanResponse.addName();
        Bluefruit.Advertising.restartOnDisconnect(true);
        Bluefruit.Advertising.start(0);
    }
};

#endif
