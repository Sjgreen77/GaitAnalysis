#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include "Config.h"
#include <bluefruit.h>

// Forward declarations for the static BLE callback
volatile bool globalSyncFlag = false;
volatile bool globalNextFlag = false;
void triggerSync() { globalSyncFlag = true; }
void triggerNext() { globalNextFlag = true; }

class BLEManager {
private:
    BLEService gaitService;
    BLECharacteristic gaitChar;

public:
    BLEManager() : gaitService(SERVICE_UUID), gaitChar(CHAR_UUID) {}

    void begin() {
        Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

        Bluefruit.begin(1, 1);
        Bluefruit.setName("SensePlus");

        Bluefruit.Periph.setConnInterval(6, 12);

        // LED off by default (HIGH = off on XIAO)
        pinMode(LED_BLUE, OUTPUT);
        digitalWrite(LED_BLUE, HIGH);

        // Connection callbacks for LED indicator
        Bluefruit.Periph.setConnectCallback(connect_callback);
        Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

        gaitService.begin();

        gaitChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ | CHR_PROPS_WRITE);
        gaitChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
        gaitChar.setWriteCallback(write_callback);
        gaitChar.setMaxLen(244);
        gaitChar.begin();

        startAdvertising();
    }

    // For battery reporting (write + notify so MATLAB polling sees it)
    void sendData(const char* data, int len) {
        if (Bluefruit.connected()) {
            gaitChar.write(data, len);
            gaitChar.notify(data, len);
        }
    }

    // Write data to characteristic value (for request-response file transfer)
    void writeCharValue(const uint8_t* data, uint16_t len) {
        gaitChar.write(data, len);
    }

    bool isConnected() {
        return Bluefruit.connected();
    }

    // Static callback wrapper required by Bluefruit library
    static void write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
        String command = "";
        for (int i = 0; i < len; i++) command += (char)data[i];

        if (command.indexOf("SYNC") >= 0) {
            triggerSync();
        } else if (command.indexOf("NEXT") >= 0) {
            triggerNext();
        }
    }

    // Static callbacks for BLE connection events
    static void connect_callback(uint16_t conn_handle) {
        (void)conn_handle;
        digitalWrite(LED_BLUE, LOW);
    }

    static void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
        (void)conn_handle;
        (void)reason;
        digitalWrite(LED_BLUE, HIGH);
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
