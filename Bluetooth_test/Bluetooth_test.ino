#include <bluefruit.h>

BLEService testService("12345678-1234-5678-1234-56789abcdef0");
BLECharacteristic testChar("12345678-1234-5678-1234-56789abcdef1");

String serialBuffer = "";

void setup() {
  Serial.begin(115200);

  Bluefruit.begin(1, 0);
  Bluefruit.setName("SensePlus");

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  testService.begin();

  // ---- IMPORTANT CHANGE ----
 testChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
 testChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
 testChar.setMaxLen(20);
 testChar.begin();
 testChar.write8(0);


  startAdvertising();

  Serial.println("Type in Serial Monitor to send BLE message");
}

void startAdvertising() {
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(testService);
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.start(0);
}

void loop() {

  // Read serial input
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      sendBLE(serialBuffer);
      serialBuffer = "";
    }
    else {
      serialBuffer += c;
    }
  }
}

void sendBLE(String msg) {
  if (Bluefruit.connected()) {
    Serial.print("Sending BLE: ");
    Serial.println(msg);

    // Write the data so it can be 'Read' by MATLAB's polling timer
    testChar.write(msg.c_str(), msg.length());
   
    // Also keep notify for compatibility
    testChar.notify(msg.c_str(), msg.length());
  }
}

void connect_callback(uint16_t conn_handle) {
  Serial.println("Connected to MATLAB");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  Serial.println("Disconnected");
}
