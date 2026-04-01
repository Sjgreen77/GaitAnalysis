#include <bluefruit.h>

// ---- Pin Definitions ----
#define VBAT_ENABLE       14
#define PIN_VBAT          35
#define PIN_ADC_IN        A0   // pin 0

// ---- ADC / Battery Constants ----
#define ADC_RESOLUTION_BITS  12
#define ADC_COUNTS           4096.0f
#define VREF                 3.6f
#define R_UPPER              1000.0f   // 1M ohm  (VBAT divider)
#define R_LOWER              510.0f    // 510k ohm (VBAT divider)
#define VBAT_SCALE           (VREF / ADC_COUNTS * (R_UPPER + R_LOWER) / R_LOWER)
#define ADC_SCALE            (VREF / ADC_COUNTS)  // A0 is a direct read, no divider

#define VBAT_MIN             3.0f
#define VBAT_MAX             4.2f

// ---- BLE ----
BLEService        dataService("12345678-1234-5678-1234-56789abcdef0");
BLECharacteristic dataChar  ("12345678-1234-5678-1234-56789abcdef1");

// ---- Sample interval ----
#define SAMPLE_INTERVAL_MS   1000

void setup() {
    analogReadResolution(ADC_RESOLUTION_BITS);

    // Enable battery ADC path
    pinMode(VBAT_ENABLE, OUTPUT);
    digitalWrite(VBAT_ENABLE, LOW);

    // ---- BLE init ----
    Bluefruit.begin(1, 0);
    Bluefruit.setName("SensePlus");

    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    dataService.begin();

    dataChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
    dataChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    dataChar.setMaxLen(64);     // enough for our CSV string
    dataChar.begin();

    startAdvertising();
}

void startAdvertising() {
    Bluefruit.Advertising.clearData();
    Bluefruit.ScanResponse.clearData();

    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addService(dataService);
    Bluefruit.ScanResponse.addName();

    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.start(0);
}

// ---- Battery helpers ----
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

// ---- ADC read ----
float readA0Voltage() {
    const int NUM_SAMPLES = 16;
    uint32_t sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += analogRead(PIN_ADC_IN);
        delay(2);
    }
    return (float)(sum / NUM_SAMPLES) * ADC_SCALE;
}

// ---- BLE send ----
void sendBLE(String msg) {
    if (Bluefruit.connected()) {
        dataChar.write(msg.c_str(), msg.length());
        dataChar.notify(msg.c_str(), msg.length());
    }
}

// ---- Main loop ----
void loop() {
    static uint32_t lastSample = 0;

    if (millis() - lastSample >= SAMPLE_INTERVAL_MS) {
        lastSample = millis();

        float vbat    = readBatteryVoltage();
        int   pct     = voltageToPercent(vbat);
        float adcVolt = readA0Voltage();

        // CSV format: "VBAT:3.901,PCT:82,A0:1.234"
        // MATLAB can parse this with strsplit on ',' then ':' 
        char buf[64];
        snprintf(buf, sizeof(buf), "VBAT:%.3f,PCT:%d,A0:%.3f",
                 vbat, pct, adcVolt);

        sendBLE(String(buf));
    }
}

void connect_callback(uint16_t conn_handle) {
    // LED indicator — blue on when connected
    digitalWrite(LED_BLUE, LOW);   // LOW = on for XIAO
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    digitalWrite(LED_BLUE, HIGH);  // off
}
```

**What MATLAB will receive** over the characteristic, once per second:
```
VBAT:3.901,PCT:82,A0:1.234