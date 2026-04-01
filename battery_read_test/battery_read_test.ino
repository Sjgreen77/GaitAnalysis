#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

#define VBAT_ENABLE   14
#define PIN_VBAT      35

#define VBAT_MIN      3.0f
#define VBAT_MAX      4.2f

#define ADC_RESOLUTION_BITS  12
#define ADC_COUNTS           4096.0f

// Seeed XIAO nRF52840: resistor divider is 1M + 510k (ratio = 1510/510)
// nRF52840 ADC internal reference = 3.6V
// VBAT = raw * (3.6 / 4096) * (1510.0 / 510.0)
#define R_UPPER       1000.0f   // 1M ohm
#define R_LOWER        510.0f   // 510k ohm
#define VREF           3.6f
#define VBAT_SCALE    (VREF / ADC_COUNTS * (R_UPPER + R_LOWER) / R_LOWER)

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    analogReadResolution(ADC_RESOLUTION_BITS);

    pinMode(VBAT_ENABLE, OUTPUT);
    digitalWrite(VBAT_ENABLE, LOW);

    Serial.println("XIAO nRF52840 Sense — Battery Monitor");
    Serial.print("VBAT_SCALE factor: ");
    Serial.println(VBAT_SCALE, 6);  // Should print ~0.002597
    Serial.println();
}

float readBatteryVoltage() {
    const int NUM_SAMPLES = 16;
    uint32_t sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += analogRead(PIN_VBAT);
        delay(5);
    }
    float raw = (float)(sum / NUM_SAMPLES);

    // Print raw ADC for debugging
    Serial.print("  Raw ADC avg : ");
    Serial.println(raw, 1);

    return raw * VBAT_SCALE;
}

int voltageToPercent(float voltage) {
    if (voltage >= VBAT_MAX) return 100;
    if (voltage <= VBAT_MIN) return 0;

    const float v[] = { 3.0f, 3.5f, 3.6f, 3.7f, 3.85f, 4.2f };
    const float p[] = {   0,   20,   40,   60,    80,   100 };
    const int segments = 5;

    for (int i = 0; i < segments; i++) {
        if (voltage <= v[i + 1]) {
            float t = (voltage - v[i]) / (v[i + 1] - v[i]);
            return (int)(p[i] + t * (p[i + 1] - p[i]));
        }
    }
    return 100;
}

void loop() {
    float voltage = readBatteryVoltage();
    int percent   = voltageToPercent(voltage);

    Serial.print("Battery Voltage : ");
    Serial.print(voltage, 3);
    Serial.println(" V");

    Serial.print("Battery Percent : ");
    Serial.print(percent);
    Serial.println(" %");

    Serial.println("---");
    delay(5000);
}