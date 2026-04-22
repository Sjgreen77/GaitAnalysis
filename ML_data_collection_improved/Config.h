#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- Hardware Pins ---
const int PIN_ADC_HEEL = A0;
const int PIN_ADC_TOE  = A1;
const int PIN_SD_CS    = TX; // Verify this matches your wiring

// --- Sampling & Buffers ---
const int SAMPLE_RATE_HZ = 50;
//const unsigned long SAMPLE_INTERVAL_MS = 1000 / SAMPLE_RATE_HZ;
const int WINDOW_SIZE = 50; // 1 second window

// --- Battery ---
#define VBAT_ENABLE       14
#define PIN_VBAT          35
#define VREF              3.6f
#define R_UPPER           1000.0f   // 1M ohm
#define R_LOWER           510.0f    // 510k ohm
#define VBAT_SCALE        (VREF / 4096.0f * (R_UPPER + R_LOWER) / R_LOWER)
#define VBAT_MIN          3.0f
#define VBAT_MAX          4.2f
const unsigned long BATTERY_INTERVAL_MS = 1000; // Report battery every 1 second

// --- Thresholds ---
// Replace these with actual ADC values determined from your testing
const int FORCE_THRESHOLD = 500;

// --- BLE UUIDs ---
#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define CHAR_UUID    "12345678-1234-5678-1234-56789abcdef1"

#endif