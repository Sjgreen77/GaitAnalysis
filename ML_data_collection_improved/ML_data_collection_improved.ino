#include <Wire.h>
#include <LSM6DS3.h>
#include "SdFat.h"
#include <SPI.h>
#include "Config.h"

LSM6DS3 imu(I2C_MODE, 0x6A);
SdFat SD;
File dataFile;

const int SAMPLE_INTERVAL_MS = 20;   // 50 Hz
unsigned long lastSampleTime = 0;
unsigned long lastBlinkTime  = 0;
bool ledState = false;
unsigned long sampleCount = 0;

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);

    pinMode(LED_RED,   OUTPUT); digitalWrite(LED_RED,   HIGH);  // off
    pinMode(LED_GREEN, OUTPUT); digitalWrite(LED_GREEN, HIGH);  // off
    pinMode(LED_BLUE,  OUTPUT); digitalWrite(LED_BLUE,  HIGH);  // off

    if (imu.begin() != 0) {
        // Red = IMU error
        digitalWrite(LED_RED, LOW);
        while(1);
    }

    if (!SD.begin(PIN_SD_CS, SPI_HALF_SPEED)) {
        // Blue = SD error  
        digitalWrite(LED_BLUE, LOW);
        while(1);
    }

    if (SD.exists("collect.csv")) SD.remove("collect.csv");
    dataFile = SD.open("collect.csv", FILE_WRITE);
    dataFile.println("ax,ay,az,gx,gy,gz,heel,toe");
    dataFile.close();

    // Solid red for 10 seconds = grace period (get into position)
    digitalWrite(LED_RED, LOW);
    delay(10000);
    digitalWrite(LED_RED, HIGH);

    // Flash green 3 times = starting now!
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_GREEN, LOW);
        delay(150);
        digitalWrite(LED_GREEN, HIGH);
        delay(150);
    }
}

void loop() {
    unsigned long now = millis();

    // Blink green once per second = "I'm alive and logging"
    if (now - lastBlinkTime >= 500) {
        lastBlinkTime = now;
        ledState = !ledState;
        digitalWrite(LED_GREEN, ledState ? LOW : HIGH);  // active low
    }

    if (now - lastSampleTime < SAMPLE_INTERVAL_MS) return;
    lastSampleTime = now;

    float ax = imu.readFloatAccelX();
    float ay = imu.readFloatAccelY();
    float az = imu.readFloatAccelZ();
    float gx = imu.readFloatGyroX();
    float gy = imu.readFloatGyroY();
    float gz = imu.readFloatGyroZ();
    int heel = analogRead(PIN_ADC_HEEL);
    int toe  = analogRead(PIN_ADC_TOE);

    dataFile = SD.open("collect.csv", FILE_WRITE);
    if (dataFile) {
        dataFile.print(ax, 4); dataFile.print(",");
        dataFile.print(ay, 4); dataFile.print(",");
        dataFile.print(az, 4); dataFile.print(",");
        dataFile.print(gx, 4); dataFile.print(",");
        dataFile.print(gy, 4); dataFile.print(",");
        dataFile.print(gz, 4); dataFile.print(",");
        dataFile.print(heel);  dataFile.print(",");
        dataFile.println(toe);
        dataFile.close();
    } else {
        // SD write failed — solid red
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_RED, LOW);
    }

    sampleCount++;
}