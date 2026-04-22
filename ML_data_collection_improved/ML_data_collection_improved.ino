#include <Wire.h>
#include <LSM6DS3.h>
#include "Config.h"

LSM6DS3 imu(I2C_MODE, 0x6A);

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    if (imu.begin() != 0) {
        Serial.println("IMU Error!");
        while(1);
    }
    // Print header so MATLAB knows column order
    Serial.println("ax,ay,az,gx,gy,gz,heel,toe");
}

void loop() {
    float ax = imu.readFloatAccelX();
    float ay = imu.readFloatAccelY();
    float az = imu.readFloatAccelZ();
    float gx = imu.readFloatGyroX();
    float gy = imu.readFloatGyroY();
    float gz = imu.readFloatGyroZ();
    int heel = analogRead(PIN_ADC_HEEL);
    int toe  = analogRead(PIN_ADC_TOE);

    Serial.print(ax,4); Serial.print(",");
    Serial.print(ay,4); Serial.print(",");
    Serial.print(az,4); Serial.print(",");
    Serial.print(gx,4); Serial.print(",");
    Serial.print(gy,4); Serial.print(",");
    Serial.print(gz,4); Serial.print(",");
    Serial.print(heel); Serial.print(",");
    Serial.println(toe);

    delay(20); // 50 Hz
}