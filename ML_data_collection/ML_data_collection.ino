#include <Wire.h>
#include <LSM6DS3.h>

LSM6DS3 imu(I2C_MODE, 0x6A);

const char* LABEL = "fidgeting";   
const unsigned long SAMPLE_PERIOD_MS = 20;      // 50 Hz
const unsigned long RECORD_DURATION_MS = 60000; // 1 minute

unsigned long lastSampleTime = 0;
unsigned long startTime = 0;
bool recording = true;

void setup() {
  Serial.begin(115200);
  delay(2000);

  if (imu.begin() != 0) {
    while (1);
  }

  startTime = millis();

  Serial.println("label,ax,ay,az,gx,gy,gz");   // CSV header
}

void loop() {
  if (!recording) {
    return;  // Stop everything after recording ends
  }

  unsigned long currentTime = millis();

  // Stop after 3 minutes
  if (currentTime - startTime >= RECORD_DURATION_MS) {
    recording = false;
    Serial.println("END_OF_RECORDING");
    while (1);   // Halt program
  }

  // Maintain fixed sample rate
  if (currentTime - lastSampleTime >= SAMPLE_PERIOD_MS) {
    lastSampleTime = currentTime;

    float ax = imu.readFloatAccelX();
    float ay = imu.readFloatAccelY();
    float az = imu.readFloatAccelZ();

    float gx = imu.readFloatGyroX();
    float gy = imu.readFloatGyroY();
    float gz = imu.readFloatGyroZ();

    Serial.print(LABEL); Serial.print(",");
    Serial.print(ax, 6); Serial.print(",");
    Serial.print(ay, 6); Serial.print(",");
    Serial.print(az, 6); Serial.print(",");
    Serial.print(gx, 6); Serial.print(",");
    Serial.print(gy, 6); Serial.print(",");
    Serial.println(gz, 6);
  }
}
