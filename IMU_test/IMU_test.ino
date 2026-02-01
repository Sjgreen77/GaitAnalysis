#include <Wire.h>
#include <LSM6DS3.h>

LSM6DS3 imu(I2C_MODE, 0x6A);   // Correct IMU + correct address

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Starting IMU...");

  int status = imu.begin();
  if (status != 0) {
    Serial.print("IMU failed to start. Code: ");
    Serial.println(status);
    while (1);
  }

  Serial.println("IMU OK!");
}

void loop() {
  float ax = imu.readFloatAccelX();
  float ay = imu.readFloatAccelY();
  float az = imu.readFloatAccelZ();

  float gx = imu.readFloatGyroX();
  float gy = imu.readFloatGyroY();
  float gz = imu.readFloatGyroZ();

  Serial.print("ACCEL  ");
  Serial.print(ax, 3); Serial.print(", ");
  Serial.print(ay, 3); Serial.print(", ");
  Serial.println(az, 3);

  Serial.print("GYRO   ");
  Serial.print(gx, 3); Serial.print(", ");
  Serial.print(gy, 3); Serial.print(", ");
  Serial.println(gz, 3);

  Serial.println();

  delay(50);   // 20 Hz sampling
}
