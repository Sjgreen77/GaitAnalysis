#include <Wire.h>
#include <LSM6DS3.h>

LSM6DS3 imu(I2C_MODE, 0x6A);

// -------------------------------
// TUNING CONSTANTS
// -------------------------------
const float HORIZ_THRESHOLD = 0.12;      // Horizontal energy needed for forward motion
const float GYRO_WALK_THRESHOLD = 15.0;  // Gyro magnitude for swing
const float GYRO_STILL = 3.0;            // Gyro threshold for stance
const float ACC_STILL = 0.05;            // Dynamic accel threshold for stance
const int   STANCE_SAMPLES = 5;          // ~250 ms at 50 Hz
const int   SAMPLE_WINDOW = 20;

// -------------------------------
// GAIT STATE MACHINE
// -------------------------------
enum GaitState { STANCE, SWING };
GaitState gait = STANCE;

unsigned long lastStanceTime = 0;
int stanceCount = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);

  if (imu.begin() != 0) {
    Serial.println("IMU failed to start");
    while (1);
  }

  // Enable step counter
  imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3E);
  imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x40);

  Serial.println("IMU ready. Calibrating...");
}

bool detectStance(float dynX, float dynY, float dynZ, float gx, float gy, float gz) {
  static int stillCount = 0;

  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);
  float accMag  = sqrt(dynX*dynX + dynY*dynY + dynZ*dynZ);

  if (gyroMag < GYRO_STILL && accMag < ACC_STILL) {
    stillCount++;
  } else {
    stillCount = 0;
  }

  return stillCount >= STANCE_SAMPLES;
}

void loop() {
  // -------------------------------
  // 1. Read Raw IMU Data
  // -------------------------------
  float ax = imu.readFloatAccelX();
  float ay = imu.readFloatAccelY();
  float az = imu.readFloatAccelZ();
  float gx = imu.readFloatGyroX();
  float gy = imu.readFloatGyroY();
  float gz = imu.readFloatGyroZ();

  // -------------------------------
  // 2. High-Pass Filter (Remove Gravity)
  // -------------------------------
  static float lpX = 0, lpY = 0, lpZ = 0;
  lpX = (0.9 * lpX) + (0.1 * ax);
  lpY = (0.9 * lpY) + (0.1 * ay);
  lpZ = (0.9 * lpZ) + (0.1 * az);

  float dynX = ax - lpX;
  float dynY = ay - lpY;
  float dynZ = az - lpZ;

  // -------------------------------
  // 3. Motion Metrics
  // -------------------------------
  float horizMag = sqrt(dynX * dynX + dynY * dynY);
  float gyroMag  = sqrt(gx * gx + gy * gy + gz * gz);

  static float horizEnergy = 0;
  horizEnergy = (0.8 * horizEnergy) + (0.2 * horizMag);

  // -------------------------------
  // 4. Step Counter
  // -------------------------------
  uint8_t lo, hi;
  imu.readRegister(&lo, LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
  imu.readRegister(&hi, LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
  uint16_t steps = (hi << 8) | lo;

  static uint16_t prevSteps = 0;
  static unsigned long lastStepTime = 0;
  bool physicalStepDetected = false;

  if (steps != prevSteps) {
    physicalStepDetected = true;
    prevSteps = steps;
    lastStepTime = millis();
  }

  // -------------------------------
  // 5. STANCE DETECTION
  // -------------------------------
  bool stance = detectStance(dynX, dynY, dynZ, gx, gy, gz);

  // -------------------------------
  // 6. GAIT STATE MACHINE
  // -------------------------------
  if (gait == STANCE && !stance) {
    gait = SWING;
  }
  else if (gait == SWING && stance) {
    gait = STANCE;
    stanceCount++;
    lastStanceTime = millis();
  }

  // -------------------------------
  // 7. CADENCE CHECK
  // -------------------------------
  bool validCadence = (millis() - lastStanceTime) < 1200;

  // -------------------------------
  // 8. FINAL CLASSIFICATION
  // -------------------------------
  String state = "STATIONARY";

  bool hasForwardMomentum = horizEnergy > HORIZ_THRESHOLD;
  bool isSwingingLeg = gyroMag > GYRO_WALK_THRESHOLD;
  bool recentlyStepped = (millis() - lastStepTime < 1500);

  bool trueWalking =
      validCadence &&
      stanceCount > 1 &&
      hasForwardMomentum &&
      isSwingingLeg;

  if (trueWalking) {
    if (horizEnergy > 0.4 || gyroMag > 40) {
      state = "RUNNING";
    } else {
      state = "WALKING";
    }
  }
  else if (physicalStepDetected && !hasForwardMomentum) {
    state = "FOOT TAP (Ignored)";
  }
  else if (isSwingingLeg && !stance) {
    state = "FIDGETING / LEG SWING";
  }

  // -------------------------------
  // 9. OUTPUT
  // -------------------------------
  Serial.print("Steps: "); Serial.print(steps);
  Serial.print(" | H-Energy: "); Serial.print(horizEnergy, 2);
  Serial.print(" | Gyro: "); Serial.print(gyroMag, 1);
  Serial.print(" | Stance: "); Serial.print(stance);
  Serial.print(" | STATE: "); Serial.println(state);

  delay(50);
}
