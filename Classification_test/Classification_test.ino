#include <Wire.h>
#include <LSM6DS3.h>
#include <math.h>

LSM6DS3 imu(I2C_MODE, 0x6A); 

// --- Constants for Sampling ---
const int SAMPLE_RATE_HZ = 50;
const unsigned long SAMPLE_INTERVAL_MS = 1000 / SAMPLE_RATE_HZ; // 20ms interval
const int WINDOW_SIZE = 50; // 1 second window at 50Hz

// --- Circular Buffers for required features ---
float ay_buffer[WINDOW_SIZE];
float gy_buffer[WINDOW_SIZE];

int insert_index = 0;
bool buffer_full = false;
unsigned long last_sample_time = 0;

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

  Serial.println("IMU OK! Filling initial 1-second buffer...");
}

void loop() {
  unsigned long current_time = millis();

  // 1. Sample at precisely 50 Hz
  if (current_time - last_sample_time >= SAMPLE_INTERVAL_MS) {
    last_sample_time = current_time;

    // Read IMU and store in circular buffer
    ay_buffer[insert_index] = imu.readFloatAccelY();
    gy_buffer[insert_index] = imu.readFloatGyroY();
    
    insert_index++; // Move the pointer forward
    
    // Check if we've hit the end of the array
    if (insert_index >= WINDOW_SIZE) {
      buffer_full = true; // We now have a full 1 second of data
      insert_index = 0;   // Wrap around to the start (Circular Buffer magic)
    }

    // 2. Process window EVERY time a new sample is added (once buffer is full)
    if (buffer_full) {
      extractFeaturesAndClassify();
    }
  }
}

void extractFeaturesAndClassify() {
  // Calculate Mean for AY
  float sum_ay = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    sum_ay += ay_buffer[i];
  }
  float mean_ay = sum_ay / WINDOW_SIZE;

  // Calculate Mean for GY
  float sum_gy = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    sum_gy += gy_buffer[i];
  }
  float mean_gy = sum_gy / WINDOW_SIZE;

  // Calculate Standard Deviation for AY and GY (Using N-1)
  float sum_sq_ay = 0;
  float sum_sq_gy = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    sum_sq_ay += pow(ay_buffer[i] - mean_ay, 2);
    sum_sq_gy += pow(gy_buffer[i] - mean_gy, 2);
  }
  float std_ay = sqrt(sum_sq_ay / (WINDOW_SIZE - 1));
  float std_gy = sqrt(sum_sq_gy / (WINDOW_SIZE - 1));

  // --- DECISION TREE LOGIC ---
  // Note: I removed the raw value printouts so your Serial Monitor 
  // doesn't become an unreadable blur at 50 updates per second!
  if (std_ay < 0.0326435) {
    Serial.println("CLASS: STATIONARY");
  } else {
    if (mean_ay < -0.213568) {
      Serial.println("CLASS: WALKING");
    } else {
      if (std_gy < 60.6775) {
        Serial.println("CLASS: FIDGETING");
      } else {
        Serial.println("CLASS: WALKING");
      }
    }
  }
}