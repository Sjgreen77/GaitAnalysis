/*
 * XIAO nRF52840 ADC Read Program
 * Reads waveform from Analog Discovery 2 on Pin A0 (D0)
 */
#include <Adafruit_TinyUSB.h>

const int adcPin = A0;    // D0 on the XIAO
float voltage = 0.0;
int rawValue = 0;

void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open (specific to nRF52840)

  // Configure ADC resolution (Default is 10-bit, but nRF52840 supports up to 12-bit)
  analogReadResolution(10); 
}

void loop() {
  // Read the analog value (0 - 1023 for 10-bit)
  rawValue = analogRead(adcPin);

  // Convert raw value to voltage: (reading * Vcc) / resolution
  voltage = (rawValue * 3.3) / 1023.0;

  // Print values for Serial Plotter
  // Format: "Label:Value" or just "Value"
  Serial.print("Raw_Value:");
  Serial.print(rawValue);
  Serial.print(",");
  Serial.print("Voltage_V:");
  Serial.println(voltage);

  // Small delay to prevent flooding the serial buffer
  // Adjust this based on your waveform frequency
  delay(10); 
}
