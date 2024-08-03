#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Define SDA and SCL pins explicitly
#define SDA_PIN 4  // D2
#define SCL_PIN 5  // D1

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const int calibration_offset = 2; // Adjust this value based on your testing
const int num_samples = 10; // Number of samples to take for averaging

void setup() {
  Serial.begin(115200);  // Initialize serial communication at 115200 baud rate
  Wire.begin(SDA_PIN, SCL_PIN);    // Initialize I2C communication with SDA and SCL

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  Serial.println(F("VL53L0X Ready"));
}

void loop() {
  long sum = 0;
  int valid_samples = 0;

  for (int i = 0; i < num_samples; i++) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);  // Perform a ranging test

    if (measure.RangeStatus != 4) {  // Check if measurement is valid
      int distance_mm = measure.RangeMilliMeter;
      int calibrated_distance_mm = distance_mm - calibration_offset;  // Apply manual offset
      sum += calibrated_distance_mm;
      valid_samples++;
    }

    delay(100);  // Wait for 100 milliseconds between samples
  }

  if (valid_samples > 0) {
    int average_distance = sum / valid_samples;
    Serial.print("Average Distance (mm): ");
    Serial.println(average_distance);  // Print average distance in mm
  } else {
    Serial.println("No valid samples collected");
  }

  delay(1000);  // Wait for 1 second before the next set of readings
}
