#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <SPI.h>
#include <SD.h>

#define SD_CS 5

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

File dataFile;

void setup() {
  Serial.begin(115200);

  // BNO055 I2C pins
  Wire.begin(21, 22);

  Serial.println("Starting BNO055...");

  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected!");
    while (1) {
      delay(10);
    }
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  // Start SD card
  Serial.println("Starting SD card...");

  if (!SD.begin(SD_CS)) {
    Serial.println("ERROR: SD card initialization failed!");
    while (1) {
      delay(10);
    }
  }

  Serial.println("SD card initialized.");

  // Create CSV header if file doesn't already exist
  if (!SD.exists("/bno055.csv")) {
    dataFile = SD.open("/bno055.csv", FILE_WRITE);

    if (dataFile) {
      dataFile.println(
        "Time_ms,Yaw_deg,Pitch_deg,Roll_deg,"
        "Cal_System,Cal_Gyro,Cal_Accel,Cal_Mag"
      );
      dataFile.close();
    }
  }

  Serial.println("Logging started.");
}

void loop() {

  // Get orientation data
  sensors_event_t orientationData;
  bno.getEvent(&orientationData);

  // Get calibration status
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  // -----------------------
  // Display on Serial Monitor
  // -----------------------

  Serial.print("Heading (Yaw): ");
  Serial.print(orientationData.orientation.x, 2);

  Serial.print(" | Pitch: ");
  Serial.print(orientationData.orientation.y, 2);

  Serial.print(" | Roll: ");
  Serial.print(orientationData.orientation.z, 2);

  Serial.print(" | Calib - Sys:");
  Serial.print(system);

  Serial.print(" G:");
  Serial.print(gyro);

  Serial.print(" A:");
  Serial.print(accel);

  Serial.print(" M:");
  Serial.println(mag);

  // -----------------------
  // Save to microSD
  // -----------------------

  dataFile = SD.open("/bno055.csv", FILE_APPEND);

  if (dataFile) {

    dataFile.print(millis());
    dataFile.print(",");

    dataFile.print(orientationData.orientation.x, 2);
    dataFile.print(",");

    dataFile.print(orientationData.orientation.y, 2);
    dataFile.print(",");

    dataFile.print(orientationData.orientation.z, 2);
    dataFile.print(",");

    dataFile.print(system);
    dataFile.print(",");

    dataFile.print(gyro);
    dataFile.print(",");

    dataFile.print(accel);
    dataFile.print(",");

    dataFile.println(mag);

    dataFile.close();

  } else {
    Serial.println("ERROR: Could not open bno055.csv");
  }

  delay(100);   // 10 Hz logging
}