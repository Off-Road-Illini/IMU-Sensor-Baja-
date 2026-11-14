#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <TinyGPS++.h>
#include <HardwareSerial.h>

#include <SPI.h>
#include <SD.h>

// ---------- PIN CONFIGURATION ----------

#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600          // what the GP-02 starts at
#define GPS_FAST_BAUD 115200   // what we switch it to, so 10 updates a second fit down the wire

#define SD_CS 5

#define BUTTON_PIN 25
#define RED_LED_PIN 33
#define GREEN_LED_PIN 32
#define YELLOW_LED_PIN 4

// ---------- LOGGING SETTINGS ----------

const unsigned long LOG_INTERVAL = 10;   // 100 Hz
const unsigned long FLUSH_INTERVAL = 1000;

const char* LOG_FILE = "/November_14_Testing.csv";

// ---------- OBJECTS ----------

Adafruit_BNO055 bno(55, 0x28);

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

File dataFile;

unsigned long lastLog = 0;
unsigned long lastFlush = 0;
unsigned long lastStatus = 0;
unsigned long lastButton = 0;

bool newGPSPosition = false;
unsigned long rowsWritten = 0;

int buttonState = 1;
int prevButtonState = 1;
bool recording = false;

// ---------- READ GPS CONTINUOUSLY ----------

void readGPS() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();

    if (gps.encode(c)) {
      if (gps.location.isUpdated()) {
        newGPSPosition = true;
      }
    }
  }
}

// ---------- GPS SETUP (GP-02 / AT6558R chip, "$PCAS" commands) ----------

// Sends one setup command with its checksum, e.g. "PCAS02,100" -> "$PCAS02,100*1E"
void sendGPSCommand(const char* body) {
  uint8_t checksum = 0;
  for (const char* p = body; *p; p++) {
    checksum ^= *p;
  }
  gpsSerial.printf("$%s*%02X\r\n", body, checksum);
  gpsSerial.flush();
}

// True if a complete, valid GPS sentence arrives within the time given
bool gpsTalking(unsigned long ms) {
  uint32_t before = gps.passedChecksum();
  unsigned long start = millis();
  while (millis() - start < ms) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    if (gps.passedChecksum() > before) {
      return true;
    }
    delay(1);
  }
  return false;
}

void setupGPS() {
  gpsSerial.setRxBufferSize(1024);   // bigger receive buffer so GPS text isn't lost while the SD card is busy
  gpsSerial.begin(GPS_FAST_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  // The GPS keeps its settings while it has power, so it may already be fast from last time
  bool fast = gpsTalking(1500);

  if (!fast) {
    gpsSerial.updateBaudRate(GPS_BAUD);
    if (!gpsTalking(2500)) {
      Serial.println("GPS: no data. Check GPS TX -> ESP32 pin 16 and power.");
      return;
    }
    sendGPSCommand("PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0");   // only send GGA + RMC (all TinyGPS++ needs)
    delay(100);
    sendGPSCommand("PCAS01,5");                           // switch the GPS to 115200 baud
    delay(200);
    gpsSerial.updateBaudRate(GPS_FAST_BAUD);
    fast = gpsTalking(2500);
  }

  if (fast) {
    sendGPSCommand("PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0");
    delay(50);
    sendGPSCommand("PCAS02,100");                         // a new position every 100 ms = 10 Hz
    Serial.println("GPS: 10 Hz at 115200 baud.");
  } else {
    // The GPS didn't take the new speed: stay at 9600, where 5 Hz still fits
    gpsSerial.updateBaudRate(GPS_BAUD);
    gpsTalking(1500);
    sendGPSCommand("PCAS02,200");                         // 5 Hz
    Serial.println("GPS: couldn't switch to 115200 (is ESP32 pin 17 wired to the GPS RX pin?). Using 5 Hz at 9600 baud.");
  }
}

// ---------- GPS UTC TIME ----------

void printGPSTime(File &file) {
  if (gps.date.isValid() && gps.time.isValid()
      && gps.time.age() < 5000) {

    char utc[32];

    snprintf(
      utc, sizeof(utc),
      "%04d-%02d-%02dT%02d:%02d:%02d.%02dZ",
      gps.date.year(),
      gps.date.month(),
      gps.date.day(),
      gps.time.hour(),
      gps.time.minute(),
      gps.time.second(),
      gps.time.centisecond()
    );

    file.print(utc);
  }
}

// ---------- SETUP ----------

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting combined sensor logger...");

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // GPS UART: finds the GPS, then sets it to 10 Hz
  setupGPS();

  // BNO055 I2C
  Wire.begin(21, 22);

  Serial.println("Starting BNO055...");

  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected!");
    while (true) {
      delay(100);
    }
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("BNO055 initialized.");

  // SD card SPI
  SPI.begin(18, 19, 23, SD_CS);

  Serial.println("Starting SD card...");

  delay(100);

  if (!SD.begin(SD_CS)) {
    Serial.println("ERROR: SD card initialization failed!");
    while (true) {
      delay(100);
    }
  }

  Serial.println("SD card initialized.");

  // Open CSV file
  dataFile = SD.open(LOG_FILE, FILE_APPEND);

  if (!dataFile) {
    Serial.println("ERROR: Cannot open CSV file!");
    while (true) {
      delay(100);
    }
  }

  // Write header only if file is empty
  if (dataFile.size() == 0) {
    dataFile.println(
      "Time_ms,Yaw_deg,Pitch_deg,Roll_deg,Cal_System,Cal_Gyro,Cal_Accel,Cal_Mag,Latitude,Longitude,Altitude_m,Speed_kmh,Satellites,GPS_Age_ms,GPS_New,GPS_UTC"
    );

    dataFile.flush();
  } else {
    dataFile.println(
      "---,---,---,---,---,---,---,---,---,---,---,---,---,---,---,---"
    );

    dataFile.flush();
  }

  lastLog = millis();
  lastFlush = lastLog;
  lastStatus = lastLog;

  Serial.println("Ready. Press the button to start logging at 100 Hz.");
}

// ---------- LOG ONE ROW ----------

void logSensorData(unsigned long timestamp) {

  // Read orientation
  sensors_event_t orientationData;
  bno.getEvent(&orientationData);

  // Read calibration
  uint8_t system = 0;
  uint8_t gyro = 0;
  uint8_t accel = 0;
  uint8_t mag = 0;

  bno.getCalibration(
    &system,
    &gyro,
    &accel,
    &mag
  );

  // Use one timestamp for this row
  dataFile.print(timestamp);
  dataFile.print(",");

  // BNO055 orientation
  dataFile.print(orientationData.orientation.x, 2);
  dataFile.print(",");

  dataFile.print(orientationData.orientation.y, 2);
  dataFile.print(",");

  dataFile.print(orientationData.orientation.z, 2);
  dataFile.print(",");

  // Calibration
  dataFile.print(system);
  dataFile.print(",");

  dataFile.print(gyro);
  dataFile.print(",");

  dataFile.print(accel);
  dataFile.print(",");

  dataFile.print(mag);
  dataFile.print(",");

  // GPS position
  if (gps.location.isValid() &&
      gps.location.age() < 5000) {

    dataFile.print(gps.location.lat(), 6);
    dataFile.print(",");

    dataFile.print(gps.location.lng(), 6);
    dataFile.print(",");

  } else {
    dataFile.print(",,");
  }

  // GPS altitude
  if (gps.altitude.isValid() &&
      gps.altitude.age() < 5000) {
    dataFile.print(gps.altitude.meters(), 2);
  }

  dataFile.print(",");

  // GPS speed
  if (gps.speed.isValid() &&
      gps.speed.age() < 5000) {
    dataFile.print(gps.speed.kmph(), 2);
  }

  dataFile.print(",");

  // Satellite count
  if (gps.satellites.isValid() &&
      gps.satellites.age() < 5000) {
    dataFile.print(gps.satellites.value());
  }

  dataFile.print(",");

  // Age of latest GPS position
  if (gps.location.isValid()) {
    dataFile.print(gps.location.age());
  }

  dataFile.print(",");

  // Did a new GPS position arrive?
  dataFile.print(newGPSPosition ? 1 : 0);
  dataFile.print(",");

  // Latest GPS UTC timestamp
  printGPSTime(dataFile);

  dataFile.println();

  rowsWritten++;
  newGPSPosition = false;
}

// ---------- MAIN LOOP ----------

void loop() {

  // Always receive incoming GPS data
  readGPS();

  unsigned long now = millis();

  // Check the button and LEDs every 10 ms
  if (now - lastButton >= 10){
    lastButton = now;
    buttonState = digitalRead(BUTTON_PIN);

    if((buttonState == 0) && (prevButtonState == 1)){
      if(recording){
        recording = false;
        digitalWrite(YELLOW_LED_PIN, LOW);

        dataFile.println(
          "---,---,---,---,---,---,---,---,---,---,---,---,---,---,---,---"
        );

        dataFile.flush();

      } else {
        recording = true;
        digitalWrite(YELLOW_LED_PIN, HIGH);
      }
    }
  

    if (gps.speed.isValid()){
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(GREEN_LED_PIN, HIGH);
    } else {
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(GREEN_LED_PIN, LOW);
    }
  }

  prevButtonState = buttonState;

  if (recording){
    // Log at 100 Hz (one row every 10 ms, evenly spaced)
    if (now - lastLog >= LOG_INTERVAL) {
      lastLog += LOG_INTERVAL;

      // If a slow SD write made us fall behind, skip ahead instead of writing a burst of rows
      if (now - lastLog >= LOG_INTERVAL) {
        lastLog = now;
      }

      logSensorData(now);
    }

    // Flush buffered SD data every second
    if (millis() - lastFlush >= FLUSH_INTERVAL) {
      dataFile.flush();
      lastFlush = millis();
    }

  // Print status every second
    if (millis() - lastStatus >= 1000) {
      lastStatus = millis();

      Serial.print("Rows: ");
      Serial.print(rowsWritten);

      Serial.print(" | GPS chars: ");
      Serial.print(gps.charsProcessed());

      Serial.print(" | GPS fix: ");
      Serial.print(gps.location.isValid() ? "YES" : "NO");

      Serial.print(" | Satellites: ");
      Serial.print(gps.satellites.value());

      Serial.print(" | Heading: ");
      Serial.println(bno.getVector(
        Adafruit_BNO055::VECTOR_EULER
      ).x(), 2);
    }
  }
}