/*
  Drone Flight Logger - ESP32 firmware

  What it does:
    1. On boot, tries to join WiFi for a few seconds.
    2. If WiFi connects: gets the current time via NTP, uploads any flight
       log left over from the last flight (saved on flash), then deletes it.
    3. Starts a new flight log file on flash (LittleFS) for THIS session.
    4. Reads GPS continuously and appends a line to the log file for every
       new fix, using milliseconds-since-boot as the timestamp (no live
       clock needed while flying).
    5. If WiFi never connects, it just keeps logging locally -- nothing is
       lost. The file uploads next time it boots near WiFi.

  Wiring (NEO-6M / similar GPS module -> ESP32, using UART2):
    GPS TX  -> ESP32 GPIO16 (RX2)
    GPS RX  -> ESP32 GPIO17 (TX2)
    GPS VCC -> 3.3V (check your module's voltage requirement)
    GPS GND -> GND

  Libraries needed (install via Arduino Library Manager):
    - TinyGPSPlus by Mikal Hart
    - ArduinoJson by Benoit Blanchon
    (WiFi, HTTPClient, LittleFS, time.h are built into the ESP32 core)

  Adding an IMU later: add roll/pitch/yaw fields to the JSON object built
  in writeFix() and the payload built in uploadPendingFlight() -- the
  server schema already has columns for them.
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>
#include <time.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "secrets.h"

// ---------- CONFIG ----------
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 8000;
const unsigned long GPS_MIN_INTERVAL_MS     = 200;   // ~5 samples/sec
const unsigned long IMU_SAMPLE_INTERVAL_MS  = 20;    // ~50 samples/sec
// -----------------------------------------

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

HardwareSerial GPSSerial(2);
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
bool imuOk = false;

const char* LOG_FILE_PATH = "/flight_log.jsonl";
unsigned long lastGpsWriteMs = 0;
unsigned long lastImuWriteMs = 0;

// ---------- WiFi / time ----------

bool connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }
  Serial.println("WiFi connect timed out.");
  return false;
}

void syncTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Syncing time");
  time_t now = time(nullptr);
  int tries = 0;
  while (now < 1700000000 && tries < 20) { // wait for a plausible epoch time
    delay(300);
    Serial.print(".");
    now = time(nullptr);
    tries++;
  }
  Serial.println();
}

// ---------- Flight file handling ----------

void startNewFlightLog() {
  File f = LittleFS.open(LOG_FILE_PATH, "w");
  if (f) {
    f.close();
    Serial.println("New flight log started.");
  } else {
    Serial.println("Failed to create flight log file!");
  }
}

void writeGpsFix(unsigned long nowMs) {
  StaticJsonDocument<256> doc;
  doc["type"] = "gps";
  doc["ms"] = nowMs;
  doc["lat"] = gps.location.lat();
  doc["lon"] = gps.location.lng();
  doc["alt"] = gps.altitude.isValid() ? gps.altitude.meters() : (double)NAN;
  doc["sats"] = gps.satellites.isValid() ? gps.satellites.value() : -1;
  doc["hdop"] = gps.hdop.isValid() ? gps.hdop.hdop() : (double)NAN;

  File f = LittleFS.open(LOG_FILE_PATH, "a");
  if (!f) {
    Serial.println("Could not open log file for append.");
    return;
  }
  serializeJson(doc, f);
  f.print("\n");
  f.close();
}

void writeImuSample(unsigned long nowMs) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  StaticJsonDocument<192> doc;
  doc["type"] = "imu";
  doc["ms"] = nowMs;
  doc["ax"] = a.acceleration.x;
  doc["ay"] = a.acceleration.y;
  doc["az"] = a.acceleration.z;
  doc["gx"] = g.gyro.x;
  doc["gy"] = g.gyro.y;
  doc["gz"] = g.gyro.z;

  File f = LittleFS.open(LOG_FILE_PATH, "a");
  if (!f) {
    Serial.println("Could not open log file for append.");
    return;
  }
  serializeJson(doc, f);
  f.print("\n");
  f.close();
}

// ---------- Upload previous flight over WiFi ----------

void uploadPendingFlight() {
  if (!LittleFS.exists(LOG_FILE_PATH)) {
    Serial.println("No pending flight file to upload.");
    return;
  }

  File f = LittleFS.open(LOG_FILE_PATH, "r");
  if (!f || f.size() == 0) {
    if (f) f.close();
    LittleFS.remove(LOG_FILE_PATH);
    return;
  }

  // Build the JSON payload: { drone_name, upload_time, points: [...] }
  DynamicJsonDocument payload(65536); // adjust up if you log long flights at high rate
  payload["drone_name"] = DRONE_NAME;

  time_t nowT = time(nullptr);
  char isoTime[32];
  struct tm tmInfo;
  gmtime_r(&nowT, &tmInfo);
  strftime(isoTime, sizeof(isoTime), "%Y-%m-%dT%H:%M:%SZ", &tmInfo);
  payload["upload_time"] = isoTime;

  JsonArray points = payload.createNestedArray("points");
  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (line.length() < 2) continue;
    StaticJsonDocument<256> point;
    DeserializationError err = deserializeJson(point, line);
    if (!err) {
      points.add(point.as<JsonObject>());
    }
  }
  f.close();

  if (points.size() == 0) {
    Serial.println("Log file had no valid points -- discarding.");
    LittleFS.remove(LOG_FILE_PATH);
    return;
  }

  String body;
  serializeJson(payload, body);

  Serial.printf("Uploading %d points...\n", points.size());

  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-API-Key", API_KEY);
  int status = http.POST(body);

  Serial.printf("HTTP Status: %d\n", status);

  String response = http.getString();
  Serial.println("Server response:");
  Serial.println(response);

  if (status == 200 || status == 201) {
      Serial.println("Upload succeeded. Clearing local log.");
      LittleFS.remove(LOG_FILE_PATH);
  } else {
      Serial.println("Upload failed. Keeping local log.");
  }
  http.end();
}

// ---------- Setup ----------

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed. Halting.");
    while (true) delay(1000);
  }

  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found -- continuing with GPS-only logging.");
    imuOk = false;
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    imuOk = true;
    Serial.println("MPU6050 ready.");
  }

  bool wifiOk = connectWiFi();

  if (wifiOk) {
    syncTime();
    uploadPendingFlight();
  } else {
    Serial.println("No WiFi at boot -- logging locally only.");
  }

  startNewFlightLog();
}

// ---------- Main loop ----------

void loop() {
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }

  unsigned long now = millis();

  if (gps.location.isUpdated() && gps.location.isValid()) {
    if (now - lastGpsWriteMs >= GPS_MIN_INTERVAL_MS) {
      writeGpsFix(now);
      lastGpsWriteMs = now;
    }
  }

  if (imuOk && (now - lastImuWriteMs >= IMU_SAMPLE_INTERVAL_MS)) {
    writeImuSample(now);
    lastImuWriteMs = now;
  }
}