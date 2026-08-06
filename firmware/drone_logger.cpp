/*
  Drone Flight Logger - ESP32 firmware

  What it does:
    1. On boot, tries to join WiFi for a few seconds.
    2. If WiFi connects: gets the current time via NTP, uploads any flight
       log left over from the last flight (saved on flash), then deletes it.
    3. Waits idle for the BOOT button (GPIO0) to be pressed. Nothing is
       logged until then -- this avoids wasting the WiFi-timeout / NTP
       window at the start of your actual flight log.
    4. On button press: starts a new flight log file on flash (LittleFS)
       and turns the LED on as an explicit "logging started" confirmation.
    5. Reads GPS continuously and appends a line to the log file for every
       new fix, using milliseconds-since-boot as the timestamp (no live
       clock needed while flying).
    6. If WiFi never connects, it just keeps logging locally -- nothing is
       lost. The file uploads next time it boots near WiFi.

  Onboard LED:
    Turns on solid the moment logging starts (button press). After that,
    it flashes once per data point written to the flight log (GPS fix or
    IMU sample), so you get a visual heartbeat confirming logging is
    actively happening. If it's solid-off, logging hasn't been started
    yet (waiting for button); if it never flashes after button press,
    the log file isn't being written to.

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
  server schema already has columns for them. [DONE -- see Orientation note below.]

  Orientation (roll/pitch/yaw):
    Computed onboard via a Madgwick AHRS filter (quaternion-based, IMU-only
    since there's no magnetometer -- yaw is relative, not a true heading,
    and will drift slowly since nothing external corrects it). Gyro bias
    is calibrated at boot (board must be stationary during that ~5s
    window) and continues adapting slowly during flight whenever the
    board is detected as stationary, to track thermal drift.
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
#include <math.h>
#include "secrets.h"

// ---------- CONFIG ----------
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 8000;
const unsigned long GPS_MIN_INTERVAL_MS     = 200;   // ~5 samples/sec
const unsigned long IMU_SAMPLE_INTERVAL_MS  = 20;    // ~50 samples/sec
const unsigned long LOG_LED_FLASH_MS        = 15;    // how long the LED stays on per flash
// -----------------------------------------

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

// Most ESP32 dev boards (including the common "ESP32 DevKit" ones) have
// a built-in LED on GPIO2. If yours doesn't light up, check your board's
// pinout -- some boards use a different pin or have no onboard LED at all,
// in which case wire an external LED + resistor to this pin instead.
#define LOG_LED_PIN 2

// Most ESP32 dev boards have a "BOOT" button wired to GPIO0, active LOW
// (pressed = LOW) with an onboard pull-up already present. INPUT_PULLUP
// is set anyway as cheap insurance for boards where it isn't.
#define BOOT_BUTTON_PIN 0
const unsigned long BUTTON_DEBOUNCE_MS = 50;

HardwareSerial GPSSerial(2);
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
bool imuOk = false;

// ---------- Madgwick AHRS (IMU-only, no magnetometer) ----------
// Quaternion-based orientation filter. beta = filter gain: higher trusts
// the accelerometer more (faster correction, more jitter under
// vibration); lower trusts the gyro more (smoother, drifts longer).
class MadgwickFilter {
public:
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
  float beta = 0.1f;

  void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
      ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

      _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
      _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2;
      _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
      q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3;

      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay
           - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
           - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

      recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
    }

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
  }

  float getRoll() {
    return atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
  }
  float getPitch() {
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) return copysignf(90.0f, sinp);
    return asinf(sinp) * 180.0f / PI;
  }
  float getYaw() {
    return atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;
  }
};

MadgwickFilter orientationFilter;
unsigned long lastFilterUpdateMs = 0;

// Gyro zero-rate bias (rad/s). Seeded by startup calibration, then
// continuously refined whenever the board is detected as stationary
// (tracks thermal drift that a one-shot calibration can't).
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;

const float STATIONARY_GYRO_THRESH = 0.017f;  // ~1 deg/s in rad/s
const float STATIONARY_ACCEL_MIN   = 9.5f;    // m/s^2, expect ~9.8 at rest
const float STATIONARY_ACCEL_MAX   = 10.1f;
const float BIAS_LEARN_RATE        = 0.002f;  // slow EMA -- don't chase real motion
const float GYRO_DEADBAND          = 0.003f;  // ~0.17 deg/s in rad/s, kills residual creep

const char* LOG_FILE_PATH = "/flight_log.jsonl";
unsigned long lastGpsWriteMs = 0;
unsigned long lastImuWriteMs = 0;

// Logging only actually starts once the boot button is pressed -- avoids
// wasting the WiFi-timeout / NTP-sync window at the start of the flight log.
bool loggingActive = false;
bool lastButtonState = HIGH; // HIGH = not pressed (pull-up)
unsigned long lastButtonChangeMs = 0;

// LED flash state -- non-blocking, so it never slows down GPS/IMU sampling.
bool logLedOn = false;
unsigned long logLedOnSinceMs = 0;

// ---------- Onboard LED ----------

void flashLogLed() {
  digitalWrite(LOG_LED_PIN, HIGH);
  logLedOn = true;
  logLedOnSinceMs = millis();
}

void serviceLogLed() {
  if (logLedOn && (millis() - logLedOnSinceMs >= LOG_LED_FLASH_MS)) {
    digitalWrite(LOG_LED_PIN, LOW);
    logLedOn = false;
  }
}

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

// ---------- Orientation filter helpers ----------

void calibrateGyro() {
  const int samples = 500; // ~5s at 10ms per sample
  double sumX = 0, sumY = 0, sumZ = 0;

  Serial.println("Calibrating gyro bias -- keep the board still...");

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(10);
  }

  gyroBiasX = sumX / samples;
  gyroBiasY = sumY / samples;
  gyroBiasZ = sumZ / samples;

  Serial.print("Initial gyro bias (rad/s) - X: "); Serial.print(gyroBiasX, 5);
  Serial.print("  Y: "); Serial.print(gyroBiasY, 5);
  Serial.print("  Z: "); Serial.println(gyroBiasZ, 5);
}

// Call every IMU sample with raw (un-bias-corrected) readings. Detects
// stationary periods and slowly nudges the bias estimate, so slow
// in-flight thermal drift gets tracked out over time.
void updateGyroBias(sensors_event_t &a, sensors_event_t &g) {
  float accelMag = sqrtf(a.acceleration.x * a.acceleration.x +
                          a.acceleration.y * a.acceleration.y +
                          a.acceleration.z * a.acceleration.z);

  bool accelStill = (accelMag > STATIONARY_ACCEL_MIN && accelMag < STATIONARY_ACCEL_MAX);
  bool gyroStill  = (fabsf(g.gyro.x - gyroBiasX) < STATIONARY_GYRO_THRESH &&
                      fabsf(g.gyro.y - gyroBiasY) < STATIONARY_GYRO_THRESH &&
                      fabsf(g.gyro.z - gyroBiasZ) < STATIONARY_GYRO_THRESH);

  if (accelStill && gyroStill) {
    gyroBiasX += BIAS_LEARN_RATE * (g.gyro.x - gyroBiasX);
    gyroBiasY += BIAS_LEARN_RATE * (g.gyro.y - gyroBiasY);
    gyroBiasZ += BIAS_LEARN_RATE * (g.gyro.z - gyroBiasZ);
  }
}

// ---------- Boot button / logging start ----------

// Returns true exactly once, on the loop iteration where a debounced
// press is detected.
bool checkBootButtonPressed() {
  bool reading = digitalRead(BOOT_BUTTON_PIN);
  unsigned long now = millis();

  if (reading != lastButtonState) {
    lastButtonChangeMs = now; // state just changed, restart debounce window
  }

  bool pressed = false;
  if ((now - lastButtonChangeMs) > BUTTON_DEBOUNCE_MS) {
    // Debounced: LOW means currently pressed. Only fire on the falling edge.
    if (reading == LOW && lastButtonState == HIGH) {
      pressed = true;
    }
  }

  lastButtonState = reading;
  return pressed;
}

void startLogging() {
  startNewFlightLog();

  // Reset write timers so we don't get a burst of "overdue" writes on
  // the first loop iteration after starting.
  unsigned long now = millis();
  lastGpsWriteMs = now;
  lastImuWriteMs = now;
  lastFilterUpdateMs = now;

  loggingActive = true;

  // Explicit "logging started" confirmation -- solid on immediately,
  // independent of the per-data-point heartbeat flash. The next
  // flashLogLed() call (on the first GPS/IMU write) will take over
  // the normal blip-per-point behavior from here.
  digitalWrite(LOG_LED_PIN, HIGH);

  Serial.println("Boot button pressed -- logging started.");
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

  flashLogLed();
}

void writeImuSample(unsigned long nowMs) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // --- Update orientation filter ---
  unsigned long filterNow = millis();
  float dt = (filterNow - lastFilterUpdateMs) / 1000.0f;
  lastFilterUpdateMs = filterNow;

  if (dt > 0 && dt < 0.5f) { // guard against stalls/first-run
    updateGyroBias(a, g);

    float gx = g.gyro.x - gyroBiasX;
    float gy = g.gyro.y - gyroBiasY;
    float gz = g.gyro.z - gyroBiasZ;

    if (fabsf(gx) < GYRO_DEADBAND) gx = 0.0f;
    if (fabsf(gy) < GYRO_DEADBAND) gy = 0.0f;
    if (fabsf(gz) < GYRO_DEADBAND) gz = 0.0f;

    orientationFilter.update(gx, gy, gz,
                              a.acceleration.x, a.acceleration.y, a.acceleration.z, dt);
  }

  StaticJsonDocument<256> doc;
  doc["type"] = "imu";
  doc["ms"] = nowMs;
  doc["ax"] = a.acceleration.x;
  doc["ay"] = a.acceleration.y;
  doc["az"] = a.acceleration.z;
  doc["gx"] = g.gyro.x;
  doc["gy"] = g.gyro.y;
  doc["gz"] = g.gyro.z;
  doc["roll"] = orientationFilter.getRoll();
  doc["pitch"] = orientationFilter.getPitch();
  doc["yaw"] = orientationFilter.getYaw();

  File f = LittleFS.open(LOG_FILE_PATH, "a");
  if (!f) {
    Serial.println("Could not open log file for append.");
    return;
  }
  serializeJson(doc, f);
  f.print("\n");
  f.close();

  flashLogLed();
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
  unsigned long firstMs = 0;
  unsigned long lastMs = 0;
  bool haveRange = false;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (line.length() < 2) continue;
    StaticJsonDocument<320> point;
    DeserializationError err = deserializeJson(point, line);
    if (!err) {
      points.add(point.as<JsonObject>());

      unsigned long pointMs = point["ms"].as<unsigned long>();
      if (!haveRange) {
        firstMs = pointMs;
        lastMs = pointMs;
        haveRange = true;
      } else {
        if (pointMs < firstMs) firstMs = pointMs;
        if (pointMs > lastMs) lastMs = pointMs;
      }
    }
  }
  f.close();

  if (points.size() == 0) {
    Serial.println("Log file had no valid points -- discarding.");
    LittleFS.remove(LOG_FILE_PATH);
    return;
  }

  // Server requires these to anchor each point's real timestamp against
  // upload_time -- see FlightUpload model in backend/app.py.
  payload["flight_first_ms"] = firstMs;
  payload["flight_last_ms"] = lastMs;

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

  pinMode(LOG_LED_PIN, OUTPUT);
  digitalWrite(LOG_LED_PIN, LOW);

  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

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

    // Board must be stationary during this ~5s window.
    calibrateGyro();
    lastFilterUpdateMs = millis();
  }

  bool wifiOk = connectWiFi();

  if (wifiOk) {
    syncTime();
    uploadPendingFlight();
  } else {
    Serial.println("No WiFi at boot -- logging locally only.");
  }

  Serial.println("Ready. Press the BOOT button to start logging.");
}

// ---------- Main loop ----------

void loop() {
  if (!loggingActive) {
    if (checkBootButtonPressed()) {
      startLogging();
    }
    return; // idle -- nothing else to do until logging starts
  }

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

  serviceLogLed();
}