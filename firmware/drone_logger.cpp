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
    6. On a SECOND button press (while a flight is actively being logged):
       stops logging, turns the log LED off, and tries to upload right
       away over the still-open WiFi connection from boot -- rather than
       requiring a reboot to trigger the upload. Falls back to "kept for
       next boot" (3 UPLOAD_LED flashes) if WiFi isn't connected.
    7. If WiFi never connects, it just keeps logging locally -- nothing is
       lost. The file uploads next time it boots near WiFi.

  Status LEDs:
    - CALIBRATE_LED (GPIO32): on solid while the gyro is being calibrated
      at boot. Off once calibration finishes.
    - UPLOAD_LED (GPIO33): on solid while a pending flight log is actually
      being uploaded over WiFi. When the upload finishes it flashes once
      for success, or three times for any kind of failure (HTTP error,
      no WiFi, etc).
    - READY_LED (GPIO25): reflects live GPS/IMU status while waiting for
      the BOOT button (rechecked every loop, not just once at boot):
        * off (not blinking)         -> no valid GPS fix yet. The button
                                         is ignored in this state.
        * slow blink (~1.25Hz cycle) -> valid GPS fix, no IMU (or IMU
                                         gone invalid). Safe to press the
                                         button and log GPS-only.
        * fast blink (~4Hz cycle)    -> valid GPS fix AND valid IMU
                                         data. Everything's good to go.
      A GPS fix is required before a button press actually starts
      logging -- if you press it while the LED is off, it's ignored
      and logged to Serial. The moment logging does start, this (and
      the other two status LEDs, defensively) turn off.
    - LOG_LED (GPIO2, the usual onboard LED on most dev boards): heartbeat
      for "logging is actively writing data." Rather than flashing once
      per GPS/IMU sample (GPS fixes come every ~200ms which is borderline,
      but IMU samples come every ~20ms -- far too fast for a flash to be
      visible, or in some cases even to fully turn off before the next
      write requests it on again), it now toggles on a fixed, easily
      visible interval (LOG_LED_BLINK_INTERVAL_MS) but ONLY toggles if at
      least one data point was actually written during that interval.
      So: steady blinking = data is being logged normally. If it stops
      blinking (goes dark) while still in a flight, that's your signal
      that writes have stalled -- e.g. no GPS fix and no IMU.

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
const unsigned long WIFI_CONNECT_TIMEOUT_MS   = 8000;
const unsigned long GPS_MIN_INTERVAL_MS       = 200;   // ~5 samples/sec
const unsigned long IMU_SAMPLE_INTERVAL_MS    = 20;    // ~50 samples/sec
const unsigned long LOG_LED_BLINK_INTERVAL_MS = 400;   // heartbeat toggle period, chosen to be clearly visible
const int UPLOAD_BATCH_SIZE                   = 150;   // points per upload request, keeps each JsonDocument/String small
const unsigned long STATUS_LED_FLASH_ON_MS    = 150;
const unsigned long STATUS_LED_FLASH_OFF_MS   = 150;
const unsigned long GPS_STALE_MS              = 3000;  // fix older than this counts as "not valid" anymore
const unsigned long IMU_VALIDITY_CHECK_INTERVAL_MS = 200; // how often to re-sanity-check the IMU while idle
const unsigned long READY_LED_BLINK_SLOW_MS   = 500;   // GPS valid, IMU not (or not installed)
const unsigned long READY_LED_BLINK_FAST_MS   = 120;   // GPS + IMU both valid -- good to go
// -----------------------------------------

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

#define CALIBRATE_LED 32
#define UPLOAD_LED 33
#define READY_LED 25

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
bool lastRawButtonReading = HIGH;  // most recent raw (unfiltered) pin reading
bool debouncedButtonState = HIGH;  // confirmed/stable state, HIGH = not pressed
unsigned long lastButtonChangeMs = 0;

// True once setup() has fully finished (calibration + WiFi/upload attempt
// both done). The BOOT button is only honored while this is true, and
// only actually starts logging once gpsValid is also true (see loop()).
bool systemReady = false;

// ---------- Idle-state GPS / IMU validity tracking ----------
// Continuously rechecked (not just at boot) so READY_LED reflects the
// current state of both sensors, not a one-time snapshot.
bool imuValid = false;
unsigned long lastImuValidityCheckMs = 0;

// Re-checks IMU sanity every IMU_VALIDITY_CHECK_INTERVAL_MS. Catches a
// sensor that was detected at boot but has since gone dead/disconnected
// (which typically reads back all-zero or garbage), without requiring
// the board to be held still.
void updateImuValidity() {
  if (!imuOk) {
    imuValid = false;
    return;
  }

  unsigned long now = millis();
  if (now - lastImuValidityCheckMs < IMU_VALIDITY_CHECK_INTERVAL_MS) {
    return;
  }
  lastImuValidityCheckMs = now;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  bool finiteReadings = isfinite(a.acceleration.x) && isfinite(a.acceleration.y) && isfinite(a.acceleration.z) &&
                         isfinite(g.gyro.x) && isfinite(g.gyro.y) && isfinite(g.gyro.z);

  float accelMag = sqrtf(a.acceleration.x * a.acceleration.x +
                          a.acceleration.y * a.acceleration.y +
                          a.acceleration.z * a.acceleration.z);
  // Loose sanity range around gravity -- wide enough to allow for the
  // board being handled/moved while you wait, tight enough to catch a
  // dead sensor reading flat zero.
  bool plausibleMagnitude = (accelMag > 1.0f && accelMag < 30.0f);

  imuValid = finiteReadings && plausibleMagnitude;
}

// ---------- Logging heartbeat LED state ----------
// Decoupled from individual GPS/IMU writes -- see header comment. We just
// track "did anything get written this interval" and toggle on a fixed,
// human-visible cadence.
bool logLedOn = false;
unsigned long lastLogLedToggleMs = 0;
bool dataWrittenSinceToggle = false;

void noteDataWritten() {
  dataWrittenSinceToggle = true;
}

void serviceLogLed() {
  unsigned long now = millis();
  if (now - lastLogLedToggleMs < LOG_LED_BLINK_INTERVAL_MS) {
    return;
  }
  lastLogLedToggleMs = now;

  if (dataWrittenSinceToggle) {
    // Data came in during this window -- keep the heartbeat blinking.
    logLedOn = !logLedOn;
  } else {
    // Nothing was written this whole window -- go dark. A steady dark
    // LED during a flight means writes have stalled.
    logLedOn = false;
  }
  digitalWrite(LOG_LED_PIN, logLedOn ? HIGH : LOW);
  dataWrittenSinceToggle = false;
}

// ---------- Status LEDs (calibrate / upload / ready) ----------

void allStatusLedsOff() {
  digitalWrite(CALIBRATE_LED, LOW);
  digitalWrite(UPLOAD_LED, LOW);
  digitalWrite(READY_LED, LOW);
}

// READY_LED behavior while idle (waiting for the BOOT button):
//   - no valid GPS fix yet        -> solid off (not ready to start)
//   - valid GPS, no/invalid IMU   -> slow blink (can start, GPS-only)
//   - valid GPS AND valid IMU     -> fast blink (can start, full data)
// GPS is the hard requirement; IMU is a bonus (logging still works fine
// without one, matching the GPS-only fallback elsewhere in this file).
bool readyLedOn = false;
unsigned long lastReadyLedToggleMs = 0;

void serviceReadyLed(bool gpsValid, bool imuValidNow) {
  if (!gpsValid) {
    if (readyLedOn) {
      readyLedOn = false;
      digitalWrite(READY_LED, LOW);
    }
    return;
  }

  unsigned long interval = imuValidNow ? READY_LED_BLINK_FAST_MS : READY_LED_BLINK_SLOW_MS;
  unsigned long now = millis();
  if (now - lastReadyLedToggleMs >= interval) {
    lastReadyLedToggleMs = now;
    readyLedOn = !readyLedOn;
    digitalWrite(READY_LED, readyLedOn ? HIGH : LOW);
  }
}

// Blocking flash pattern -- only ever used at setup-time (calibration /
// upload result), never inside the logging loop, so it's fine that it
// blocks briefly.
void flashLedBlocking(int pin, int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(STATUS_LED_FLASH_ON_MS);
    digitalWrite(pin, LOW);
    if (i < times - 1) {
      delay(STATUS_LED_FLASH_OFF_MS);
    }
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
  digitalWrite(CALIBRATE_LED, HIGH);

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

  digitalWrite(CALIBRATE_LED, LOW);
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

  if (reading != lastRawButtonReading) {
    lastButtonChangeMs = now; // raw signal just changed, restart debounce window
  }
  lastRawButtonReading = reading;

  bool pressed = false;
  if ((now - lastButtonChangeMs) > BUTTON_DEBOUNCE_MS) {
    // Debounce window elapsed with a stable reading -- safe to trust it.
    // Only fire on the falling edge of the *debounced* state, not the raw one.
    if (reading != debouncedButtonState) {
      debouncedButtonState = reading;
      if (debouncedButtonState == LOW) {
        pressed = true;
      }
    }
  }

  return pressed;
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

void startLogging() {
  startNewFlightLog();

  // Reset write timers so we don't get a burst of "overdue" writes on
  // the first loop iteration after starting.
  unsigned long now = millis();
  lastGpsWriteMs = now;
  lastImuWriteMs = now;
  lastFilterUpdateMs = now;

  // Reset the logging heartbeat too.
  lastLogLedToggleMs = now;
  dataWrittenSinceToggle = false;
  logLedOn = false;

  // Reset the ready-LED blink state so it starts clean next time we're idle.
  readyLedOn = false;
  lastReadyLedToggleMs = now;

  loggingActive = true;

  // Leaving the "waiting for button" state -- status LEDs turn off
  // (READY_LED is the only one that should actually be lit at this
  // point, the others are defensive).
  allStatusLedsOff();

  // Explicit "logging started" confirmation -- solid on immediately,
  // independent of the heartbeat blink. The heartbeat takes over from
  // here as soon as serviceLogLed() runs in the main loop.
  digitalWrite(LOG_LED_PIN, HIGH);
  logLedOn = true;

  Serial.println("Boot button pressed -- logging started.");
}

void writeGpsFix(unsigned long nowMs) {
  JsonDocument doc;
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

  noteDataWritten();
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

  JsonDocument doc;
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

  noteDataWritten();
}

// ---------- Upload previous flight over WiFi ----------

void uploadPendingFlight() {
  if (!LittleFS.exists(LOG_FILE_PATH)) {
    Serial.println("No pending flight file to upload.");
    flashLedBlocking(UPLOAD_LED, 2);
    return;
  }

  File f = LittleFS.open(LOG_FILE_PATH, "r");
  if (!f || f.size() == 0) {
    if (f) f.close();
    LittleFS.remove(LOG_FILE_PATH);
    flashLedBlocking(UPLOAD_LED, 2);
    return;
  }

  // ---- Pass 1: pre-scan the whole file for the ms range and point count.
  // Needed so every batch anchors timestamps against the same first/last
  // ms, regardless of upload order or how the file gets split into batches.
  unsigned long firstMs = 0;
  unsigned long lastMs = 0;
  bool haveRange = false;
  unsigned int totalPoints = 0;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (line.length() < 2) continue;
    JsonDocument point;
    DeserializationError err = deserializeJson(point, line);
    if (!err) {
      totalPoints++;
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

  if (totalPoints == 0) {
    Serial.println("Log file had no valid points -- discarding.");
    LittleFS.remove(LOG_FILE_PATH);
    flashLedBlocking(UPLOAD_LED, 2);
    return;
  }

  // We now know there's a real upload to attempt -- UPLOAD_LED goes solid
  // on for the duration of the network work, then flashes the result.
  digitalWrite(UPLOAD_LED, HIGH);

  time_t nowT = time(nullptr);
  char isoTime[32];
  struct tm tmInfo;
  gmtime_r(&nowT, &tmInfo);
  strftime(isoTime, sizeof(isoTime), "%Y-%m-%dT%H:%M:%SZ", &tmInfo);

  Serial.printf("Uploading %u points in batches of %d...\n", totalPoints, UPLOAD_BATCH_SIZE);

  // ---- Pass 2: re-read the file and upload it in small batches. Keeping
  // each batch's JsonDocument/String small and short-lived (freed at the
  // end of each loop iteration) avoids the heap exhaustion that a single
  // huge payload for the whole flight can cause.
  f = LittleFS.open(LOG_FILE_PATH, "r");
  if (!f) {
    Serial.println("Could not reopen log file for upload.");
    digitalWrite(UPLOAD_LED, LOW);
    flashLedBlocking(UPLOAD_LED, 3);
    return;
  }

  int flightId = -1; // -1 = not yet assigned; server returns it after batch 1
  unsigned int gpsStored = 0, imuStored = 0, skipped = 0;
  bool uploadFailed = false;

  while (f.available()) {
    JsonDocument batch;
    batch["drone_name"] = DRONE_NAME;
    batch["upload_time"] = isoTime;
    if (flightId != -1) batch["flight_id"] = flightId;
    batch["flight_first_ms"] = firstMs;
    batch["flight_last_ms"] = lastMs;
    JsonArray points = batch.createNestedArray("points");

    int batchCount = 0;
    while (f.available() && batchCount < UPLOAD_BATCH_SIZE) {
      String line = f.readStringUntil('\n');
      if (line.length() < 2) continue;
      JsonDocument point;
      DeserializationError err = deserializeJson(point, line);
      if (!err) {
        points.add(point.as<JsonObject>());
        batchCount++;
      }
    }

    if (batchCount == 0) break; // trailing blank lines, nothing left to send

    String body;
    serializeJson(batch, body);

    HTTPClient http;
    http.begin(SERVER_URL);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-API-Key", API_KEY);
    int status = http.POST(body);
    String response = http.getString();
    http.end();

    Serial.printf("Batch of %d points -> HTTP %d\n", batchCount, status);

    if (status == 200 || status == 201) {
      JsonDocument respDoc;
      DeserializationError respErr = deserializeJson(respDoc, response);
      if (!respErr) {
        flightId = respDoc["flight_id"].as<int>();
        gpsStored += respDoc["gps_points_stored"].as<unsigned int>();
        imuStored += respDoc["imu_points_stored"].as<unsigned int>();
        skipped += respDoc["points_skipped"].as<unsigned int>();
      } else {
        Serial.println("Could not parse batch response -- flight_id may be lost.");
      }
    } else {
      Serial.println("Batch upload failed -- stopping. Local log kept for retry.");
      Serial.println(response);
      uploadFailed = true;
      break;
    }
  }
  f.close();

  digitalWrite(UPLOAD_LED, LOW);

  if (!uploadFailed && flightId != -1) {
    Serial.printf("Upload complete: %u GPS, %u IMU, %u skipped (flight_id=%d)\n",
                  gpsStored, imuStored, skipped, flightId);
    Serial.println("Clearing local log.");
    LittleFS.remove(LOG_FILE_PATH);
    flashLedBlocking(UPLOAD_LED, 1);
  } else {
    // NOTE: if some batches succeeded before one failed, those points are
    // already stored server-side. Retrying uploads the whole file again
    // next boot, which will re-send (and duplicate) those earlier batches.
    // Fine for a personal project; worth revisiting if that ever matters.
    Serial.println("Upload incomplete -- local log kept for retry next boot.");
    flashLedBlocking(UPLOAD_LED, 3);
  }
}

// Called when the boot button is pressed a second time, while a flight is
// already being logged. GPS/IMU writes already close the file after every
// single append (see writeGpsFix()/writeImuSample()), so there's no
// dangling file handle to clean up here -- but ending the session
// explicitly, rather than just power-cycling the board, gives you a clear
// "logging stopped" confirmation and lets us try uploading immediately
// instead of waiting for the next boot.
void stopLogging() {
  loggingActive = false;

  digitalWrite(LOG_LED_PIN, LOW);
  logLedOn = false;

  Serial.println("Boot button pressed again -- logging stopped.");

  // WiFi was connected once at boot and never explicitly disconnected, so
  // if it's still up we can upload right now instead of waiting for the
  // next power cycle.
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Attempting immediate upload...");
    uploadPendingFlight();
  } else {
    Serial.println("No WiFi connection -- flight log kept on flash for upload next boot.");
    flashLedBlocking(UPLOAD_LED, 3);
  }

  // Reset ready-LED / debounce bookkeeping so the idle state starts clean
  // in case you want to start a new flight log right after this one.
  readyLedOn = false;
  lastReadyLedToggleMs = millis();
}

// ---------- Setup ----------

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LOG_LED_PIN, OUTPUT);
  digitalWrite(LOG_LED_PIN, LOW);

  pinMode(CALIBRATE_LED, OUTPUT);
  pinMode(UPLOAD_LED, OUTPUT);
  pinMode(READY_LED, OUTPUT);
  allStatusLedsOff();

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

  Serial.println("Ready. Waiting for a valid GPS fix before the BOOT button will work.");
  systemReady = true;
  // READY_LED itself is now driven every loop() by serviceReadyLed(),
  // based on live GPS/IMU validity rather than a one-time boot state.
}

// ---------- Main loop ----------

void loop() {
  // Keep feeding the GPS parser continuously -- even before logging
  // starts -- so we know as soon as a fix comes in (and READY_LED can
  // reflect it), rather than only reading GPS once already logging.
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }

  // Re-derived every loop, not just at boot: a fix older than
  // GPS_STALE_MS (satellites lost) no longer counts as valid.
  bool gpsValid = gps.location.isValid() && (gps.location.age() < GPS_STALE_MS);
  updateImuValidity();

  if (!loggingActive) {
    serviceReadyLed(gpsValid, imuValid);

    // Always service the debounce state machine so it doesn't build up
    // a stale "pressed" edge from before a fix appeared -- but only
    // actually start logging if we currently have a valid GPS fix.
    bool buttonPressed = systemReady && checkBootButtonPressed();
    if (buttonPressed) {
      if (gpsValid) {
        startLogging();
      } else {
        Serial.println("Button pressed but no valid GPS fix yet -- ignoring.");
      }
    }
    return; // idle -- nothing else to do until logging starts
  }

  // Still service the debounce state machine while a flight is being
  // logged, so a second press can end the session cleanly and trigger an
  // immediate upload instead of requiring a reboot.
  if (systemReady && checkBootButtonPressed()) {
    stopLogging();
    return;
  }

  unsigned long now = millis();

  if (gps.location.isUpdated() && gpsValid) {
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