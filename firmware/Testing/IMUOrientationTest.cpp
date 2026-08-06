// firmware/Testing/IMU_OrientationTest.cpp
// Tests MPU6050 + GPS with a Madgwick AHRS filter (quaternion-based,
// IMU-only / 6-DOF: accel + gyro, no magnetometer).
// Uses existing PlatformIO [env:testing] environment.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <math.h>

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);

// ---------------------------------------------------------------------
// Madgwick AHRS (IMU-only variant, no magnetometer)
// Based on Sebastian Madgwick's 2010 algorithm.
// beta = filter gain: higher = trusts accel more (faster correction,
// more jitter), lower = trusts gyro more (smoother, more drift).
// 0.1 is a common starting point for MEMS IMUs; tune from there.
// ---------------------------------------------------------------------
class MadgwickFilter {
public:
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion
    float beta = 0.1f;

    // gx, gy, gz in rad/s; ax, ay, az in any consistent units (normalized internally)
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Only apply accelerometer correction if the reading is valid
        // (avoids NaN when accel vector is exactly zero)
        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

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

    // Euler angles in degrees
    float getRoll() {
        return atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
    }
    float getPitch() {
        float sinp = 2.0f * (q0 * q2 - q3 * q1);
        if (fabsf(sinp) >= 1.0f)
            return copysignf(90.0f, sinp); // gimbal lock clamp
        return asinf(sinp) * 180.0f / PI;
    }
    float getYaw() {
        return atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;
    }
};

MadgwickFilter filter;
unsigned long lastFilterUpdate = 0;

// Gyro zero-rate bias, in rad/s. Seeded by startup calibration,
// then continuously refined while the board is detected as stationary
// (corrects for thermal drift, which a one-shot calibration can't).
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;

// Thresholds for "is the board currently stationary?"
const float STATIONARY_GYRO_THRESH = 0.017f;  // ~1 deg/s in rad/s
const float STATIONARY_ACCEL_MIN   = 9.5f;    // m/s^2, expect ~9.8 at rest
const float STATIONARY_ACCEL_MAX   = 10.1f;
const float BIAS_LEARN_RATE        = 0.002f;  // slow EMA - don't chase real motion

// Deadband: residual rates below this (after bias removal) are treated as
// zero. Cheap insurance against sub-threshold bias/noise still integrating.
const float GYRO_DEADBAND = 0.003f; // ~0.17 deg/s in rad/s

void calibrateGyro() {
    const int samples = 500; // ~5s at 10ms per sample
    double sumX = 0, sumY = 0, sumZ = 0;

    Serial.println("Calibrating gyro bias - keep the board still...");

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
    Serial.println("Calibration done. Bias will continue adapting at rest.");
}

// Call every loop with raw (un-bias-corrected) readings. Detects stationary
// periods and slowly nudges the bias estimate to track thermal drift.
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

void setup() {
    Wire.begin(4, 5); // SDA=4, SCL=5
    Serial.begin(115200);
    GPSSerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17

    if (!mpu.begin()) {
        Serial.println("MPU6050 not found - check wiring!");
        while (1) delay(10);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("MPU6050 + GPS ready (Madgwick AHRS)");

    calibrateGyro(); // board MUST be stationary and flat during this ~5s window

    Serial.println("Ready. Lay flat and let it settle ~2-3s before testing tilts.");

    lastFilterUpdate = millis();
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long now = millis();
    float dt = (now - lastFilterUpdate) / 1000.0f;
    lastFilterUpdate = now;

    if (dt > 0 && dt < 0.5f) { // guard against stalls/first-run
        // Keep bias estimate adapting while stationary (tracks thermal drift)
        updateGyroBias(a, g);

        float gx = g.gyro.x - gyroBiasX;
        float gy = g.gyro.y - gyroBiasY;
        float gz = g.gyro.z - gyroBiasZ;

        // Deadband: kill tiny residual rates so they don't slowly integrate
        if (fabsf(gx) < GYRO_DEADBAND) gx = 0.0f;
        if (fabsf(gy) < GYRO_DEADBAND) gy = 0.0f;
        if (fabsf(gz) < GYRO_DEADBAND) gz = 0.0f;

        filter.update(gx, gy, gz,
                       a.acceleration.x, a.acceleration.y, a.acceleration.z, dt);
    }

    Serial.print("Roll: ");  Serial.print(filter.getRoll(), 1);
    Serial.print("\tPitch: "); Serial.print(filter.getPitch(), 1);
    Serial.print("\tYaw: ");   Serial.println(filter.getYaw(), 1);

    // --- GPS (ground truth for displacement, unaffected by filter) ---
    while (GPSSerial.available() > 0) {
        gps.encode(GPSSerial.read());
    }
    if (gps.location.isUpdated()) {
        Serial.print("Lat: "); Serial.print(gps.location.lat(), 6);
        Serial.print("  Lng: "); Serial.println(gps.location.lng(), 6);
    }

    delay(20); // ~50Hz; Madgwick performs best with tighter, consistent loop timing
}