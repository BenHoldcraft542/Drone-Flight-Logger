#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial GPSSerial(2); // UART2

void setup() {
    Wire.begin(4, 5);
    Serial.begin(115200);
    GPSSerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17

    if (!mpu.begin()) {
        Serial.println("MPU6050 not found - check wiring!");
        while (1) delay(10);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("MPU6050 + GPS ready");
    }

    void loop() {
    // Read accelerometer/gyro
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.print("Accel X:"); Serial.print(a.acceleration.x);
    Serial.print(" Y:"); Serial.print(a.acceleration.y);
    Serial.print(" Z:"); Serial.println(a.acceleration.z);

    // Feed GPS parser
    while (GPSSerial.available() > 0) {
        gps.encode(GPSSerial.read());
    }

    if (gps.location.isUpdated()) {
        Serial.print("Lat: "); Serial.println(gps.location.lat(), 6);
        Serial.print("Lng: "); Serial.println(gps.location.lng(), 6);
    }

    delay(200);
}