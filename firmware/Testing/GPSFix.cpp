/*
  GPS Diagnostic Test - ESP32 + NEO-6M (or similar) GPS module

  What this does, in order:
    1. Checks that the ESP32 is receiving ANY bytes at all on UART2.
       If not -> wiring/power problem, not a "no satellite fix" problem.
    2. Prints raw NMEA sentences for a few seconds so you can see the
       module is talking (even before it has a fix).
    3. Feeds those sentences into TinyGPSPlus and reports parsed stats
       every second: satellites in view, HDOP (signal quality), whether
       a fix has been achieved, and how many sentences failed checksum.

  Wiring (same as main firmware):
    GPS TX  -> ESP32 GPIO16 (RX2)
    GPS RX  -> ESP32 GPIO17 (TX2)
    GPS VCC -> 3.3V (check your module's voltage requirement -- some
               NEO-6M breakout boards want 5V into a VCC pin that has
               its own regulator; check silkscreen/datasheet)
    GPS GND -> GND

  How to use:
    - This is wired up to PlatformIO's existing "testing" environment.
      From the project root:
        pio run -e testing -t upload -t monitor
    - A GPS fix can take 30 seconds to several minutes on a cold start,
      or longer indoors. For fastest results, take the module outside
      or near a window with a clear view of the sky.

  How to read the output:
    - No raw bytes at all after ~5s -> wiring or baud rate problem.
      Double check TX/RX aren't swapped, and that VCC/GND are solid.
    - Raw NMEA lines appear ($GPGGA, $GPRMC, etc.) but "Satellites: 0"
      forever -> module is alive and talking, but hasn't found a fix
      yet. Move it outside/near a window and give it more time.
    - Satellites > 0 but climbing slowly, HDOP high (>5) -> weak view
      of the sky, needs a clearer line of sight.
    - "Fix acquired!" with a real Lat/Lng -> GPS module is working
      correctly, so if you're still not seeing data in flights, the
      problem is elsewhere (firmware upload logic, WiFi, backend).
*/

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

#define GPS_RX_PIN 16   // ESP32 RX2 <- GPS TX
#define GPS_TX_PIN 17   // ESP32 TX2 -> GPS RX
#define GPS_BAUD   9600 // NEO-6M default; some modules use 4800 or 38400

HardwareSerial GPSSerial(2);
TinyGPSPlus gps;

unsigned long lastStatusPrintMs = 0;
unsigned long bootMs = 0;
unsigned long rawByteCount = 0;
bool sawAnyRawData = false;
bool sawFix = false;

void printRawByteWarningIfNeeded() {
  // 5 seconds with zero bytes received strongly suggests a wiring,
  // power, or baud-rate problem rather than "just needs more time".
  if (!sawAnyRawData && millis() - bootMs > 5000) {
    Serial.println("!! No data received from GPS module after 5s.");
    Serial.println("!! Check: TX/RX not swapped, VCC/GND solid, baud rate correct.");
    Serial.println("!! (Will keep listening in case it's just slow to power up.)");
    bootMs = millis(); // reset so this warning repeats every 5s instead of spamming
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println();
  Serial.println("=== GPS Diagnostic Test ===");
  Serial.printf("Listening on UART2 (RX=%d, TX=%d) at %d baud\n",
                GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
  Serial.println("Waiting for data from GPS module...");
  Serial.println();

  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  bootMs = millis();
  lastStatusPrintMs = millis();
}

void loop() {
  // Pull in and parse anything the GPS module has sent.
  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    rawByteCount++;
    if (!sawAnyRawData) {
      sawAnyRawData = true;
      Serial.println(">> First byte received from GPS module. It's alive.");
    }

    // Echo raw NMEA as it arrives so you can eyeball it directly.
    Serial.write(c);

    gps.encode(c);
  }

  printRawByteWarningIfNeeded();

  // Print a parsed-status summary once a second.
  if (millis() - lastStatusPrintMs >= 1000) {
    lastStatusPrintMs = millis();

    Serial.println();
    Serial.println("---- Status ----");
    Serial.printf("Raw bytes received total : %lu\n", rawByteCount);
    Serial.printf("NMEA sentences parsed OK : %lu\n", gps.passedChecksum());
    Serial.printf("NMEA sentences failed    : %lu\n", gps.failedChecksum());
    Serial.printf("Satellites in view       : %d\n",
                  gps.satellites.isValid() ? gps.satellites.value() : 0);
    Serial.printf("HDOP (lower = better)    : %s\n",
                  gps.hdop.isValid() ? String(gps.hdop.hdop(), 1).c_str() : "unknown");

    if (gps.location.isValid() && gps.location.isUpdated()) {
      if (!sawFix) {
        sawFix = true;
        Serial.println();
        Serial.println(">>>> Fix acquired! <<<<");
      }
      Serial.printf("Lat / Lng                : %.6f, %.6f\n",
                    gps.location.lat(), gps.location.lng());
      Serial.printf("Altitude (m)             : %.1f\n",
                    gps.altitude.isValid() ? gps.altitude.meters() : 0.0);
    } else {
      Serial.println("Fix status                : no fix yet");
    }
    Serial.println("-----------------");
    Serial.println();
  }
}