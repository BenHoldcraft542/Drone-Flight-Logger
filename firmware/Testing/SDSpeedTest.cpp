// sd_speed_test.cpp
//
// SD card SPI write/read speed benchmark for the Adafruit MicroSD
// breakout board+ (PID 254) on an ESP32.
//
// Drop this into firmware/Testing/ as its own PlatformIO env, e.g.:
//
// [env:sd_speed_test]
// platform = espressif32
// board = esp32dev
// framework = arduino
// build_src_filter = +<Testing/sd_speed_test.cpp> -<main.cpp>
// lib_deps =
//     SD
//
// Wiring (VSPI default pins):
//   Breakout CLK  -> GPIO18
//   Breakout DO   -> GPIO19 (MISO)
//   Breakout DI   -> GPIO23 (MOSI)
//   Breakout CS   -> GPIO33 (change SD_CS below if needed; GPIO5 is
//                    reserved for the IMU on your board)
//   Breakout 5V/3V -> match your breakout's logic level pin
//   Breakout GND  -> GND
//
// What it does:
//   1. Mounts the card, prints type/size.
//   2. For each block size in TEST_BLOCK_SIZES, writes TEST_FILE_SIZE_MB
//      worth of data in that block size and reports write throughput.
//   3. Reads the same file back sequentially and reports read throughput.
//   4. Deletes the test file and moves to the next block size.
//
// Notes on interpreting results:
//   - SPI mode is 1-bit, so don't expect anywhere close to the card's
//     rated 80MB/s (that number assumes 4-bit SD-bus mode at high clock).
//   - Realistic ESP32 SPI numbers are usually low single-digit MB/s for
//     writes, better for reads. If you see well under ~1MB/s at all
//     block sizes, suspect SPI clock speed, wiring, or a slow/counterfeit
//     card rather than a fundamental limit.
//   - Larger block sizes amortize per-transaction SPI overhead, so
//     throughput should generally rise with block size up to a point.

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// ---- Configuration ----------------------------------------------------

#define SD_CS 15          // change to GPIO15 if that's what you wired
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23
#define SPI_SD_FREQ_HZ 20000000   // 20MHz; drop to 4000000 if you get errors

static const size_t TEST_BLOCK_SIZES[] = { 512, 4096, 16384, 65536 };
static const size_t NUM_BLOCK_SIZES = sizeof(TEST_BLOCK_SIZES) / sizeof(TEST_BLOCK_SIZES[0]);

static const size_t TEST_FILE_SIZE_MB = 20;   // total data written per block-size pass
static const char* TEST_FILE_PATH = "/sdtest.bin";

// ---- Helpers ------------------------------------------------------------

// Fill a buffer with pseudo-random-ish but cheap-to-generate data so we're
// not benchmarking memset() instead of the SD card.
void fillBuffer(uint8_t* buf, size_t len, uint32_t seed) {
  uint32_t x = seed;
  for (size_t i = 0; i < len; i++) {
    x = x * 1664525u + 1013904223u; // simple LCG
    buf[i] = (uint8_t)(x >> 24);
  }
}

void printCardInfo() {
  uint8_t cardType = SD.cardType();
  Serial.print("Card type: ");
  switch (cardType) {
    case CARD_NONE:  Serial.println("No SD card attached"); return;
    case CARD_MMC:   Serial.println("MMC"); break;
    case CARD_SD:    Serial.println("SDSC"); break;
    case CARD_SDHC:  Serial.println("SDHC/SDXC"); break;
    default:         Serial.println("Unknown"); break;
  }
  uint64_t cardSizeMB = SD.cardSize() / (1024 * 1024);
  Serial.printf("Card size: %llu MB\n", cardSizeMB);
  Serial.printf("Used: %llu MB / Total: %llu MB\n",
                 SD.usedBytes() / (1024 * 1024),
                 SD.totalBytes() / (1024 * 1024));
}

// Runs a write test for one block size. Returns throughput in MB/s.
float runWriteTest(size_t blockSize, size_t totalBytes) {
  uint8_t* buf = (uint8_t*)malloc(blockSize);
  if (!buf) {
    Serial.println("  FAILED: could not allocate buffer");
    return -1.0f;
  }
  fillBuffer(buf, blockSize, 0xA5A5A5A5);

  SD.remove(TEST_FILE_PATH);
  File f = SD.open(TEST_FILE_PATH, FILE_WRITE);
  if (!f) {
    Serial.println("  FAILED: could not open file for write");
    free(buf);
    return -1.0f;
  }

  size_t written = 0;
  uint32_t startMs = millis();
  while (written < totalBytes) {
    size_t n = f.write(buf, blockSize);
    if (n != blockSize) {
      Serial.printf("  WARNING: short write (%u of %u bytes) at offset %u\n",
                     (unsigned)n, (unsigned)blockSize, (unsigned)written);
      break;
    }
    written += n;
  }
  f.flush();
  f.close();
  uint32_t elapsedMs = millis() - startMs;

  free(buf);

  float seconds = elapsedMs / 1000.0f;
  float mbWritten = written / (1024.0f * 1024.0f);
  float mbPerSec = seconds > 0 ? (mbWritten / seconds) : 0;

  Serial.printf("  Wrote %.2f MB in %lu ms -> %.2f MB/s\n",
                mbWritten, (unsigned long)elapsedMs, mbPerSec);
  return mbPerSec;
}

// Runs a read test for the file just written. Returns throughput in MB/s.
float runReadTest(size_t blockSize) {
  uint8_t* buf = (uint8_t*)malloc(blockSize);
  if (!buf) {
    Serial.println("  FAILED: could not allocate buffer");
    return -1.0f;
  }

  File f = SD.open(TEST_FILE_PATH, FILE_READ);
  if (!f) {
    Serial.println("  FAILED: could not open file for read");
    free(buf);
    return -1.0f;
  }

  size_t readTotal = 0;
  uint32_t startMs = millis();
  int n;
  while ((n = f.read(buf, blockSize)) > 0) {
    readTotal += n;
  }
  f.close();
  uint32_t elapsedMs = millis() - startMs;

  free(buf);

  float seconds = elapsedMs / 1000.0f;
  float mbRead = readTotal / (1024.0f * 1024.0f);
  float mbPerSec = seconds > 0 ? (mbRead / seconds) : 0;

  Serial.printf("  Read %.2f MB in %lu ms -> %.2f MB/s\n",
                mbRead, (unsigned long)elapsedMs, mbPerSec);
  return mbPerSec;
}

// ---- Arduino entry points ------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1500); // give serial monitor time to attach
  Serial.println();
  Serial.println("=== SD Card SPI Speed Test ===");

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, SPI, SPI_SD_FREQ_HZ)) {
    Serial.println("FATAL: SD.begin() failed. Check wiring and CS pin.");
    while (true) delay(1000);
  }

  printCardInfo();
  Serial.printf("SPI clock: %d Hz\n", SPI_SD_FREQ_HZ);
  Serial.printf("Test file size per pass: %u MB\n\n", (unsigned)TEST_FILE_SIZE_MB);

  size_t totalBytes = TEST_FILE_SIZE_MB * 1024UL * 1024UL;

  Serial.println("block_size_bytes,write_mb_s,read_mb_s");
  for (size_t i = 0; i < NUM_BLOCK_SIZES; i++) {
    size_t blockSize = TEST_BLOCK_SIZES[i];
    Serial.printf("--- Block size: %u bytes ---\n", (unsigned)blockSize);

    float writeSpeed = runWriteTest(blockSize, totalBytes);
    float readSpeed = runReadTest(blockSize);

    Serial.printf("SUMMARY,%u,%.2f,%.2f\n\n",
                  (unsigned)blockSize, writeSpeed, readSpeed);

    SD.remove(TEST_FILE_PATH);
    delay(200);
  }

  Serial.println("=== Test complete ===");
}

void loop() {
  // nothing to do
}