#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include "vqf.hpp" // C++ VQF library

// I2C multiplexer (PCA9548A)
#define PCA9548A_ADDRESS 0x70

// I2C address for BNO085/BNO08x (default is usually 0x4A). Define it here
// if your board uses a different address.
#define BNO_ADDR 0x4A

#define ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
  #define I2C_SDA 42
  #define I2C_SCL 41
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
  #define I2C_SDA 22
  #define I2C_SCL 20
#else
  #define I2C_SDA SDA
  #define I2C_SCL SCL
#endif
// === TFT display for ESP32-S2 Feather TFT ===
#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
  #include <Adafruit_GFX.h>
  #include <Adafruit_ST7789.h>
  #include <Fonts/FreeSans12pt7b.h>

  // Board variant provides these pins:
  // TFT_CS (GPIO7), TFT_DC (GPIO39), TFT_RST (GPIO40), TFT_BACKLITE (GPIO45)
  Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);
  GFXcanvas16 canvas(240, 135);
#endif

if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
static void tftInitOnce() {
  static bool inited = false;
  if (inited) return;

  // Ensure STEMMA QT/TFT/BME 3V3 rail is ON
  #ifdef TFT_I2C_POWER
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    delay(5);
  #endif

  tft.init(135, 240);     // 240x135 panel, params per Adafruit example
  tft.setRotation(3);

  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  canvas.setFont(&FreeSans12pt7b);
  canvas.setTextColor(ST77XX_WHITE);
  inited = true;
}

/** Scan I2C bus and paint found addresses to the TFT (skips 0x36 by default). */
static void tftShowI2CAddresses() {
  tftInitOnce();

  canvas.fillScreen(ST77XX_BLACK);
  int y0 = 26;

  canvas.setCursor(0, y0);
  canvas.setTextColor(ST77XX_RED);
  canvas.println("Adafruit Feather");
  canvas.setTextColor(ST77XX_YELLOW);
  canvas.println("ESP32-S2 TFT");
  canvas.setTextColor(ST77XX_BLUE);
  canvas.print("I2C: ");
  canvas.setTextColor(ST77XX_WHITE);

  uint8_t perLine = 8, printed = 0;
  for (uint8_t addr = 1; addr < 0x7F; ++addr) {
    if (addr == 0x36) continue; // battery gauge; hide like the example
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      char buf[6];
      snprintf(buf, sizeof(buf), "0x%02X", addr);
      canvas.print(buf);
      canvas.print(", ");
      if (++printed % perLine == 0) canvas.println();
    }
  }

  tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
}
#endif

// ====== IMU + VQF config ======
static const float GYR_HZ = 110.0f;   // gyro  rate
static const float ACC_HZ = 110.0f;   // accel rate

static const uint32_t GYR_US = (uint32_t)(1e6f / GYR_HZ);
static const uint32_t ACC_US = (uint32_t)(1e6f / ACC_HZ);

static const float GYR_DT = 1.0f / GYR_HZ;
static const float ACC_DT = 1.0f / ACC_HZ;

// Optional: override I2C pins if your wiring needs it.
// #define I2C_SDA 21
// #define I2C_SCL 22

Adafruit_BNO08x bno08x;
bool bnoPresent = false;

// VQF instance - using C++ class
VQF vqf(GYR_DT, ACC_DT, 0.0f); // 0.0f for magTs disables magnetometer

// latest samples
vqf_real_t gyr[3] = {0}, acc[3] = {0};
bool haveNewGyr = false, haveNewAcc = false;
bool haveGyrSample = false, haveAccSample = false;

static uint32_t streamStartUs = 0;

static void selectMultiplexerChannel(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(PCA9548A_ADDRESS);
  Wire.write(1u << ch);
  Wire.endTransmission();
}

// Optional: handle PCA9548A mux @ 0x70 if present
static bool selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(PCA9548A_ADDRESS);
  Wire.write(1 << (channel & 7));
  return (Wire.endTransmission() == 0);
}

static bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// Try all mux channels to find the BNO on 0x4A/0x28
static int8_t autoSelectMuxForBNO() {
  if (!i2cDevicePresent(PCA9548A_ADDRESS)) return -1; // no mux present
  for (uint8_t ch = 0; ch < 8; ++ch) {
    if (!selectMuxChannel(ch)) continue;
    delay(2);
    if (i2cDevicePresent(0x4A) || i2cDevicePresent(0x28)) return (int8_t)ch;
  }
  return -2; // mux present but BNO not seen on any channel
}

static void printCsvSample(float t_ms) {
  // Get the 6D quaternion from VQF (w, x, y, z)
  vqf_real_t quat[4];
  vqf.getQuat6D(quat);
  
  Serial.print(t_ms, 3);      Serial.print(',');
  Serial.print(acc[0], 6);    Serial.print(',');
  Serial.print(acc[1], 6);    Serial.print(',');
  Serial.print(acc[2], 6);    Serial.print(',');
  Serial.print(gyr[0], 6);    Serial.print(',');
  Serial.print(gyr[1], 6);    Serial.print(',');
  Serial.print(gyr[2], 6);    Serial.print(',');
  Serial.print(quat[0], 6);   Serial.print(',');
  Serial.print(quat[1], 6);   Serial.print(',');
  Serial.print(quat[2], 6);   Serial.print(',');
  Serial.println(quat[3], 6);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  #ifdef TFT_I2C_POWER         // present on Feather ESP32-S2 TFT
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    delay(5);
  #endif
  #ifdef NEOPIXEL_I2C_POWER    // some variants used this name; harmless if defined
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
    delay(5);
  #endif

  #ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT
    pinMode(21, OUTPUT);
    digitalWrite(21, HIGH); // Enable 3V3 rail, TFT_I2C_POWER may not be defined
    delay(5);
  #endif
  #ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT
    pinMode(34, OUTPUT);
    digitalWrite(34, HIGH); // Neopixel power
    delay(5);
  #endif
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeout(1000);
  // Wire.setClock(100000);
  delay(100);

  Serial.println(F("BNO085 + VQF-C (PlatformIO)"));

  // Optional: auto-pick mux channel if a PCA9548A (0x70) is on the bus
  int8_t muxCh = autoSelectMuxForBNO();
  if (muxCh >= 0) {
    Serial.printf("I2C mux detected @0x70, selected channel %d for BNO.\n", muxCh);
  } else if (muxCh == -2) {
    Serial.println("I2C mux present but BNO not found on any channel.");
  } else {
    Serial.println("No I2C mux detected.");
  }
  Adafruit_BNO08x bno;

  // bool bno_ok = false;
  // for (uint8_t addr : {0x4A, 0x28}) {
  //   Serial.printf("Trying BNO init at 0x%02X...\n", addr);
  //   if (bno.begin_I2C(addr, &Wire)) {  // or bno.begin_I2C(addr)
  //     Serial.printf("BNO08x initialized at 0x%02X\n", addr);
  //     bno_ok = true;
  //     break;
  //   } else {
  //     Serial.println("I2C address not found");
  //   }
  // }
  // if (!bno_ok) {
  //   Serial.println("ERROR: BNO08x not found. Check wiring and I2C address. Proceeding without sensor.");
  //   // Make sure the rest of your loop guards against bno use when !bno_ok.
  // }


  // Print a quick I2C scan to help debugging
  Serial.println(F("I2C scan:"));
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf(" - Found device at 7-bit 0x%02X\n", addr);
      delay(5);
    }
  }

  // Select multiplexer channel 0 for IMU
  selectMultiplexerChannel(0);
  delay(50);

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
  tftShowI2CAddresses();   // draw the detected I2C addresses to the TFT
  #endif

  // Try to initialize the BNO at the configured address, fall back to 0x28
  uint8_t triedAddrs[2] = { (uint8_t)BNO_ADDR, 0x28 };
  uint8_t usedAddr = 0;
  for (int i = 0; i < 2; ++i) {
    uint8_t a = triedAddrs[i];
    Serial.printf("Trying BNO init at 0x%02X...\n", a);
    if (bno08x.begin_I2C(a, &Wire)) {
      usedAddr = a;
      bnoPresent = true;
      Serial.printf("BNO08x initialized at 0x%02X\n", a);
      break;
    }
    delay(50);
  }

  if (!bnoPresent) {
    Serial.println(F("ERROR: BNO08x not found. Check wiring and I2C address. Proceeding without sensor."));
  }

  // Enable raw/uncalibrated sensors (perfect for external fusion)
  if (bnoPresent) {
    if (!bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED, GYR_US))
      Serial.println(F("WARN: could not enable uncal gyro"));
    if (!bno08x.enableReport(SH2_ACCELEROMETER, ACC_US))
      Serial.println(F("WARN: could not enable accel"));
  } else {
    Serial.println(F("Skipping sensor report enable since BNO not present."));
  }

  // VQF is already initialized in the global declaration above
  // No need to call initVqf() - the C++ constructor handles it
  streamStartUs = micros();

  Serial.println(F("Setup OK. Streaming..."));
}

void loop() {
  // Select multiplexer channel 0 before reading IMU
  // selectMultiplexerChannel(0);
  
  sh2_SensorValue_t evt;
  uint32_t guard_us = micros();
  while (bno08x.getSensorEvent(&evt)) {
    switch (evt.sensorId) {
      case SH2_GYROSCOPE_UNCALIBRATED:
        // rad/s
        gyr[0] = evt.un.gyroscope.x;
        gyr[1] = evt.un.gyroscope.y;
        gyr[2] = evt.un.gyroscope.z;
        vqf.updateGyr(gyr);
        haveNewGyr = true;
        haveGyrSample = true;
        break;

      case SH2_ACCELEROMETER:
        // m/s^2 including gravity
        acc[0] = evt.un.accelerometer.x;
        acc[1] = evt.un.accelerometer.y;
        acc[2] = evt.un.accelerometer.z;
        vqf.updateAcc(acc);
        haveNewAcc = true;
        haveAccSample = true;
        break;
    }
  }

  if ((haveNewGyr || haveNewAcc) && haveGyrSample && haveAccSample) {
    uint32_t nowUs = micros();
    float t_ms = (nowUs - streamStartUs) * 0.001f;
    printCsvSample(t_ms);
    haveNewGyr = false;
    haveNewAcc = false;
  }
  if ((micros() - guard_us) > 5000)break;
  yield(); // allow background tasks

}