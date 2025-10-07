#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>

// I2C address for BNO085/BNO08x (default is usually 0x4A). Define it here
// if your board uses a different address.
#define BNO_ADDR 0x4A

extern "C" {
  #include "vqf.h" // community VQF-C header you dropped in lib/vqf_c
}

// ====== IMU + VQF config ======
static const float GYR_HZ = 400.0f;   // gyro  rate
static const float ACC_HZ = 100.0f;   // accel rate
static const float MAG_HZ = 25.0f;    // mag   rate

static const uint32_t GYR_US = (uint32_t)(1e6f / GYR_HZ);
static const uint32_t ACC_US = (uint32_t)(1e6f / ACC_HZ);
static const uint32_t MAG_US = (uint32_t)(1e6f / MAG_HZ);

static const float GYR_DT = 1.0f / GYR_HZ;
static const float ACC_DT = 1.0f / ACC_HZ;
static const float MAG_DT = 1.0f / MAG_HZ;

// Optional: override I2C pins if your wiring needs it.
// #define I2C_SDA 21
// #define I2C_SCL 22

Adafruit_BNO08x bno08x;
bool bnoPresent = false;

// latest samples
vqf_real_t gyr[3] = {0}, acc[3] = {0}, mag[3] = {0};
bool haveNewGyr = false, haveNewAcc = false;
bool haveGyrSample = false, haveAccSample = false, haveMagSample = false;

static uint32_t streamStartUs = 0;

static void printCsvSample(float t_ms) {
  Serial.print(t_ms, 3);      Serial.print(',');
  Serial.print(acc[0], 6);    Serial.print(',');
  Serial.print(acc[1], 6);    Serial.print(',');
  Serial.print(acc[2], 6);    Serial.print(',');
  Serial.print(gyr[0], 6);    Serial.print(',');
  Serial.print(gyr[1], 6);    Serial.print(',');
  Serial.print(gyr[2], 6);    Serial.print(',');

  if (haveMagSample) {
    Serial.print(mag[0], 6);  Serial.print(',');
    Serial.print(mag[1], 6);  Serial.print(',');
    Serial.println(mag[2], 6);
  } else {
    Serial.print("nan");      Serial.print(',');
    Serial.print("nan");      Serial.print(',');
    Serial.println("nan");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // If needed, pick explicit I2C pins:
  #if defined(I2C_SDA) && defined(I2C_SCL)
    Wire.begin(I2C_SDA, I2C_SCL);
  #else
    Wire.begin();
  #endif
  Wire.setClock(400000);
  delay(100);

  Serial.println(F("BNO085 + VQF-C (PlatformIO)"));

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
    if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_UNCALIBRATED, MAG_US))
      Serial.println(F("WARN: could not enable uncal mag"));
  } else {
    Serial.println(F("Skipping sensor report enable since BNO not present."));
  }

  // Initialize VQF with per-sensor sample times (s)
  initVqf(GYR_DT, ACC_DT, MAG_DT);

  streamStartUs = micros();

  Serial.println(F("Setup OK. Streaming..."));
}

void loop() {
  sh2_SensorValue_t evt;
  while (bno08x.getSensorEvent(&evt)) {
    switch (evt.sensorId) {
      case SH2_GYROSCOPE_UNCALIBRATED:
        // rad/s
        gyr[0] = evt.un.gyroscope.x;
        gyr[1] = evt.un.gyroscope.y;
        gyr[2] = evt.un.gyroscope.z;
        updateGyr(gyr);
        haveNewGyr = true;
        haveGyrSample = true;
        break;

      case SH2_ACCELEROMETER:
        // m/s^2 including gravity
        acc[0] = evt.un.accelerometer.x;
        acc[1] = evt.un.accelerometer.y;
        acc[2] = evt.un.accelerometer.z;
        updateAcc(acc);
        haveNewAcc = true;
        haveAccSample = true;
        break;

      case SH2_MAGNETIC_FIELD_UNCALIBRATED:
        // microtesla
        mag[0] = evt.un.magneticField.x;
        mag[1] = evt.un.magneticField.y;
        mag[2] = evt.un.magneticField.z;
        updateMag(mag);
        haveMagSample = true;
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
}
