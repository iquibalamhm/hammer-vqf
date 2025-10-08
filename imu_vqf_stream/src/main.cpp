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

// --- Foot Progression Angle ---
#include "FPA.hpp"

#ifndef ENABLE_FPA_DEBUG
#define ENABLE_FPA_DEBUG 0
#endif

#ifndef ENABLE_CSV_STREAM
#define ENABLE_CSV_STREAM 1
#endif

#if ENABLE_FPA_DEBUG
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINTLN(x) do {} while (0)
#define DEBUG_PRINT(x) do {} while (0)
#define DEBUG_PRINTF(...) do {} while (0)
#endif
static FPA g_fpa(/*is_left_foot=*/false); // set to false if this sensor is on the RIGHT foot
static uint32_t g_lastFeedUs = 0;
static float g_lastFeedDt = 0.0f;
static bool g_lastFeedAccepted = false;
static uint32_t g_lastDebugMs = 0;
static uint32_t g_lastStridePrinted = 0;
static float g_lastFpaDeg = NAN;
static float g_lastStrideDuration = 0.0f;
static float g_lastStrideDx = 0.0f;
static float g_lastStrideDy = 0.0f;
static bool g_lastFootValid = false;
static float g_lastFootRollDeg = NAN;
static float g_lastFootPitchDeg = NAN;
static float g_lastFootYawDeg = NAN;

static void quatToEulerDeg(const float q[4], float* outRollDeg, float* outPitchDeg, float* outYawDeg) {
  // q = (w, x, y, z)
  float sinr_cosp = 2.0f * (q[0]*q[1] + q[2]*q[3]);
  float cosr_cosp = 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]);
  float roll = atan2f(sinr_cosp, cosr_cosp);

  float sinp = 2.0f * (q[0]*q[2] - q[3]*q[1]);
  float pitch;
  if (fabsf(sinp) >= 1.0f) {
    pitch = copysignf(1.57079632679f, sinp); // clamp to +-pi/2
  } else {
    pitch = asinf(sinp);
  }

  float siny_cosp = 2.0f * (q[0]*q[3] + q[1]*q[2]);
  float cosy_cosp = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);
  float yaw = atan2f(siny_cosp, cosy_cosp);

  const float RAD2DEG = 57.29577951308232f;
  if (outRollDeg) *outRollDeg = roll * RAD2DEG;
  if (outPitchDeg) *outPitchDeg = pitch * RAD2DEG;
  if (outYawDeg) *outYawDeg = yaw * RAD2DEG;
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

static const float FPA_ACC_DIFF_TH = 0.25f;
static const float FPA_GYR_NORM_TH = 0.7f;
static const float FPA_HYS_FRAC = 0.2f;
static const float FPA_MIN_REST_S = 0.05f;
static const float FPA_MIN_MOTION_S = 0.05f;

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
#if ENABLE_CSV_STREAM
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
    Serial.print(mag[2], 6);
  } else {
    Serial.print("nan");      Serial.print(',');
    Serial.print("nan");      Serial.print(',');
    Serial.print("nan");
  }

  Serial.print(",footRoll_deg=");
  if (g_lastFootValid && !isnan(g_lastFootRollDeg)) {
    Serial.print(g_lastFootRollDeg, 3);
  } else {
    Serial.print("nan");
  }
  Serial.print(",footPitch_deg=");
  if (g_lastFootValid && !isnan(g_lastFootPitchDeg)) {
    Serial.print(g_lastFootPitchDeg, 3);
  } else {
    Serial.print("nan");
  }
  Serial.print(",footYaw_deg=");
  if (g_lastFootValid && !isnan(g_lastFootYawDeg)) {
    Serial.print(g_lastFootYawDeg, 3);
  } else {
    Serial.print("nan");
  }
#else
  (void)t_ms;
#endif
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

  DEBUG_PRINTLN(F("BNO085 + VQF-C (PlatformIO)"));

  #if ENABLE_FPA_DEBUG
    // Print a quick I2C scan to help debugging
    DEBUG_PRINTLN(F("I2C scan:"));
    for (uint8_t addr = 1; addr < 127; ++addr) {
      Wire.beginTransmission(addr);
      uint8_t err = Wire.endTransmission();
      if (err == 0) {
        DEBUG_PRINTF(" - Found device at 7-bit 0x%02X\n", addr);
        delay(5);
      }
    }
  #endif

  // Try to initialize the BNO at the configured address, fall back to 0x28
  uint8_t triedAddrs[2] = { (uint8_t)BNO_ADDR, 0x28 };
  uint8_t usedAddr = 0;
  for (int i = 0; i < 2; ++i) {
    uint8_t a = triedAddrs[i];
    DEBUG_PRINTF("Trying BNO init at 0x%02X...\n", a);
    if (bno08x.begin_I2C(a, &Wire)) {
      usedAddr = a;
      bnoPresent = true;
      DEBUG_PRINTF("BNO08x initialized at 0x%02X\n", a);
      break;
    }
    delay(50);
  }

  if (!bnoPresent) {
    DEBUG_PRINTLN(F("ERROR: BNO08x not found. Check wiring and I2C address. Proceeding without sensor."));
  }

  // Enable raw/uncalibrated sensors (perfect for external fusion)
  if (bnoPresent) {
    if (!bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED, GYR_US))
      DEBUG_PRINTLN(F("WARN: could not enable uncal gyro"));
    if (!bno08x.enableReport(SH2_ACCELEROMETER, ACC_US))
      DEBUG_PRINTLN(F("WARN: could not enable accel"));
    if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_UNCALIBRATED, MAG_US))
      DEBUG_PRINTLN(F("WARN: could not enable uncal mag"));
  } else {
    DEBUG_PRINTLN(F("Skipping sensor report enable since BNO not present."));
  }

  // Initialize VQF with per-sensor sample times (s)
  initVqf(GYR_DT, ACC_DT, MAG_DT);

  streamStartUs = micros();
  g_lastFeedUs = 0;
  g_fpa.reset();
  g_fpa.configure(FPA_ACC_DIFF_TH, FPA_GYR_NORM_TH, FPA_HYS_FRAC, FPA_MIN_REST_S, FPA_MIN_MOTION_S);
  g_lastStridePrinted = 0;
  g_lastFpaDeg = NAN;
  g_lastStrideDuration = 0.0f;
  g_lastStrideDx = 0.0f;
  g_lastStrideDy = 0.0f;
  g_lastFootValid = false;
  g_lastFootRollDeg = NAN;
  g_lastFootPitchDeg = NAN;
  g_lastFootYawDeg = NAN;

  DEBUG_PRINTLN(F("Setup OK. Streaming..."));
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
    haveNewGyr = false;
    haveNewAcc = false;
    // --- Feed FPA estimator once per fused sample ---
    g_lastFeedAccepted = false;
    float dt = 0.0f;
    if (g_lastFeedUs != 0) {
      uint32_t deltaUs = nowUs - g_lastFeedUs;
      dt = deltaUs * 1e-6f;
    }
    g_lastFeedUs = nowUs;
    g_lastFeedDt = dt;

    if (dt > 0.0f && dt < 0.2f) {
      vqf_real_t q6[4];
      getQuat6D(q6);
      float q[4] = { static_cast<float>(q6[0]), static_cast<float>(q6[1]), static_cast<float>(q6[2]), static_cast<float>(q6[3]) };
      float acc_f[3] = { static_cast<float>(acc[0]), static_cast<float>(acc[1]), static_cast<float>(acc[2]) };
      float gyr_f[3] = { static_cast<float>(gyr[0]), static_cast<float>(gyr[1]), static_cast<float>(gyr[2]) };
      quatToEulerDeg(q, &g_lastFootRollDeg, &g_lastFootPitchDeg, &g_lastFootYawDeg);
      g_lastFootValid = true;
      g_fpa.feed(acc_f, gyr_f, q, dt);
      g_lastFeedAccepted = true;
    }

    FPA_Result r;
    bool stridePrinted = false;
    bool haveResult = g_fpa.poll(&r) && r.valid;

    printCsvSample(t_ms);

    if (haveResult) {
      Serial.printf(",FPA_deg=%.2f,stride=%lu,dx=%.3f,dy=%.3f,Ts=%.3f",
                    r.fpa_deg,
                    static_cast<unsigned long>(r.stride_index),
                    r.delta_xy_m[0],
                    r.delta_xy_m[1],
                    r.stride_duration_s);
      g_lastStridePrinted = r.stride_index;
      g_lastFpaDeg = r.fpa_deg;
      g_lastStrideDuration = r.stride_duration_s;
      g_lastStrideDx = r.delta_xy_m[0];
      g_lastStrideDy = r.delta_xy_m[1];
      stridePrinted = true;
    }

    bool linePrinted = false;
    #if ENABLE_CSV_STREAM
      linePrinted = true;
    #endif
    if (stridePrinted) {
      linePrinted = true;
    }
    if (linePrinted) {
      Serial.println();
    }
  }

#if ENABLE_FPA_DEBUG
  uint32_t nowMs = millis();
  if (nowMs - g_lastDebugMs > 250) {
    FPA_Debug dbg;
    g_fpa.getDebug(&dbg);
    float acc_norm = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    float acc_diff = fabsf(acc_norm - 9.81f);
    float gyr_norm = sqrtf(gyr[0]*gyr[0] + gyr[1]*gyr[1] + gyr[2]*gyr[2]);
    Serial.printf(
      "# FPA dbg: stride=%lu state=%d raw=%d have(hr=%d fc=%d trest=%d) samples=%d grav=%lu accDiff=%.3f (>%.2f?) gyrNorm=%.3f (>%.2f?) dt=%.4f accepted=%d",
      static_cast<unsigned long>(dbg.stride_index),
      dbg.state,
      dbg.state_raw,
      dbg.have_thr ? 1 : 0,
      dbg.have_tfc ? 1 : 0,
      dbg.have_trest ? 1 : 0,
      dbg.sample_count,
      static_cast<unsigned long>(dbg.grav_count),
      acc_diff,
      dbg.acc_thr,
      gyr_norm,
      dbg.gyr_thr,
      g_lastFeedDt,
      g_lastFeedAccepted ? 1 : 0);

    if (dbg.last_result_valid) {
      Serial.printf(" lastFpaDeg=%.2f stride=%lu Ts=%.3f dx=%.3f dy=%.3f",
                    dbg.last_result_fpa_deg,
                    static_cast<unsigned long>(dbg.last_result_stride_index),
                    dbg.last_result_stride_duration_s,
                    dbg.last_result_dx,
                    dbg.last_result_dy);
    } else if (!isnan(g_lastFpaDeg)) {
      Serial.printf(" lastFpaDeg=%.2f stride=%lu Ts=%.3f dx=%.3f dy=%.3f",
                    g_lastFpaDeg,
                    static_cast<unsigned long>(g_lastStridePrinted),
                    g_lastStrideDuration,
                    g_lastStrideDx,
                    g_lastStrideDy);
    } else {
      Serial.print(" lastFpaDeg=nan");
    }

    if (dbg.last_foot_valid) {
      g_lastFootValid = true;
      g_lastFootRollDeg = dbg.last_foot_roll_deg;
      g_lastFootPitchDeg = dbg.last_foot_pitch_deg;
      g_lastFootYawDeg = dbg.last_foot_yaw_deg;
    }

  Serial.printf(" footRoll_deg=%.2f,footPitch_deg=%.2f,footYaw_deg=%.2f",
          g_lastFootValid && !isnan(g_lastFootRollDeg) ? g_lastFootRollDeg : NAN,
          g_lastFootValid && !isnan(g_lastFootPitchDeg) ? g_lastFootPitchDeg : NAN,
          g_lastFootValid && !isnan(g_lastFootYawDeg) ? g_lastFootYawDeg : NAN);

    Serial.printf(" reject=%d\n", dbg.last_reject_reason);
    g_lastDebugMs = nowMs;
    Serial.println();

  }
#endif
}
