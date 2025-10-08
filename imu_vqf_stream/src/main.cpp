#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include <math.h>
#include <strings.h>

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

#ifndef ENABLE_TCP_STREAM
#define ENABLE_TCP_STREAM 1
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "GL-SFT1200-0ce"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "goodlife"
#endif

#ifndef TCP_SERVER_IP
#define TCP_SERVER_IP "192.168.8.158"
#endif

#ifndef TCP_SERVER_PORT
#define TCP_SERVER_PORT 12345
#endif

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 13
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
static float g_zeroRollDeg = 0.0f;
static float g_zeroPitchDeg = 0.0f;
static float g_zeroYawDeg = 0.0f;

static bool g_wifiConnected = false;
static bool g_tcpConnected = false;
static WiFiClient g_tcpClient;
static unsigned long g_lastWifiAttemptMs = 0;
static unsigned long g_lastTcpAttemptMs = 0;
static char g_tcpCmdBuf[32];
static size_t g_tcpCmdLen = 0;

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

static float wrapAngleDeg(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

static void setStatusLed(bool on) {
  digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
}

static bool getZeroedOrientation(float* outRollDeg, float* outPitchDeg, float* outYawDeg) {
  if (!g_lastFootValid || isnan(g_lastFootRollDeg) || isnan(g_lastFootPitchDeg) || isnan(g_lastFootYawDeg)) {
    if (outRollDeg) *outRollDeg = NAN;
    if (outPitchDeg) *outPitchDeg = NAN;
    if (outYawDeg) *outYawDeg = NAN;
    return false;
  }
  float roll = wrapAngleDeg(g_lastFootRollDeg - g_zeroRollDeg);
  float pitch = wrapAngleDeg(g_lastFootPitchDeg - g_zeroPitchDeg);
  float yaw = wrapAngleDeg(g_lastFootYawDeg - g_zeroYawDeg);
  if (outRollDeg) *outRollDeg = roll;
  if (outPitchDeg) *outPitchDeg = pitch;
  if (outYawDeg) *outYawDeg = yaw;
  return true;
}

static void zeroFootOrientation() {
  if (!g_lastFootValid) {
    DEBUG_PRINTLN(F("ZE command ignored: orientation invalid"));
    return;
  }
  g_zeroRollDeg = g_lastFootRollDeg;
  g_zeroPitchDeg = g_lastFootPitchDeg;
  g_zeroYawDeg = g_lastFootYawDeg;
  DEBUG_PRINTLN(F("Foot orientation zeroed"));
}

static void handleTcpCommand(const char* cmd) {
  if (!cmd || !cmd[0]) {
    return;
  }
  if (strcasecmp(cmd, "ze") == 0) {
    zeroFootOrientation();
    setStatusLed(true);
    delay(50);
    setStatusLed(g_tcpConnected);
#if ENABLE_TCP_STREAM
    if (g_tcpClient.connected()) {
      g_tcpClient.println(F("ACK ZE"));
    }
#endif
  } else {
    DEBUG_PRINT(F("Unknown TCP cmd: "));
    DEBUG_PRINTLN(cmd);
  }
}

static void pollTcpCommands() {
#if ENABLE_TCP_STREAM
  if (!g_tcpClient.connected()) {
    g_tcpCmdLen = 0;
    return;
  }
  while (g_tcpClient.available()) {
    int c = g_tcpClient.read();
    if (c < 0) {
      break;
    }
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      if (g_tcpCmdLen > 0) {
        g_tcpCmdBuf[g_tcpCmdLen] = '\0';
        handleTcpCommand(g_tcpCmdBuf);
        g_tcpCmdLen = 0;
      }
      continue;
    }
    if (g_tcpCmdLen + 1 < sizeof(g_tcpCmdBuf)) {
      g_tcpCmdBuf[g_tcpCmdLen++] = static_cast<char>(c);
    } else {
      g_tcpCmdLen = 0; // overflow, reset buffer
    }
  }
#endif
}

static void sendTcpTelemetry(float rollDeg, float pitchDeg, float yawDeg) {
#if ENABLE_TCP_STREAM
  if (!g_tcpClient.connected()) {
    return;
  }
  char line[128];
  int n = snprintf(line, sizeof(line),
                   "footRoll_deg=%.3f,footPitch_deg=%.3f,footYaw_deg=%.3f\n",
                   rollDeg, pitchDeg, yawDeg);
  if (n > 0) {
    g_tcpClient.write(reinterpret_cast<const uint8_t*>(line), static_cast<size_t>(n));
  }
#else
  (void)rollDeg;
  (void)pitchDeg;
  (void)yawDeg;
#endif
}

static void attemptTcpConnect();

static void maintainNetwork() {
#if ENABLE_TCP_STREAM
  bool wifiNow = (WiFi.status() == WL_CONNECTED);
  if (!wifiNow && (millis() - g_lastWifiAttemptMs) > 5000UL) {
    g_lastWifiAttemptMs = millis();
    WiFi.disconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
  if (wifiNow != g_wifiConnected) {
    g_wifiConnected = wifiNow;
  }

  if (g_wifiConnected) {
    if (!g_tcpClient.connected() && (millis() - g_lastTcpAttemptMs) > 2000UL) {
      attemptTcpConnect();
    }
  } else {
    if (g_tcpClient.connected()) {
      g_tcpClient.stop();
    }
  }

  bool tcpNow = g_tcpClient.connected();
  if (tcpNow != g_tcpConnected) {
    g_tcpConnected = tcpNow;
  }
  setStatusLed(g_tcpConnected);
#endif
}

static void attemptTcpConnect() {
#if ENABLE_TCP_STREAM
  g_lastTcpAttemptMs = millis();
  g_tcpClient.stop();
  if (g_tcpClient.connect(TCP_SERVER_IP, TCP_SERVER_PORT)) {
    g_tcpConnected = true;
    DEBUG_PRINTLN(F("TCP connected"));
  } else {
    g_tcpConnected = false;
    DEBUG_PRINTLN(F("TCP connect failed"));
  }
#endif
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

  float rollZero = NAN, pitchZero = NAN, yawZero = NAN;
  bool haveZero = getZeroedOrientation(&rollZero, &pitchZero, &yawZero);
  Serial.print(",footRoll_deg=");
  if (haveZero && !isnan(rollZero)) {
    Serial.print(rollZero, 3);
  } else {
    Serial.print("nan");
  }
  Serial.print(",footPitch_deg=");
  if (haveZero && !isnan(pitchZero)) {
    Serial.print(pitchZero, 3);
  } else {
    Serial.print("nan");
  }
  Serial.print(",footYaw_deg=");
  if (haveZero && !isnan(yawZero)) {
    Serial.print(yawZero, 3);
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

#if ENABLE_TCP_STREAM
  pinMode(STATUS_LED_PIN, OUTPUT);
  setStatusLed(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiStart) < 10000UL) {
    delay(100);
  }
  g_wifiConnected = (WiFi.status() == WL_CONNECTED);
  g_lastWifiAttemptMs = millis();
  if (g_wifiConnected) {
    attemptTcpConnect();
  }
  setStatusLed(g_wifiConnected && g_tcpClient.connected());
#else
  pinMode(STATUS_LED_PIN, OUTPUT);
  setStatusLed(false);
#endif

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
  maintainNetwork();
  pollTcpCommands();

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

    float rollZero = NAN, pitchZero = NAN, yawZero = NAN;
    bool haveZeroOrientation = getZeroedOrientation(&rollZero, &pitchZero, &yawZero);

    printCsvSample(t_ms);

    if (haveZeroOrientation) {
      sendTcpTelemetry(rollZero, pitchZero, yawZero);
    }

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
