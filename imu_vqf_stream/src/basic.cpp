// This program reads foot orientation (X-axis) using a BNO085/BNO088 IMU, provides
// real-time haptic feedback via DRV2605 drivers, supports manual and automatic
// modes, and uses an RGB LED for visual feedback. It also communicates with a
// Python server over Wi-Fi.
// Este programa lee la orientación del pie (eje X) con un IMU BNO085/BNO088,
// proporciona retroalimentación háptica en tiempo real mediante DRV2605, soporta
// modos manual y automático, y utiliza un LED RGB para retroalimentación visual.
// También se comunica con un servidor Python por Wi-Fi.

#include <Wire.h>              // EN: I2C library | ES: Librería I2C
#include <WiFi.h>              // EN: Wi-Fi support | ES: Soporte Wi-Fi
#include <Adafruit_Sensor.h>   // EN: Base sensor class | ES: Clase base de sensores
#include <Adafruit_BNO08x.h>   // EN: IMU library | ES: Librería del IMU (BNO085/BNO088)
#include <Adafruit_DRV2605.h>  // EN: Haptic driver | ES: Controlador háptico
#include <math.h>

/* ---------- Wi-Fi credentials / Credenciales ---------- */
const char *ssid = "GL-SFT1200-0ce";     // EN: Network SSID | ES: Nombre de red
const char *password = "goodlife";      // EN: Network password | ES: Contraseña
const char *server_ip = "192.168.8.175";// EN: Python server IP | ES: IP del servidor
const uint16_t server_port = 12345;      // EN: TCP port | ES: Puerto TCP
WiFiClient client;                       // EN: TCP client | ES: Cliente TCP

/* ---------- I2C multiplexer PCA9548A ---------- */
#define PCA9548A_ADDRESS 0x70
static void selectMultiplexerChannel(uint8_t ch) {
  if (ch > 7) return;                     // EN: Guard | ES: Límite
  Wire.beginTransmission(PCA9548A_ADDRESS);
  Wire.write(1 << ch);                    // EN: Enable channel | ES: Habilita canal
  Wire.endTransmission();
}

/* ---------- RGB LED (common-anode / ánodo común) ---------- */
#define LED_R 12
#define LED_G 27
#define LED_B 33
//set to 1 if the LED is active-low (common-anode), 0 for common-cathode (cmn to GND)
#define LED_ACTIVE_LOW 0
static void setColor(bool r, bool g, bool b) {
#if LED_ACTIVE_LOW
  digitalWrite(LED_R, r ? LOW : HIGH);
  digitalWrite(LED_G, g ? LOW : HIGH);
  digitalWrite(LED_B, b ? LOW : HIGH);
#else
  digitalWrite(LED_R, r ? HIGH : LOW);
  digitalWrite(LED_G, g ? HIGH : LOW);
  digitalWrite(LED_B, b ? HIGH : LOW);
#endif
}

/* ---------- Sensors / Sensores ---------- */
static Adafruit_BNO08x bno08x;            // EN: IMU on I2C | ES: IMU en I2C
static Adafruit_DRV2605 drvPos;           // EN: Right-foot LRA | ES: Vibrador derecho
static Adafruit_DRV2605 drvNeg;           // EN: Left-foot LRA | ES: Vibrador izquierdo

/* ---------- State variables / Variables de estado ---------- */
static bool manualMode = false;           // EN: Manual vs auto | ES: Modo manual o auto
static bool serverConnected = false;      // EN: TCP status | ES: Estado del socket
static bool bnoReady = false;             // EN: IMU ready flag | ES: IMU listo
static bool drvPosReady = false;
static bool drvNegReady = false;
static unsigned long lastWifiAttemptMs = 0;
static unsigned long lastSocketAttemptMs = 0;

static const unsigned long WIFI_RETRY_INTERVAL_MS = 5000;
static const unsigned long SOCKET_RETRY_INTERVAL_MS = 2000;

/* ---------- Calibration / Calibración ---------- */
static float zeroOffsetX = 0.0f;                       // EN: X offset deg | ES: Offset de X
static const float MARGIN_DEGREES = 10.0f;             // EN: Threshold ±10° | ES: Umbral ±10°
static const unsigned long pulseInterval = 10;         // EN: min 10 ms between pulses | ES: min 10 ms
static unsigned long lastPulseTime = 0;                // EN: Last vibration | ES: Última vibración
static float lastFootAngleDeg = NAN;                   // EN: Last foot angle | ES: Último ángulo
static bool footAngleValid = false;
static unsigned long lastOrientationMs = 0;
static const unsigned long ORIENTATION_TIMEOUT_MS = 500; // EN: Timeout | ES: Tiempo límite

static float normalizeAngle(float a) {                 // EN: Map 0-360→±180 | ES: Normalizar
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

/* ---------- BNO08x helpers ---------- */
static bool configureBnoReports() {
  bool ok = false;
  // Prefer game rotation vector (gravity removed), fall back to rotation vector
  if (bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000)) {
    ok = true;
  }
  if (bno08x.enableReport(SH2_ROTATION_VECTOR, 5000)) {
    ok = true;
  }
  return ok;
}

static bool extractRollDeg(const sh2_SensorValue_t &evt, float &outDeg) {
  float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
  switch (evt.sensorId) {
    case SH2_GAME_ROTATION_VECTOR: {
      const auto &rv = evt.un.gameRotationVector;
      qw = rv.real;
      qx = rv.i;
      qy = rv.j;
      qz = rv.k;
      break;
    }
    case SH2_ROTATION_VECTOR: {
      const auto &rv = evt.un.rotationVector;
      qw = rv.real;
      qx = rv.i;
      qy = rv.j;
      qz = rv.k;
      break;
    }
    default:
      return false;
  }
  const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
  const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
  const float roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / PI;
  outDeg = normalizeAngle(roll);
  return true;
}

static bool waitForOrientation(float &rollDeg, uint32_t timeoutMs) {
  if (!bnoReady) return false;
  const uint32_t start = millis();
  sh2_SensorValue_t evt;
  while ((millis() - start) < timeoutMs) {
    if (bno08x.getSensorEvent(&evt)) {
      if (extractRollDeg(evt, rollDeg)) {
        return true;
      }
    } else {
      delay(5);
    }
  }
  return false;
}

static void refreshFootAngle() {
  if (!bnoReady) return;
  selectMultiplexerChannel(0);
  sh2_SensorValue_t evt;
  bool updated = false;
  while (bno08x.getSensorEvent(&evt)) {
    float angleDeg = 0.0f;
    if (extractRollDeg(evt, angleDeg)) {
      lastFootAngleDeg = normalizeAngle(angleDeg - zeroOffsetX);
      footAngleValid = true;
      lastOrientationMs = millis();
      updated = true;
    }
  }
  if (!updated) {
    if (footAngleValid && (millis() - lastOrientationMs) > ORIENTATION_TIMEOUT_MS) {
      footAngleValid = false;
      lastFootAngleDeg = NAN;
    }
  }
}

/* ---------- Networking helpers / Ayudas de red ---------- */
static void maintainSocket() {
  wl_status_t wifiStatus = WiFi.status();
  if (wifiStatus != WL_CONNECTED) {
    if (serverConnected) {
      client.stop();
      serverConnected = false;
    }
    unsigned long now = millis();
    if (now - lastWifiAttemptMs >= WIFI_RETRY_INTERVAL_MS) {
      lastWifiAttemptMs = now;
      Serial.println(F("[info] WiFi reconnect attempt"));
      WiFi.disconnect(true);
      delay(50);
      WiFi.begin(ssid, password);
    }
    return;
  }

  if (!client.connected()) {
    serverConnected = false;
    unsigned long now = millis();
    if (now - lastSocketAttemptMs >= SOCKET_RETRY_INTERVAL_MS) {
      lastSocketAttemptMs = now;
      Serial.print(F("[info] Connecting to TCP server "));
      Serial.print(server_ip);
      Serial.print(F(":"));
      Serial.print(server_port);
      if (client.connect(server_ip, server_port)) {
        serverConnected = true;
        Serial.println(F(" ... connected"));
      } else {
        Serial.println(F(" ... failed"));
      }
    }
  } else {
    serverConnected = true;
  }
}

/* ---------- Robust calibration / Calibración robusta ---------- */
static void calibrateIMU() {
  setColor(true, true, true);              // EN: White = calibrating | ES: Blanco
  for (int i = 3; i > 0; --i) delay(1000); // EN: 3-s countdown | ES: Cuenta atrás

  bool ok = true;

  /* --- Read IMU --- */
  selectMultiplexerChannel(0);
  float rollDeg = 0.0f;
  if (waitForOrientation(rollDeg, 2000)) {
    zeroOffsetX = rollDeg;
  } else {
    Serial.println(F("[warn] BNO08x orientation timeout during calibration"));
    ok = false;
  }

  /* --- Init DRV2605 drivers / Inicializar DRV2605 --- */
  selectMultiplexerChannel(3);
  if (!drvPosReady) {
    drvPosReady = drvPos.begin();
    if (!drvPosReady) {
      Serial.println(F("[warn] DRV2605 channel 3 not detected"));
      // setColor(true,false,false); // Temporarily disabled: focusing on BNO status
    }
    if (drvPosReady) {
      drvPos.selectLibrary(1);
      drvPos.setMode(DRV2605_MODE_REALTIME);
    }
  }

  selectMultiplexerChannel(4);
  if (!drvNegReady) {
    drvNegReady = drvNeg.begin();
    if (!drvNegReady) {
      Serial.println(F("[warn] DRV2605 channel 4 not detected"));
      // setColor(true,false,false); // Temporarily disabled: focusing on BNO status
    }
    if (drvNegReady) {
      drvNeg.selectLibrary(1);
      drvNeg.setMode(DRV2605_MODE_REALTIME);
    }
  }

  /* --- Re-open socket if Wi-Fi up / Reabrir socket si hay Wi-Fi --- */
  maintainSocket();

  /* --- Final LED status / LED final --- */
  if (!ok) {
    Serial.println(F("[warn] Calibration finished with warnings"));
  }
  if (manualMode) {
    setColor(true, true, false);           // EN: Yellow | ES: Amarillo
  } else if (serverConnected) {
    setColor(false, true, false);          // EN: Green | ES: Verde
  } else {
    setColor(false, true, true);           // EN: Cyan | ES: Cian
  }
}

/* ---------- setup ---------- */
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Wire.begin();
  Wire.setClock(400000);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  setColor(false, false, false); // OFF

  /* Wi-Fi with 5-s timeout / Wi-Fi con 5 s de espera */
  WiFi.begin(ssid, password);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 5000) {
    setColor(false, true, true);  delay(250);   // cyan blink
    setColor(false, false, false); delay(250);  // off
  }
  maintainSocket();

  if (serverConnected) {
    setColor(false, true, false);              // green solid
  } else if (WiFi.status() == WL_CONNECTED) {
    setColor(false, true, true);               // cyan solid
  } else {
    setColor(true, true, false);               // amber to indicate local-only
  }

  /* Init IMU / Inicializar IMU */
  selectMultiplexerChannel(0);
  const uint8_t possibleAddrs[] = {0x4A, 0x4B};
  for (uint8_t addr : possibleAddrs) {
    if (bno08x.begin_I2C(addr, &Wire)) {
      bnoReady = true;
      Serial.print(F("[info] BNO08x ready at 0x"));
      Serial.println(addr, HEX);
      break;
    }
  }
  if (!bnoReady) {
    Serial.println(F("[error] BNO08x not detected. Check wiring and multiplexer channel."));
  } else {
    if (!configureBnoReports()) {
      Serial.println(F("[warn] Failed to enable rotation vector reports"));
    }
  }

  /* Init drivers / Inicializar drivers */
  selectMultiplexerChannel(3);
  drvPosReady = drvPos.begin();
  if (!drvPosReady) {
    Serial.println(F("[warn] DRV2605 channel 3 not detected during setup"));
    // setColor(true,false,false); // Temporarily disabled: focusing on BNO status
  } else {
    drvPos.selectLibrary(1);
    drvPos.setMode(DRV2605_MODE_REALTIME);
  }

  selectMultiplexerChannel(4);
  drvNegReady = drvNeg.begin();
  if (!drvNegReady) {
    Serial.println(F("[warn] DRV2605 channel 4 not detected during setup"));
    // setColor(true,false,false); // Temporarily disabled: focusing on BNO status
  } else {
    drvNeg.selectLibrary(1);
    drvNeg.setMode(DRV2605_MODE_REALTIME);
  }

  calibrateIMU();                           // first calibration
}

/* ---------- loop ---------- */
void loop() {
  selectMultiplexerChannel(0);
  if (bnoReady && bno08x.wasReset()) {
    Serial.println(F("[info] BNO08x reported reset; reconfiguring reports"));
    configureBnoReports();
  }

  refreshFootAngle();

  if (footAngleValid && client.connected()) {
    client.print(String(lastFootAngleDeg, 3) + ",0,0\n");
  }

  /* Automatic vibration / Vibración automática */
  const unsigned long now = millis();
  bool vibrated = false;
  if (!manualMode && footAngleValid && (now - lastPulseTime) >= pulseInterval) {
    if (lastFootAngleDeg > MARGIN_DEGREES && drvPosReady) {
      selectMultiplexerChannel(3);
      drvPos.setRealtimeValue(127);
      setColor(false, false, true); // blue
      delay(1);
      drvPos.setRealtimeValue(0);
      lastPulseTime = now;
      vibrated = true;
    } else if (lastFootAngleDeg < -MARGIN_DEGREES && drvNegReady) {
      selectMultiplexerChannel(4);
      drvNeg.setRealtimeValue(127);
      setColor(true, false, true); // magenta
      delay(1);
      drvNeg.setRealtimeValue(0);
      lastPulseTime = now;
      vibrated = true;
    }
    if (vibrated) {
      selectMultiplexerChannel(0);
    }
  }

  /* Remote commands / Comandos remotos */
  maintainSocket();
  if (client.available()) {
    String cmd = client.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("MANUAL")) {
      manualMode = true;
      setColor(true, true, false); // yellow
    } else if (cmd.equalsIgnoreCase("AUTOMATICO")) {
      manualMode = false;
      setColor(!serverConnected, true, false);
    } else if (cmd.equalsIgnoreCase("VIBRA_NEG") && manualMode && drvNegReady) {
      setColor(true, false, true); // magenta
      selectMultiplexerChannel(4);
      for (int i = 0; i < 3; ++i) {
        drvNeg.setRealtimeValue(127); delay(10);
        drvNeg.setRealtimeValue(0);   delay(100);
      }
      selectMultiplexerChannel(0);
      setColor(true, true, false); // yellow
    } else if (cmd.equalsIgnoreCase("VIBRA_POS") && manualMode && drvPosReady) {
      setColor(false, false, true); // blue
      selectMultiplexerChannel(3);
      for (int i = 0; i < 3; ++i) {
        drvPos.setRealtimeValue(127); delay(10);
        drvPos.setRealtimeValue(0);   delay(100);
      }
      selectMultiplexerChannel(0);
      setColor(true, true, false); // yellow
    } else if (cmd.equalsIgnoreCase("CALIBRAR")) {
      calibrateIMU();
    }
  }

  /* Restore LED if not vibrating / Restaurar LED */
  if (!vibrated) {
    if (!manualMode) {
  if (serverConnected) setColor(false, true, false); // green
  else setColor(false, true, true);                  // cyan
    } else {
      setColor(true, true, false);                       // yellow
    }
  }

  delay(10);                                // EN: 100 Hz loop | ES: Bucle 100 Hz
}
