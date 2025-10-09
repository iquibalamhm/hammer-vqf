/* ESP32 TCP client for FPA telemetry + commands
 *
 * - Connects to Wi‑Fi and to the Python server (esp_fpa_dashboard.py).
 * - Sends:
 *     footRoll_deg=...,footPitch_deg=...,footYaw_deg=...
 *     fpa_deg=...
 * - Receives commands (newline‑terminated):
 *     ze
 *     fpa.offset_deg=FLOAT
 *     fpa.median_win=INT
 *     fpa.lowpass_hz=FLOAT
 *     fpa.step_thresh_deg=FLOAT
 *
 * Replace IMU stubs with your BNO085/BNO055 read functions.
 */
#include <WiFi.h>

// --------- CONFIG ---------
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* SERVER_IP = "192.168.1.100";  // Laptop IP running esp_fpa_dashboard.py
const uint16_t SERVER_PORT = 12345;

WiFiClient client;

// --------- FPA params/state ---------
float offset_deg = 0.0f;
int   median_win = 25;
float lowpass_hz = 4.0f;
float step_thresh_deg = 8.0f;

float footRoll=0, footPitch=0, footYaw=0;
float fpa_deg = NAN;
float yaw_zero = 0.0f;

// Simple one-pole lowpass helper
float lpf_apply(float x, float y_prev, float dt, float cutoff_hz){
  if (cutoff_hz <= 0.0f) return x;
  float RC = 1.0f/(2.0f*3.1415926f*cutoff_hz);
  float a = dt/(RC+dt);
  return y_prev + a*(x - y_prev);
}

float fpa_lpf = NAN;
uint32_t last_us = 0;

// TODO: wire your real IMU read here
void imu_read(float& roll, float& pitch, float& yaw){
  // Replace with BNO085/BNO055 queries — values in degrees
  // roll, pitch, yaw = ...
  // For demo, just make yaw sweep:
  static float t=0; t += 0.02f;
  roll =  5.0f * sinf(t);
  pitch = 3.0f * cosf(0.5f*t);
  yaw = 30.0f * sinf(0.2f*t);
}

void imu_zero_yaw(){
  // For BNO085 (Orientation Estimator): store current yaw offset
  // If you have a library method to reset heading, call it instead.
  float r,p,y; imu_read(r,p,y);
  yaw_zero = y;
}

// Median filter stub (no buffer here for brevity). For real use, implement a ring buffer.
float median_stub(float x){ return x; }

void ensure_wifi(){
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Wi‑Fi connecting to %s ...\n", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nWi‑Fi OK, IP=%s\n", WiFi.localIP().toString().c_str());
}

void ensure_tcp(){
  if (client.connected()) return;
  Serial.printf("Connecting TCP %s:%u ...\n", SERVER_IP, SERVER_PORT);
  if (!client.connect(SERVER_IP, SERVER_PORT)){
    Serial.println("TCP connect failed, retrying in 1s");
    delay(1000);
  } else {
    Serial.println("TCP connected");
  }
}

void handle_command_line(String line){
  line.trim();
  if (line.length()==0) return;
  Serial.printf("cmd: %s\n", line.c_str());
  if (line.equalsIgnoreCase("ze")){
    imu_zero_yaw();
    return;
  }
  int eq = line.indexOf('=');
  if (eq>0){
    String key = line.substring(0, eq);
    String val = line.substring(eq+1);
    float f = val.toFloat();
    if (key=="fpa.offset_deg"){ offset_deg = f; }
    else if (key=="fpa.median_win"){ median_win = (int)f; }
    else if (key=="fpa.lowpass_hz"){ lowpass_hz = f; }
    else if (key=="fpa.step_thresh_deg"){ step_thresh_deg = f; }
  }
}

void read_commands(){
  static String buf;
  while (client.available()){
    char c = (char)client.read();
    if (c=='\n'){
      handle_command_line(buf);
      buf = "";
    } else if (c!='\r'){
      buf += c;
    }
  }
}

void send_line(const String& s){
  if (client.connected()){
    client.print(s);
    client.print("\n");
  }
}

void setup(){
  Serial.begin(115200);
  ensure_wifi();
  last_us = micros();
}

void loop(){
  ensure_wifi();
  ensure_tcp();
  read_commands();

  // IMU step
  uint32_t now_us = micros();
  float dt = (now_us - last_us) / 1e6f;
  if (dt < 0.005f) { delay(1); return; } // ~200 Hz max
  last_us = now_us;

  imu_read(footRoll, footPitch, footYaw);
  float yaw_rel = footYaw - yaw_zero;
  // FPA = yaw_rel + offset (replace with your true FPA computation)
  float fpa_raw = yaw_rel + offset_deg;
  float fpa_med = median_stub(fpa_raw);
  fpa_lpf = isnan(fpa_lpf) ? fpa_med : lpf_apply(fpa_med, fpa_lpf, dt, lowpass_hz);
  fpa_deg = fpa_lpf;

  // Telemetry out
  char line1[128];
  snprintf(line1, sizeof(line1), "footRoll_deg=%.3f,footPitch_deg=%.3f,footYaw_deg=%.3f", footRoll, footPitch, footYaw);
  send_line(String(line1));
  char line2[64];
  snprintf(line2, sizeof(line2), "fpa_deg=%.3f", fpa_deg);
  send_line(String(line2));

  delay(10);
}
