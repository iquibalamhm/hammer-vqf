#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("I2C Scanner starting...");
  Wire.begin();               // default SDA=21, SCL=22 on many ESP32 boards
  Wire.setClock(400000);
  delay(100);
}

void loop() {
  Serial.println("Scanning I2C bus...");
  for (uint8_t addr = 3; addr < 150; ++addr) {
    Wire.beginTransmission(addr);
    Wire.write(0x00);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom((int)addr, 1)) {
        uint8_t id = Wire.read();
        Serial.printf(" -> CHIP_ID = 0x%02X\n", id);
    }
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      const uint8_t addr8 = (uint8_t)(addr << 1);
      Serial.printf("I2C device found at 7-bit 0x%02X (8-bit 0x%02X)\n",
                    addr, addr8);
    } else if (err == 4) {
      Serial.printf("Unknown error at 0x%02X\n", addr);
    }
    delay(5);
  }
  Serial.println("Scan done.");
  delay(2000);
}
