#include <Arduino.h>
#include <Wire.h>

// I2C multiplexer (PCA9548A/TCA9546)
#define PCA9548A_ADDRESS 0x70

#ifndef MUX_CHANNEL_COUNT
#define MUX_CHANNEL_COUNT 4
#endif

#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN SDA
#endif

#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN SCL
#endif

static void disableMultiplexer() {
  Wire.beginTransmission(PCA9548A_ADDRESS);
  Wire.write(0x00);
  Wire.endTransmission();
}

static void selectMultiplexerChannel(uint8_t channel) {
  if (channel >= MUX_CHANNEL_COUNT) {
    return;
  }
  Wire.beginTransmission(PCA9548A_ADDRESS);
  Wire.write(1u << channel);
  Wire.endTransmission();
  delay(2);
}

static void scanChannel(uint8_t channel) {
  Serial.printf("\n=== PCA9548A channel %u ===\n", channel);
  selectMultiplexerChannel(channel);
  delay(5);

  bool foundAny = false;
  for (uint8_t addr = 3; addr < 120; ++addr) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      const uint8_t addr8 = (uint8_t)(addr << 1);
      Serial.printf("I2C device at 7-bit 0x%02X (8-bit 0x%02X)\n", addr, addr8);
      foundAny = true;
    } else if (err == 4) {
      Serial.printf("Unknown error at 0x%02X\n", addr);
    }
    delay(4);
  }

  if (!foundAny) {
    Serial.println("No devices detected on this channel.");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  delay(100);

  Serial.println("PCA9548A I2C multiplexer scanner");
  Serial.printf("Scanning %u channels...\n", MUX_CHANNEL_COUNT);
}

void loop() {
  for (uint8_t channel = 0; channel < MUX_CHANNEL_COUNT; ++channel) {
    scanChannel(channel);
  }

  disableMultiplexer();
  Serial.println("\nScan complete. Restarting in 2 seconds...\n");
  delay(2000);
}
