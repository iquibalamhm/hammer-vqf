#include <Arduino.h>
#include <Wire.h>

// I2C multiplexer (PCA9548A/TCA9546)
#define PCA9548A_ADDRESS 0x70

#ifndef MUX_CHANNEL_COUNT
#define MUX_CHANNEL_COUNT 8
#endif

#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN SDA
#endif

#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN SCL
#endif

static bool muxPresent = false;
static int8_t selectedChannel = -1;
static int8_t bnoChannel = -1;
static uint8_t bnoAddress = 0;

static bool probeAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

static bool disableMultiplexer() {
  Wire.beginTransmission(PCA9548A_ADDRESS);
  bool ok = Wire.write(0x00) == 1 && Wire.endTransmission() == 0;
  if (ok) {
    selectedChannel = -1;
  }
  delay(2);
  return ok;
}

static bool selectMultiplexerChannel(uint8_t channel) {
  if (!muxPresent || channel >= MUX_CHANNEL_COUNT) {
    return false;
  }
  Wire.beginTransmission(PCA9548A_ADDRESS);
  bool ok = Wire.write(1u << channel) == 1 && Wire.endTransmission() == 0;
  if (ok) {
    selectedChannel = static_cast<int8_t>(channel);
    delay(3);
  }
  return ok;
}

static bool isBno08x(uint8_t address) {
  return address == 0x4A || address == 0x4B;
}

static void scanBaseBus() {
  Serial.println("\n=== Base I2C bus ===");
  if (muxPresent && selectedChannel != -1) {
    disableMultiplexer();
  }

  bool foundAny = false;
  for (uint8_t addr = 3; addr < 120; ++addr) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      const uint8_t addr8 = static_cast<uint8_t>(addr << 1);
      Serial.printf("I2C device at 7-bit 0x%02X (8-bit 0x%02X)\n", addr, addr8);
      foundAny = true;
      if (isBno08x(addr)) {
        bnoChannel = -1;
        bnoAddress = addr;
      }
    } else if (err == 4) {
      Serial.printf("Unknown error at 0x%02X\n", addr);
    }
    delay(4);
  }

  if (!foundAny) {
    Serial.println("No devices detected on the base bus.");
  }

  muxPresent = probeAddress(PCA9548A_ADDRESS);
  if (muxPresent) {
    Serial.println("PCA9548A multiplexer detected at 0x70.");
  } else {
    Serial.println("No PCA9548A multiplexer detected.");
  }
}

static void scanChannel(uint8_t channel) {
  Serial.printf("\n=== PCA9548A channel %u ===\n", channel);
  if (!selectMultiplexerChannel(channel)) {
    Serial.println("Failed to select multiplexer channel.");
    return;
  }

  bool foundAny = false;
  for (uint8_t addr = 3; addr < 120; ++addr) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      const uint8_t addr8 = static_cast<uint8_t>(addr << 1);
      Serial.printf("I2C device at 7-bit 0x%02X (8-bit 0x%02X)\n", addr, addr8);
      foundAny = true;
      if (isBno08x(addr)) {
        bnoChannel = static_cast<int8_t>(channel);
        bnoAddress = addr;
      }
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
  bnoChannel = -1;
  bnoAddress = 0;

  scanBaseBus();

  if (muxPresent) {
    for (uint8_t channel = 0; channel < MUX_CHANNEL_COUNT; ++channel) {
      scanChannel(channel);
    }

    if (bnoChannel >= 0) {
      selectMultiplexerChannel(static_cast<uint8_t>(bnoChannel));
      Serial.print("\nSelecting BNO08x on channel ");
      Serial.print(bnoChannel);
      Serial.print(" at 0x");
      if (bnoAddress < 16) {
        Serial.print('0');
      }
      Serial.println(bnoAddress, HEX);
    } else {
      disableMultiplexer();
      Serial.println("\nBNO08x not found on any channel.");
    }
  }

  Serial.println("\nScan complete. Restarting in 2 seconds...\n");
  delay(2000);
}
