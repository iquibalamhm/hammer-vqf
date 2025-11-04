// SPDX-FileCopyrightText: 2022 Limor Fried for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#include <Arduino.h>
#include "Adafruit_MAX1704X.h"
#include "Adafruit_LC709203F.h"
#include <Adafruit_NeoPixel.h>
#include "Adafruit_TestBed.h"
#include <Adafruit_BME280.h>
#include <Adafruit_ST7789.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Wire.h>


#define PCA9548A_ADDRESS 0x70

Adafruit_BME280 bme; // I2C
bool bmefound = false;
extern Adafruit_TestBed TB;

Adafruit_LC709203F lc_bat;
Adafruit_MAX17048 max_bat;

Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

GFXcanvas16 canvas(240, 135);

bool maxfound = false;
bool lcfound = false;

#ifndef MUX_CHANNEL_COUNT
#define MUX_CHANNEL_COUNT 8
#endif

static constexpr uint8_t MAX_DEVICES_PER_BUS = 16;

static bool muxPresent = false;
static int8_t selectedMuxChannel = -1;
static int8_t bnoMuxChannel = -1;
static uint8_t bnoAddress = 0;
static int8_t lastReportedBnoChannel = -2;
static uint8_t lastReportedBnoAddress = 0;

static uint8_t baseAddresses[MAX_DEVICES_PER_BUS];
static uint8_t baseDeviceCount = 0;
static uint8_t muxAddresses[MUX_CHANNEL_COUNT][MAX_DEVICES_PER_BUS];
static uint8_t muxDeviceCount[MUX_CHANNEL_COUNT];

static bool probeI2C(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

static bool isBno08x(uint8_t address) {
  return address == 0x4A || address == 0x4B;
}

static void disableMux() {
  Wire.beginTransmission(PCA9548A_ADDRESS);
  Wire.write(0x00);
  Wire.endTransmission();
  selectedMuxChannel = -1;
}

static bool selectMuxChannel(uint8_t channel) {
  if (!muxPresent || channel >= MUX_CHANNEL_COUNT) {
    return false;
  }
  Wire.beginTransmission(PCA9548A_ADDRESS);
  if (Wire.write(1u << channel) != 1) {
    Wire.endTransmission();
    return false;
  }
  if (Wire.endTransmission() != 0) {
    return false;
  }
  selectedMuxChannel = static_cast<int8_t>(channel);
  delay(3);
  return true;
}

static uint8_t scanBus(int8_t channel, uint8_t *dest, uint8_t maxCount) {
  if (channel < 0) {
    if (muxPresent && selectedMuxChannel != -1) {
      disableMux();
      delay(2);
    }
  } else {
    if (!selectMuxChannel(static_cast<uint8_t>(channel))) {
      return 0;
    }
  }

  uint8_t found = 0;
  for (uint8_t addr = 3; addr < 0x78; ++addr) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      if (found < maxCount) {
        dest[found++] = addr;
      }
    } else if (err == 4) {
      Serial.print(F("Unknown error at 0x"));
      if (addr < 16) {
        Serial.print('0');
      }
      Serial.println(addr, HEX);
    }
    delay(2);
  }
  return found;
}

static void scanI2CBuses() {
  muxPresent = probeI2C(PCA9548A_ADDRESS);
  if (!muxPresent) {
    selectedMuxChannel = -1;
  }

  baseDeviceCount = scanBus(-1, baseAddresses, MAX_DEVICES_PER_BUS);
  bnoMuxChannel = -1;
  bnoAddress = 0;

  for (uint8_t i = 0; i < baseDeviceCount; ++i) {
    if (isBno08x(baseAddresses[i])) {
      bnoMuxChannel = -1;
      bnoAddress = baseAddresses[i];
    }
  }

  if (muxPresent) {
    for (uint8_t ch = 0; ch < MUX_CHANNEL_COUNT; ++ch) {
      muxDeviceCount[ch] = scanBus(static_cast<int8_t>(ch), muxAddresses[ch], MAX_DEVICES_PER_BUS);
      for (uint8_t idx = 0; idx < muxDeviceCount[ch]; ++idx) {
        if (isBno08x(muxAddresses[ch][idx])) {
          bnoMuxChannel = static_cast<int8_t>(ch);
          bnoAddress = muxAddresses[ch][idx];
        }
      }
    }
  } else {
    for (uint8_t ch = 0; ch < MUX_CHANNEL_COUNT; ++ch) {
      muxDeviceCount[ch] = 0;
    }
  }

  if (bnoMuxChannel >= 0) {
    selectMuxChannel(static_cast<uint8_t>(bnoMuxChannel));
  } else if (muxPresent) {
    disableMux();
  }

  if (bnoMuxChannel != lastReportedBnoChannel || bnoAddress != lastReportedBnoAddress) {
    if (bnoAddress != 0) {
      if (bnoMuxChannel >= 0) {
        Serial.print(F("BNO08x detected on mux channel "));
        Serial.print(bnoMuxChannel);
        Serial.print(F(" at 0x"));
        if (bnoAddress < 16) {
          Serial.print('0');
        }
        Serial.println(bnoAddress, HEX);
      } else {
        Serial.print(F("BNO08x detected on base bus at 0x"));
        if (bnoAddress < 16) {
          Serial.print('0');
        }
        Serial.println(bnoAddress, HEX);
      }
    } else {
      Serial.println(F("BNO08x not detected."));
    }
    lastReportedBnoChannel = bnoMuxChannel;
    lastReportedBnoAddress = bnoAddress;
  }
}

static void printAddressList(int8_t channel, const uint8_t *addresses, uint8_t count) {
  if (count == 0) {
    canvas.setTextColor(ST77XX_WHITE);
    canvas.println("--");
    return;
  }

  for (uint8_t i = 0; i < count; ++i) {
    const bool isBno = (channel == bnoMuxChannel && addresses[i] == bnoAddress);
    if (isBno) {
      canvas.setTextColor(ST77XX_CYAN);
    } else {
      canvas.setTextColor(ST77XX_WHITE);
    }
    canvas.print("0x");
    if (addresses[i] < 16) {
      canvas.print('0');
    }
    canvas.print(addresses[i], HEX);
    if (isBno) {
      canvas.print('*');
    }
    canvas.print(' ');
  }
  canvas.println();
  canvas.setTextColor(ST77XX_WHITE);
}

void setup() {
  Serial.begin(115200);
  // while (! Serial) delay(10);
  
  delay(100);
  
  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);

  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
  delay(10);
  
  TB.neopixelPin = PIN_NEOPIXEL;
  TB.neopixelNum = 1; 
  TB.begin();
  TB.setColor(WHITE);

  display.init(135, 240);           // Init ST7789 240x135
  display.setRotation(3);
  canvas.setFont(&FreeSans12pt7b);
  canvas.setTextColor(ST77XX_WHITE); 

   if (lc_bat.begin()) {
    Serial.println("Found LC709203F");
    Serial.print("Version: 0x"); Serial.println(lc_bat.getICversion(), HEX);
    lc_bat.setPackSize(LC709203F_APA_500MAH);
    lcfound = true;
  }
  else {
    Serial.println(F("Couldnt find Adafruit LC709203F?\nChecking for Adafruit MAX1704X.."));
    delay(200);
    if (!max_bat.begin()) {
      Serial.println(F("Couldnt find Adafruit MAX1704X?\nMake sure a battery is plugged in!"));
      while (1) delay(10);
    }
    Serial.print(F("Found MAX17048"));
    Serial.print(F(" with Chip ID: 0x")); 
    Serial.println(max_bat.getChipID(), HEX);
    maxfound = true;
    
  } 

  Wire.begin();
  Wire.setClock(400000);

  if (TB.scanI2CBus(0x77)) {
    Serial.println("BME280 address");

    unsigned status = bme.begin();  
    if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      return;
    }
    Serial.println("BME280 found OK");
    bmefound = true;
  }
}

uint8_t j = 0;

void loop() {

  if (j % 5 == 0) {
    Serial.println("**********************");

    scanI2CBuses();
    TB.printI2CBusScan();
  canvas.fillScreen(ST77XX_BLACK);
  canvas.setFont(&FreeSans12pt7b);
    canvas.setCursor(0, 25);
    canvas.setTextColor(ST77XX_RED);
    canvas.println("Adafruit Feather");
    canvas.setTextColor(ST77XX_YELLOW);
    canvas.println("ESP32-S2 TFT Demo");
    canvas.setTextColor(ST77XX_GREEN); 
    canvas.print("Battery: ");
    canvas.setTextColor(ST77XX_WHITE);
    if (lcfound == true) {
      canvas.print(lc_bat.cellVoltage(), 1);
      canvas.print(" V  /  ");
      canvas.print(lc_bat.cellPercent(), 0);
      canvas.println("%");
    }
    else {
      canvas.print(max_bat.cellVoltage(), 1);
      canvas.print(" V  /  ");
      canvas.print(max_bat.cellPercent(), 0);
      canvas.println("%");
    }
    canvas.setFont();
    canvas.setCursor(0, 92);
    canvas.setTextColor(ST77XX_BLUE);
    canvas.println("I2C scan:");
    canvas.setTextColor(ST77XX_GREEN);
    canvas.print("BNO : ");
    if (bnoAddress != 0) {
      if (bnoMuxChannel >= 0) {
        canvas.print("CH");
        canvas.print(bnoMuxChannel);
        canvas.print(" 0x");
      } else {
        canvas.print("Base 0x");
      }
      if (bnoAddress < 16) {
        canvas.print('0');
      }
      canvas.print(bnoAddress, HEX);
      canvas.println();
    } else {
      canvas.println("--");
    }
    canvas.setTextColor(ST77XX_WHITE);
    canvas.print("Base : ");
    printAddressList(-1, baseAddresses, baseDeviceCount);
    if (muxPresent) {
      for (uint8_t ch = 0; ch < MUX_CHANNEL_COUNT; ++ch) {
        canvas.print("CH");
        canvas.print(ch);
        canvas.print(" : ");
        printAddressList(static_cast<int8_t>(ch), muxAddresses[ch], muxDeviceCount[ch]);
      }
    } else {
      canvas.println("No PCA9548A detected.");
    }
    display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);
  }
  
  TB.setColor(TB.Wheel(j++));
  return;
}