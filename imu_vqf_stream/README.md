# IMU VQF Stream Firmware

This PlatformIO project contains two firmware targets for the Adafruit Feather ESP32 V2 that works with a BNO08x IMU:

- **`feather-main`** – streams calibrated accelerometer, gyroscope, magnetometer data and publishes VQF-derived orientation angles.
- **`feather-scanner`** – performs an I²C bus scan to confirm the IMU address before running the main firmware.

Both targets share the same hardware wiring (SDA = GPIO23, SCL = GPIO22) and default to the BNO08x 7-bit I²C address `0x4A`.

## Prerequisites

- PlatformIO CLI installed (bundled with the PlatformIO VS Code extension or installable via `pip install platformio`).
- ESP32 Feather V2 connected over USB and put into bootloader mode if required (usually reset + boot buttons).
- BNO08x sensor connected to the board over I²C.

All commands below should be executed from the project root:

```bash
cd /home/iquibalh/Documents/PlatformIO/Projects/imu_vqf_stream
```

## Building and Uploading the Main Streaming Firmware

1. **Compile and upload** (the default environment is `feather-main`, so `-e` is optional):

   ```bash
   platformio run --target upload
   # or explicitly
   platformio run --target upload -e feather-main
   ```

2. **Open the serial monitor** to view data at 115200 baud:

   ```bash
   platformio device monitor -b 115200
   ```

   You will see CSV rows with the following columns:

   ```text
   t_ms, roll6_deg, pitch6_deg, yaw6_deg, roll6_rel_deg, pitch6_rel_deg, yaw6_rel_deg [, roll9_deg, pitch9_deg, yaw9_deg, roll9_rel_deg, pitch9_rel_deg, yaw9_rel_deg]
   ```

   - `roll/pitch/yaw6_deg` come from the 6-DoF VQF solution (gyro + accel).
   - If the magnetometer is enabled (`MAG_HZ > 0`), additional 9-DoF fields follow.
   - The `*_rel_deg` columns subtract a zero-offset captured at startup; send `z` (or `Z`) over the serial monitor to re-zero when the sensor is in the desired neutral pose.

## Running the I²C Scanner

Use the scanner firmware to confirm the IMU address if communication fails or the hardware has changed.

1. **Build and upload the scanner firmware**:

   ```bash
   platformio run --target upload -e feather-scanner
   ```

2. **Monitor the serial output** (still 115200 baud):

   ```bash
   platformio device monitor -b 115200
   ```

   The scanner prints every detected address in both 7-bit and 8-bit formats. Verify that the BNO08x appears (expected 7-bit address `0x4A`).

## Switching Between Targets

- To go back to the streaming firmware after running the scanner, either set `default_envs = feather-main` (already configured) and run `platformio run --target upload`, or explicitly include `-e feather-main`.
- You can also specify the serial port if PlatformIO does not auto-detect it, for example:

  ```bash
  platformio run --target upload -e feather-main --upload-port /dev/ttyACM0
  platformio device monitor -b 115200 -p /dev/ttyACM0
  ```

## Troubleshooting Tips

- If the ESP32 repeatedly reboots due to the watchdog, disconnect and reconnect USB, then re-upload the firmware; ensure the serial monitor is closed during flashing.
- Confirm that only one firmware (scanner *or* main) is flashed at a time, as they use different `src` filters.
- Use the scanner first whenever wiring is modified or a new sensor is attached to verify the bus address before streaming data.
