# BNO085 Basic Read Example

A minimal, self-contained ESP-IDF project demonstrating the `bno085` component: reads sensor data from a BNO085 9-axis IMU over I2C and prints it as CSV to the serial console. Sensor selection, output formatting, and timing are all configured via `idf.py menuconfig` — no code changes needed.

This example depends on `bno085` as a regular [ESP-IDF Component Registry](https://components.espressif.com/) dependency (see `main/idf_component.yml`), so it also serves as a template for a project that consumes the component the way an end user would, rather than building against the driver source directly.

## Hardware Wiring

| BNO085 Pin | ESP32-S3 | Signal |
|-----------|----------|--------|
| SDA | GPIO 6 | I2C Data |
| SCL | GPIO 7 | I2C Clock |
| INT | GPIO 5 | Data-Ready (active low) |
| RST | GPIO 4 | Reset (active low) |
| VCC | 3.3V | Power |
| GND | GND | Ground |

**AD0:** GND → 0x4A, VCC → 0x4B

## Build & Flash

From inside this directory (`examples/basic_read/`):

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

**Default serial output** (AR/VR Stabilized Rotation Vector, Euler angles):
```
arvr_roll, arvr_pitch, arvr_yaw, arvr_acc
-5.99, -0.94, 4.29, 180.0
-5.99, -0.94, 4.29, 180.0
```

## Configuration

Run `idf.py menuconfig` → **BNO085 Application Configuration** to change:
- **Sensors to Enable** — which sensors are read from the device (rotation vectors, raw IMU, environmental, activity/motion detectors). Rotation vectors fuse on the BNO085 itself and don't require separately enabling the raw accelerometer/gyroscope/magnetometer.
- **Sensors to Print** — which of the enabled sensors are included in the CSV output (a sensor can be read without being printed, but not printed without being read).
- **Output Format** — CSV separator, header, optional timestamp column, and whether rotation vectors print as quaternion or Euler angles.
- **Sensor Update Period** / **Output Period** — how often the BNO085 is polled vs. how often a CSV row is printed (the latter is floored to the former, since printing faster than the sensor refreshes would just repeat the same reading).

For the full sensor list and API reference, see the [component README](../../README.md).

## Code Structure

```
main/
  ├── main.c             — I2C/GPIO setup, bno085_init(), sensor enable from Kconfig
  ├── sensor.c/h         — ISR install, sensor callback, per-sensor data cache, polling task
  ├── output.c/h         — CSV formatting and output task (separate timer from the polling task)
  └── Kconfig.projbuild  — All menuconfig options for this example
```

## License

Apache-2.0, matching the `bno085` component.
