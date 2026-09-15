# BNO085 IMU Driver for ESP-IDF

ESP-IDF component for the Bosch BNO085 9-axis IMU over I2C using CEVA's SH2 sensor hub protocol.

## Features

- 35+ sensors (orientation, raw IMU, environmental, activity detection, gesture detection)
- Multiple rotation vector types (standard, game, geomagnetic, AR/VR optimized, 1kHz gyro-integrated)
- Up to 8 concurrent sensors on the same I2C bus (configurable)
- Flexible I2C addressing (0x28 or 0x29 via AD0 pin)
- Multiplexer support for 8+ sensors
- Handle-based API with callback-based data delivery
- Full sensor decoding into named fields

## Hardware Setup

### Wiring Example (Heltec LoRa V3)

| BNO085 Pin | ESP32-S3 Pin | Signal       | Notes           |
|------------|--------------|--------------|-----------------|
| SDA        | GPIO 6       | I2C SDA      | Pull-up enabled |
| SCL        | GPIO 7       | I2C SCL      | Pull-up enabled |
| INT (H_INTN)| GPIO 5      | Interrupt    | Active-low      |
| RST (NRST) | GPIO 4       | Reset        | Active-low      |
| AD0        | GND          | Slave addr   | I2C addr 0x28   |
| VCC        | 3.3V         | Power        |                 |
| GND        | GND          | Ground       |                 |

**Important**: PS0 and PS1 must be tied to GND for I2C mode.

### I2C Address Selection

The BNO085 has an AD0 pin that selects the I2C address:
- AD0 = GND (0V) → I2C address 0x28
- AD0 = 3.3V → I2C address 0x29

Create the I2C device handle with the correct address matching your AD0 pin configuration.

### Multiple Sensors

Use an I2C multiplexer (e.g., TCA9548A) to connect multiple BNO085 sensors on different channels. Initialize one handle per sensor and select the multiplexer channel before calling `bno085_service()` on each handle.

## Installation

### From ESP-IDF Component Registry

```bash
idf.py add-dependency bno085
```

### From Git (Development)

```bash
git clone --recursive https://github.com/<your-username>/bno085.git components/bno085
```

## Quick Start

```c
#include "bno085.h"
#include "driver/i2c_master.h"
#include "freertos/task.h"

static void on_sensor_data(bno085_handle_t handle, const bno085_sensor_value_t *value, void *ctx)
{
    if (value->sensor_id == BNO085_SENSOR_ROTATION_VECTOR) {
        printf("Quaternion: i=%.4f, j=%.4f, k=%.4f, real=%.4f\n",
               value->data.quaternion.i, value->data.quaternion.j,
               value->data.quaternion.k, value->data.quaternion.real);
    }
}

void app_main(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_6,
        .scl_io_num = GPIO_NUM_7,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus(&bus_config, &bus_handle);

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x28,  // AD0 = GND
        .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t i2c_dev;
    i2c_master_bus_add_device(bus_handle, &dev_config, &i2c_dev);

    bno085_handle_t bno085;
    bno085_init(NULL, i2c_dev, GPIO_NUM_5, GPIO_NUM_4, &bno085);  // NULL = default config
    bno085_register_sensor_callback(bno085, on_sensor_data, NULL);
    bno085_enable_sensor(bno085, BNO085_SENSOR_ROTATION_VECTOR, 100000);  // 10Hz

    while (1) {
        bno085_service(bno085);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

## API Reference

### Initialization

```c
esp_err_t bno085_init(const bno085_config_t *config,
                      i2c_master_dev_handle_t i2c_dev,
                      gpio_num_t int_pin,
                      gpio_num_t reset_pin,
                      bno085_handle_t *out_handle);
```

Initialize a BNO085 instance. Up to `CONFIG_BNO085_MAX_INSTANCES` instances may be open concurrently (default 2, configurable 1-8) — see Limitations below.

### Service Loop

```c
void bno085_service(bno085_handle_t handle);
```

Call periodically (10ms recommended) to read sensor data and dispatch callbacks.

### Cleanup

```c
void bno085_deinit(bno085_handle_t handle);
```

Close the device and free resources.

### Callbacks

```c
esp_err_t bno085_register_sensor_callback(bno085_handle_t handle,
                                          bno085_sensor_callback_t callback,
                                          void *user_context);
```

Register a callback function. Replaces any previous callback.

### Quaternion to Euler Conversion

```c
void bno085_quaternion_to_euler(float i, float j, float k, float real,
                                float *out_roll, float *out_pitch, float *out_yaw);
```

Pure math utility — no handle or hardware required. Converts a quaternion to roll/pitch/yaw
(radians). Works with any of the rotation-vector-shaped union members (`.quaternion`,
`.game_rotation_vector`, `.geomagnetic_rotation_vector`, `.ar_vr_stabilized_rv`,
`.gyro_integrated_rv`), which all share the same `{i, j, k, real}` layout. Pass `NULL` for any
output you don't need.

```c
static void sensor_callback(bno085_handle_t handle,
                           const bno085_sensor_value_t *value,
                           void *user_context)
{
    if (value->sensor_id == BNO085_SENSOR_ROTATION_VECTOR) {
        float roll, pitch, yaw;
        bno085_quaternion_to_euler(value->data.quaternion.i, value->data.quaternion.j,
                                   value->data.quaternion.k, value->data.quaternion.real,
                                   &roll, &pitch, &yaw);
        printf("Roll=%.2f Pitch=%.2f Yaw=%.2f (deg)\n",
               roll * 57.2958f, pitch * 57.2958f, yaw * 57.2958f);
    }
}
```

### Sensor Control

```c
esp_err_t bno085_enable_sensor(bno085_handle_t handle,
                               uint8_t sensor_id,
                               uint32_t report_interval_us);

esp_err_t bno085_disable_sensor(bno085_handle_t handle,
                                uint8_t sensor_id);
```

Enable/disable sensors with specified reporting interval in microseconds:
- 10000 = 10ms (100Hz)
- 100000 = 100ms (10Hz)
- 1000000 = 1000ms (1Hz)

### Configuration

```c
typedef struct {
    uint32_t i2c_timeout_ms;        // I2C operation timeout (default: 100ms)
    uint8_t  reset_retry_count;     // Soft-reset retries (default: 5)
    uint32_t reset_retry_delay_ms;  // Delay between retries (default: 30ms)
    uint32_t reset_settle_delay_ms; // Delay after reset (default: 300ms)
} bno085_config_t;

void bno085_config_default(bno085_config_t *config);
```

## Available Sensors

### Orientation Sensors

| Sensor ID | Constant | Field (`value->data.`) | Output | Use Case |
|-----------|----------|--------|--------|----------|
| 0x05 | `BNO085_SENSOR_ROTATION_VECTOR` | `.quaternion` | Quaternion | Most accurate, AR/VR, drones |
| 0x08 | `BNO085_SENSOR_GAME_ROTATION_VECTOR` | `.game_rotation_vector` | Quaternion | Games (no mag yaw drift) |
| 0x09 | `BNO085_SENSOR_GEOMAGNETIC_ROTATION_VECTOR` | `.geomagnetic_rotation_vector` | Quaternion | Low power, stationary devices |
| 0x28 | `BNO085_SENSOR_AR_VR_STABILIZED_RV` | `.ar_vr_stabilized_rv` | Quaternion | High-quality AR/VR |
| 0x2A | `BNO085_SENSOR_GYRO_INTEGRATED_RV` | `.gyro_integrated_rv` | Quaternion | Head tracking (1kHz) |

All five quaternion fields share the same `{i, j, k, real, accuracy_rad}` layout. Convert to Euler with `bno085_quaternion_to_euler()` (see above) — the driver doesn't auto-convert.

### Raw IMU Data

| Sensor ID | Constant | Field (`value->data.`) | Output | Range |
|-----------|----------|--------|--------|-------|
| 0x01 | `BNO085_SENSOR_ACCELEROMETER` | `.accelerometer` | x, y, z (m/s²) | ±8g |
| 0x02 | `BNO085_SENSOR_GYROSCOPE_CALIBRATED` | `.gyroscope_calibrated` | x, y, z (rad/s) | Calibrated |
| 0x03 | `BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED` | `.magnetic_field_calibrated` | x, y, z (µT) | Calibrated |
| 0x04 | `BNO085_SENSOR_LINEAR_ACCELERATION` | `.linear_acceleration` | x, y, z (m/s²) | Gravity removed |
| 0x06 | `BNO085_SENSOR_GRAVITY` | `.gravity` | x, y, z (m/s²) | Gravity vector only |
| 0x07 | `BNO085_SENSOR_GYROSCOPE_UNCALIBRATED` | `.gyroscope_uncalibrated` | x, y, z + bias_x/y/z | Raw with bias |
| 0x0F | `BNO085_SENSOR_MAGNETIC_FIELD_UNCALIBRATED` | `.magnetic_field_uncalibrated` | x, y, z + bias_x/y/z | Raw with bias |
| 0x14 | `BNO085_SENSOR_RAW_ACCELEROMETER` | `.raw_accelerometer` | x, y, z (`int16_t`) | ADC counts |
| 0x15 | `BNO085_SENSOR_RAW_GYROSCOPE` | `.raw_gyroscope` | x, y, z (`int16_t`) | ADC counts |
| 0x16 | `BNO085_SENSOR_RAW_MAGNETOMETER` | `.raw_magnetometer` | x, y, z (`int16_t`) | ADC counts |

### Motion Detection & Activity

| Sensor ID | Constant | Field (`value->data.`) | Description |
|-----------|----------|--------|-------------|
| 0x10 | `BNO085_SENSOR_TAP_DETECTOR` | `.tap_detector` — `{tap_type, direction}` | `tap_type`: 1=single, 2=double |
| 0x11 | `BNO085_SENSOR_STEP_COUNTER` | `.step_counter` — `{count}` | Cumulative step count since reset |
| 0x12 | `BNO085_SENSOR_SIGNIFICANT_MOTION` | `.significant_motion` — `{event}` | Significant motion event |
| 0x13 | `BNO085_SENSOR_STABILITY_CLASSIFIER` | `.stability_classifier` — `{activity}` | 0=unknown, 1=on-table, 2=stationary, 3=stable, 4=motion |
| 0x18 | `BNO085_SENSOR_STEP_DETECTOR` | `.step_detector` — `{event}` | Fires on each step |
| 0x19 | `BNO085_SENSOR_SHAKE_DETECTOR` | `.shake_detector` — `{event}` | Shake/vibration detection |
| 0x1A | `BNO085_SENSOR_FLIP_DETECTOR` | `.flip_detector` — `{event}` | Device flip motion |
| 0x1B | `BNO085_SENSOR_PICKUP_DETECTOR` | `.pickup_detector` — `{event}` | Device pickup detection |
| 0x1E | `BNO085_SENSOR_PERSONAL_ACTIVITY_CLASSIFIER` | `.personal_activity_classifier` — `{activity}` | 0=unknown, 1=still, 2=walking, 3=running, 4=on-bicycle, 5=in-vehicle, etc. |
| 0x1F | `BNO085_SENSOR_SLEEP_DETECTOR` | `.sleep_detector` — `{event}` | Sleep state detection |
| 0x20 | `BNO085_SENSOR_TILT_DETECTOR` | `.tilt_detector` — `{event}` | Tilt change detection |
| 0x21 | `BNO085_SENSOR_POCKET_DETECTOR` | `.pocket_detector` — `{event}` | In-pocket detection |
| 0x22 | `BNO085_SENSOR_CIRCLE_DETECTOR` | `.circle_detector` — `{event}` | Circular motion detection |

### Environmental Sensors

*(not all BNO085 module variants are equipped with these)*

| Sensor ID | Constant | Field (`value->data.`) | Output | Units |
|-----------|----------|--------|--------|-------|
| 0x0A | `BNO085_SENSOR_PRESSURE` | `.pressure` — `{value}` | Atmospheric pressure | Pa |
| 0x0B | `BNO085_SENSOR_AMBIENT_LIGHT` | `.ambient_light` — `{value}` | Light intensity | lux |
| 0x0C | `BNO085_SENSOR_HUMIDITY` | `.humidity` — `{value}` | Relative humidity | % |
| 0x0E | `BNO085_SENSOR_TEMPERATURE` | `.temperature` — `{value}` | Ambient temperature | °C |

## Usage Examples

### Get Orientation Only

```c
void sensor_callback(bno085_handle_t handle,
                    const bno085_sensor_value_t *value,
                    void *user_context)
{
    if (value->sensor_id == BNO085_SENSOR_ROTATION_VECTOR) {
        printf("Q: i=%f, j=%f, k=%f, real=%f, acc=%f rad\n",
               value->data.quaternion.i,
               value->data.quaternion.j,
               value->data.quaternion.k,
               value->data.quaternion.real,
               value->data.quaternion.accuracy_rad);
    }
}

bno085_register_sensor_callback(bno085, sensor_callback, NULL);
bno085_enable_sensor(bno085, BNO085_SENSOR_ROTATION_VECTOR, 100000);
```

### Get All IMU Data

```c
bno085_register_sensor_callback(bno085, sensor_callback, NULL);
bno085_enable_sensor(bno085, BNO085_SENSOR_ACCELEROMETER, 50000);
bno085_enable_sensor(bno085, BNO085_SENSOR_GYROSCOPE_CALIBRATED, 50000);
bno085_enable_sensor(bno085, BNO085_SENSOR_MAGNETIC_FIELD_CALIBRATED, 50000);
bno085_enable_sensor(bno085, BNO085_SENSOR_LINEAR_ACCELERATION, 50000);
```

### Activity Detection (Fitness Tracker)

```c
bno085_enable_sensor(bno085, BNO085_SENSOR_STEP_COUNTER, 500000);
bno085_enable_sensor(bno085, BNO085_SENSOR_PERSONAL_ACTIVITY_CLASSIFIER, 1000000);
bno085_enable_sensor(bno085, BNO085_SENSOR_SIGNIFICANT_MOTION, 1000000);
```

### Motion + Orientation (Robot/Drone)

```c
bno085_enable_sensor(bno085, BNO085_SENSOR_ROTATION_VECTOR, 50000);  // 20Hz
bno085_enable_sensor(bno085, BNO085_SENSOR_LINEAR_ACCELERATION, 50000);
bno085_enable_sensor(bno085, BNO085_SENSOR_GYROSCOPE_CALIBRATED, 50000);
```

### Game Controller

```c
bno085_register_sensor_callback(bno085, sensor_callback, NULL);
bno085_enable_sensor(bno085, BNO085_SENSOR_GAME_ROTATION_VECTOR, 10000);  // 100Hz
```

## Sensor Data Structure

The callback receives `bno085_sensor_value_t` with:
- `sensor_id`: Which sensor sent this data
- `status`: Accuracy/reliability (0=unreliable, 1=low, 2=medium, 3=high)
- `timestamp_us`: Timestamp in microseconds
- `data`: Union with decoded sensor data

Access the appropriate data field based on sensor_id:

```c
if (value->sensor_id == BNO085_SENSOR_ACCELEROMETER) {
    float x = value->data.accelerometer.x;
    float y = value->data.accelerometer.y;
    float z = value->data.accelerometer.z;
}

if (value->sensor_id == BNO085_SENSOR_ROTATION_VECTOR) {
    float i = value->data.quaternion.i;
    float j = value->data.quaternion.j;
    float k = value->data.quaternion.k;
    float real = value->data.quaternion.real;
    float accuracy = value->data.quaternion.accuracy_rad;
}

if (value->sensor_id == BNO085_SENSOR_STEP_COUNTER) {
    uint32_t steps = value->data.step_counter.count;
}
```

## Limitations

- Up to `CONFIG_BNO085_MAX_INSTANCES` concurrent sensors (default 2, configurable to 1-8)
- Only one active BNO085 per I2C bus without a multiplexer
- The underlying SH2 library is not thread-safe for init/deinit; call from a single task

## Example Project

See `examples/basic_read/` for a complete working example.

## License

Apache-2.0. Includes CEVA SH2 library (also Apache-2.0). See `LICENSE` for details.
