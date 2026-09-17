#include "bno085.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "euler.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

/* =========================================================
 * P-101 / EDGE GUARD
 * BNO085 DRIVER
 * ESP-IDF 6.x
 * ========================================================= */

static const char *TAG = "bno085";

/* =========================================================
 * Estado interno
 * ========================================================= */

static bno085_dev_t s_dev;

static sh2_Hal_t s_hal;

static i2c_master_dev_handle_t s_i2c_device = NULL;

/*
 * Sensor atualmente habilitado.
 */
static sh2_SensorId_t s_enabled_sensor = 0;

/*
 * Intervalo solicitado ao sensor.
 */
static uint32_t s_enabled_interval_us = 0;

/*
 * Após reset assíncrono, relatórios precisam
 * ser habilitados novamente.
 */
static volatile bool s_need_reenable = false;

/*
 * Últimos valores recebidos.
 */
static bno085_orientation_t s_last_orientation;

static bno085_linear_accel_t s_last_linear_accel;

/*
 * Buffer para pacotes SHTP.
 */
static uint8_t s_i2c_rx_buf[SH2_HAL_MAX_TRANSFER_IN];

/* =========================================================
 * HAL SH2
 * ========================================================= */

static int hal_open(sh2_Hal_t *self) {
  (void)self;

  return 0;
}

static void hal_close(sh2_Hal_t *self) {
  (void)self;

  /*
   * Não removemos o device I2C aqui.
   * Apenas mantemos o estado do hardware.
   */
}

/**
 * @brief Leitura SHTP sobre I2C.
 *
 * O BNO085 utiliza INT ativo em LOW.
 * Só tentamos ler quando INT está baixo.
 */
static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                    uint32_t *t_us) {
  (void)self;

  /*
   * Sem dados pendentes.
   */
  if (s_dev.int_gpio != GPIO_NUM_NC && gpio_get_level(s_dev.int_gpio) != 0) {
    return 0;
  }

  /*
   * Primeiro lemos os 4 bytes do cabeçalho SHTP.
   */
  esp_err_t err = i2c_master_receive(s_i2c_device, s_i2c_rx_buf, 4, 100);

  if (err != ESP_OK) {

    ESP_LOGD(TAG, "Falha lendo header SHTP: %s", esp_err_to_name(err));

    return 0;
  }

  /*
   * Tamanho do pacote:
   *
   * byte 0 = LSB
   * byte 1 = MSB
   */
  uint16_t packet_size =
      (uint16_t)s_i2c_rx_buf[0] | ((uint16_t)s_i2c_rx_buf[1] << 8);

  /*
   * Bit 15 indica continuation.
   */
  packet_size &= ~0x8000;

  if (packet_size == 0 || packet_size > len ||
      packet_size > sizeof(s_i2c_rx_buf)) {

    ESP_LOGD(TAG, "Packet size invalido: %u", packet_size);

    return 0;
  }

  /*
   * Agora lemos o pacote completo.
   */
  err = i2c_master_receive(s_i2c_device, s_i2c_rx_buf, packet_size, 100);

  if (err != ESP_OK) {

    ESP_LOGD(TAG, "Falha lendo pacote SHTP: %s", esp_err_to_name(err));

    return 0;
  }

  memcpy(pBuffer, s_i2c_rx_buf, packet_size);

  if (t_us != NULL) {

    *t_us = (uint32_t)esp_timer_get_time();
  }

  return (int)packet_size;
}

/**
 * @brief Escrita SHTP sobre I2C.
 */
static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  (void)self;

  esp_err_t err = i2c_master_transmit(s_i2c_device, pBuffer, len, 100);

  if (err != ESP_OK) {

    ESP_LOGD(TAG, "Falha escrevendo SHTP: %s", esp_err_to_name(err));

    return 0;
  }

  return (int)len;
}

/**
 * @brief Timestamp usado internamente pelo SH2.
 */
static uint32_t hal_get_time_us(sh2_Hal_t *self) {
  (void)self;

  return (uint32_t)esp_timer_get_time();
}

/* =========================================================
 * CALLBACKS SH2
 * ========================================================= */

/**
 * @brief Eventos assíncronos do SH2.
 */
static void event_callback(void *cookie, sh2_AsyncEvent_t *event) {
  (void)cookie;

  if (event == NULL) {
    return;
  }

  if (event->eventId == SH2_RESET) {

    ESP_LOGW(TAG, "BNO085 resetou; relatorio sera reabilitado");

    s_need_reenable = true;
  }
}

/**
 * @brief Callback dos sensores.
 */
static void sensor_callback(void *cookie, sh2_SensorEvent_t *event) {
  (void)cookie;

  if (event == NULL) {
    return;
  }

  sh2_SensorValue_t value;

  if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {

    ESP_LOGD(TAG, "Falha decodificando evento SH2");

    return;
  }

  /*
   * Ignora sensores que não correspondam
   * ao relatório atualmente habilitado.
   */
  if (value.sensorId != s_enabled_sensor) {
    return;
  }

  /* =====================================================
   * ROTATION VECTOR
   * ===================================================== */

  if (value.sensorId == SH2_ROTATION_VECTOR) {

    float roll;
    float pitch;
    float yaw;

    q_to_ypr(value.un.rotationVector.real, value.un.rotationVector.i,
             value.un.rotationVector.j, value.un.rotationVector.k, &roll,
             &pitch, &yaw);

    s_last_orientation.angle_x = roll * (180.0f / (float)M_PI);

    s_last_orientation.angle_y = pitch * (180.0f / (float)M_PI);

    s_last_orientation.angle_z = yaw * (180.0f / (float)M_PI);

    s_last_orientation.accuracy_rad = value.un.rotationVector.accuracy;

    s_last_orientation.valid = true;

    return;
  }

  /* =====================================================
   * LINEAR ACCELERATION
   * ===================================================== */

  if (value.sensorId == SH2_LINEAR_ACCELERATION) {

    s_last_linear_accel.x = value.un.linearAcceleration.x;

    s_last_linear_accel.y = value.un.linearAcceleration.y;

    s_last_linear_accel.z = value.un.linearAcceleration.z;

    /*
     * Timestamp local no momento em que
     * processamos a amostra.
     */
    s_last_linear_accel.timestamp_us = (uint64_t)esp_timer_get_time();

    s_last_linear_accel.valid = true;

    return;
  }
}

/* =========================================================
 * API PÚBLICA
 * ========================================================= */

esp_err_t bno085_init(const bno085_dev_t *dev) {
  if (dev == NULL || dev->i2c_bus == NULL) {

    return ESP_ERR_INVALID_ARG;
  }

  /*
   * Salva configuração.
   */
  s_dev = *dev;

  /*
   * Defaults.
   */
  if (s_dev.scl_speed_hz == 0) {

    s_dev.scl_speed_hz = 100000;
  }

  memset(&s_last_orientation, 0, sizeof(s_last_orientation));

  memset(&s_last_linear_accel, 0, sizeof(s_last_linear_accel));

  s_enabled_sensor = 0;

  s_enabled_interval_us = 0;

  s_need_reenable = false;

  /* =====================================================
   * Adiciona o BNO085 ao barramento I2C
   * ===================================================== */

  i2c_device_config_t device_config = {

      .dev_addr_length = I2C_ADDR_BIT_LEN_7,

      .device_address = s_dev.i2c_addr,

      .scl_speed_hz = s_dev.scl_speed_hz,
  };

  esp_err_t err =
      i2c_master_bus_add_device(s_dev.i2c_bus, &device_config, &s_i2c_device);

  if (err != ESP_OK) {

    ESP_LOGE(TAG, "Falha adicionando device I2C: %s", esp_err_to_name(err));

    return err;
  }

  /* =====================================================
   * RESET GPIO
   * ===================================================== */

  if (s_dev.reset_gpio != GPIO_NUM_NC) {

    gpio_config_t reset_cfg = {

        .pin_bit_mask = 1ULL << s_dev.reset_gpio,

        .mode = GPIO_MODE_OUTPUT,

        .pull_up_en = GPIO_PULLUP_DISABLE,

        .pull_down_en = GPIO_PULLDOWN_DISABLE,

        .intr_type = GPIO_INTR_DISABLE,
    };

    err = gpio_config(&reset_cfg);

    if (err != ESP_OK) {
      return err;
    }
  }

  /* =====================================================
   * INT GPIO
   * ===================================================== */

  if (s_dev.int_gpio != GPIO_NUM_NC) {

    gpio_config_t int_cfg = {

        .pin_bit_mask = 1ULL << s_dev.int_gpio,

        .mode = GPIO_MODE_INPUT,

        .pull_up_en = GPIO_PULLUP_ENABLE,

        .pull_down_en = GPIO_PULLDOWN_DISABLE,

        .intr_type = GPIO_INTR_DISABLE,
    };

    err = gpio_config(&int_cfg);

    if (err != ESP_OK) {
      return err;
    }
  }

  /* =====================================================
   * Hardware reset
   * ===================================================== */

  if (s_dev.reset_gpio != GPIO_NUM_NC) {

    gpio_set_level(s_dev.reset_gpio, 0);

    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_set_level(s_dev.reset_gpio, 1);
  }

  /* =====================================================
   * Espera INT ser assertado
   * ===================================================== */

  if (s_dev.int_gpio != GPIO_NUM_NC) {

    TickType_t start = xTaskGetTickCount();

    bool asserted = false;

    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(500)) {

      if (gpio_get_level(s_dev.int_gpio) == 0) {

        asserted = true;

        break;
      }

      vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (!asserted) {

      ESP_LOGE(TAG, "BNO085 nao assertou INT apos reset");

      return ESP_ERR_TIMEOUT;
    }
  }

  /* =====================================================
   * Inicializa SH2
   * ===================================================== */

  memset(&s_hal, 0, sizeof(s_hal));

  s_hal.open = hal_open;

  s_hal.close = hal_close;

  s_hal.read = hal_read;

  s_hal.write = hal_write;

  s_hal.getTimeUs = hal_get_time_us;

  int status = sh2_open(&s_hal, event_callback, NULL);

  if (status != SH2_OK) {

    ESP_LOGE(TAG, "sh2_open falhou: %d", status);

    return ESP_FAIL;
  }

  sh2_setSensorCallback(sensor_callback, NULL);

  /*
   * O SH2 gera um RESET durante o handshake
   * inicial. Ainda não existe relatório
   * habilitado, portanto limpamos a flag.
   */
  s_need_reenable = false;

  ESP_LOGI(TAG,
           "BNO085 inicializado "
           "(addr 0x%02X, I2C %lu Hz)",
           s_dev.i2c_addr, (unsigned long)s_dev.scl_speed_hz);

  return ESP_OK;
}

/* =========================================================
 * ROTATION VECTOR
 * ========================================================= */

esp_err_t bno085_enable_rotation_vector(uint32_t interval_us) {
  sh2_SensorConfig_t config = {0};

  config.reportInterval_us = interval_us;

  int status = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config);

  if (status != SH2_OK) {

    ESP_LOGE(TAG, "Falha habilitando Rotation Vector: %d", status);

    return ESP_FAIL;
  }

  s_enabled_sensor = SH2_ROTATION_VECTOR;

  s_enabled_interval_us = interval_us;

  s_last_orientation.valid = false;

  ESP_LOGI(TAG, "Rotation Vector habilitado: %lu us",
           (unsigned long)interval_us);

  return ESP_OK;
}

/* =========================================================
 * LINEAR ACCELERATION
 * ========================================================= */

esp_err_t bno085_enable_linear_acceleration(uint32_t interval_us) {
  sh2_SensorConfig_t config = {0};

  config.reportInterval_us = interval_us;

  int status = sh2_setSensorConfig(SH2_LINEAR_ACCELERATION, &config);

  if (status != SH2_OK) {

    ESP_LOGE(TAG, "Falha habilitando Linear Acceleration: %d", status);

    return ESP_FAIL;
  }

  s_enabled_sensor = SH2_LINEAR_ACCELERATION;

  s_enabled_interval_us = interval_us;

  s_last_linear_accel.valid = false;

  ESP_LOGI(TAG, "Linear Acceleration habilitado: %lu us",
           (unsigned long)interval_us);

  return ESP_OK;
}

/* =========================================================
 * SERVICE
 * ========================================================= */

void bno085_service(void) {
  /*
   * Processa os pacotes pendentes.
   */
  sh2_service();

  /*
   * BNO085 pode reiniciar internamente.
   *
   * Nesse caso precisamos reabilitar
   * o relatório atualmente ativo.
   */
  if (s_need_reenable && s_enabled_sensor != 0) {

    ESP_LOGW(TAG, "Reabilitando sensor apos reset");

    esp_err_t err = ESP_FAIL;

    if (s_enabled_sensor == SH2_ROTATION_VECTOR) {

      err = bno085_enable_rotation_vector(s_enabled_interval_us);
    }

    else if (s_enabled_sensor == SH2_LINEAR_ACCELERATION) {

      err = bno085_enable_linear_acceleration(s_enabled_interval_us);
    }

    if (err == ESP_OK) {

      s_need_reenable = false;
    }
  }
}

/* =========================================================
 * GETTERS
 * ========================================================= */

bool bno085_get_orientation(bno085_orientation_t *out) {
  if (out == NULL) {
    return false;
  }

  if (!s_last_orientation.valid) {
    return false;
  }

  *out = s_last_orientation;

  return true;
}

bool bno085_get_linear_acceleration(bno085_linear_accel_t *out) {
  if (out == NULL) {
    return false;
  }

  if (!s_last_linear_accel.valid) {
    return false;
  }

  *out = s_last_linear_accel;

  return true;
}