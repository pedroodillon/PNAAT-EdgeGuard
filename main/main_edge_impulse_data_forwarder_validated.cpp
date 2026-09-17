#include <cstdint>
#include <cstdio>

#include "bno085.h"
#include "i2c_config.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * ============================================================================
 * P-101 / Edge Guard
 * Edge Impulse Data Forwarder streamer
 *
 * Sensor:
 *   BNO085
 *
 * Interface:
 *   I2C1
 *   SDA = GPIO6
 *   SCL = GPIO7
 *   INT = GPIO5
 *   RST = GPIO4
 *   Address = 0x4A
 *
 * Sampling:
 *   Linear Acceleration
 *   200 Hz
 *   Interval = 5000 us
 *
 * Serial output:
 *
 *   X,Y,Z
 *
 * Example:
 *
 *   0.1250,-0.0625,0.0312
 *
 * IMPORTANT:
 *   No labels
 *   No RMS
 *   No magnitude
 *   No debug text
 *
 * This format is intended exclusively for:
 *
 *   edge-impulse-data-forwarder
 * ============================================================================
 */

static constexpr gpio_num_t BNO085_SDA_GPIO = GPIO_NUM_6;
static constexpr gpio_num_t BNO085_SCL_GPIO = GPIO_NUM_7;

static constexpr gpio_num_t BNO085_INT_GPIO = GPIO_NUM_5;
static constexpr gpio_num_t BNO085_RST_GPIO = GPIO_NUM_4;

static constexpr uint8_t BNO085_ADDRESS = BNO085_I2C_ADDR_DEFAULT;

/*
 * 200 Hz:
 *
 * 1 / 200 = 0.005 s
 *          = 5000 us
 */
static constexpr uint32_t BNO085_SAMPLE_INTERVAL_US = 5000;

extern "C" void app_main(void) {
  /*
   * Evita buffering da saída serial.
   */
  setvbuf(stdout, nullptr, _IONBF, 0);

  /*
   * IMPORTANTE:
   *
   * O Edge Impulse Data Forwarder precisa receber somente
   * valores numéricos.
   *
   * Portanto desligamos os ESP_LOG durante a execução.
   */
  esp_log_level_set("*", ESP_LOG_NONE);

  /*
   * ------------------------------------------------------------------------
   * I2C1
   * ------------------------------------------------------------------------
   */

  i2c_master_bus_handle_t i2c_bus = nullptr;

  initialize_i2c(&i2c_bus, I2C_NUM_1, BNO085_SDA_GPIO, BNO085_SCL_GPIO);

  /*
   * ------------------------------------------------------------------------
   * BNO085
   * ------------------------------------------------------------------------
   */

  bno085_dev_t bno_dev = {};

  bno_dev.i2c_bus = i2c_bus;
  bno_dev.i2c_addr = BNO085_ADDRESS;

  bno_dev.reset_gpio = BNO085_RST_GPIO;
  bno_dev.int_gpio = BNO085_INT_GPIO;

  bno_dev.scl_speed_hz = 400000;

  esp_err_t err = bno085_init(&bno_dev);

  if (err != ESP_OK) {
    /*
     * Não imprimimos texto porque isso confundiria
     * o Data Forwarder.
     */
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  /*
   * ------------------------------------------------------------------------
   * Linear Acceleration @ 200 Hz
   * ------------------------------------------------------------------------
   */

  err = bno085_enable_linear_acceleration(BNO085_SAMPLE_INTERVAL_US);

  if (err != ESP_OK) {
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  /*
   * Timestamp da última amostra transmitida.
   *
   * Isso evita repetir a mesma amostra diversas vezes enquanto
   * bno085_service() aguarda o próximo relatório do sensor.
   */
  uint64_t last_timestamp_us = 0;

  /*
   * ------------------------------------------------------------------------
   * Streaming
   * ------------------------------------------------------------------------
   */

  while (true) {

    /*
     * Processa os pacotes SH2 recebidos do BNO085.
     */
    bno085_service();

    bno085_linear_accel_t accel = {};

    if (bno085_get_linear_acceleration(&accel) && accel.valid) {

      /*
       * Só envia quando uma nova amostra chegou.
       */
      if (accel.timestamp_us != last_timestamp_us) {

        last_timestamp_us = accel.timestamp_us;

        /*
         * FORMATO EDGE IMPULSE:
         *
         * X,Y,Z
         *
         * Não adicionar nenhuma outra informação.
         */
        printf("%.4f,%.4f,%.4f\r\n", accel.x, accel.y, accel.z);
      }
    }

    /*
     * Dá oportunidade ao scheduler sem impor a frequência
     * de aquisição.
     *
     * A frequência real é determinada pelo BNO085:
     * 5000 us -> 200 Hz.
     */
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}