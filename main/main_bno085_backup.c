#include <stdio.h>

#include "bno085.h"
#include "i2c_config.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* =========================================================
 * P-101 / EDGE GUARD
 *
 * BNO085 -> Edge Impulse Data Forwarder
 *
 * Saída serial:
 *
 * accX,accY,accZ
 *
 * IMPORTANTE:
 * - sem timestamp
 * - sem cabeçalho
 * - sem textos no loop
 * ========================================================= */

/* =========================================================
 * PINAGEM BNO085
 * ========================================================= */

#define BNO085_SDA_GPIO GPIO_NUM_6
#define BNO085_SCL_GPIO GPIO_NUM_7
#define BNO085_INT_GPIO GPIO_NUM_5
#define BNO085_RESET_GPIO GPIO_NUM_4

/* =========================================================
 * I2C
 * ========================================================= */

#define BNO085_I2C_PORT I2C_NUM_1
#define BNO085_I2C_SPEED_HZ 100000

/* =========================================================
 * TAXA DE AMOSTRAGEM
 *
 * 200 Hz = 5000 us
 * ========================================================= */

#define REPORT_INTERVAL_US 5000

static const char *TAG = "P101_EI";

void app_main(void) {
  /* =====================================================
   * Inicializa barramento I2C
   * ===================================================== */

  i2c_master_bus_handle_t i2c_bus = NULL;

  initialize_i2c(&i2c_bus, BNO085_I2C_PORT, BNO085_SDA_GPIO, BNO085_SCL_GPIO);

  /* =====================================================
   * Configuração do BNO085
   * ===================================================== */

  bno085_dev_t bno = {
      .i2c_bus = i2c_bus,
      .i2c_addr = BNO085_I2C_ADDR_DEFAULT,
      .reset_gpio = BNO085_RESET_GPIO,
      .int_gpio = BNO085_INT_GPIO,
      .scl_speed_hz = BNO085_I2C_SPEED_HZ,
  };

  /* =====================================================
   * Inicializa BNO085
   * ===================================================== */

  esp_err_t err = bno085_init(&bno);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Falha inicializando BNO085: %s", esp_err_to_name(err));

    return;
  }

  /* =====================================================
   * Habilita Linear Acceleration a 200 Hz
   * ===================================================== */

  err = bno085_enable_linear_acceleration(REPORT_INTERVAL_US);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Falha habilitando Linear Acceleration");

    return;
  }

  /*
   * Dá tempo para as mensagens iniciais do ESP-IDF
   * terminarem antes de começar o stream puro de dados.
   */
  vTaskDelay(pdMS_TO_TICKS(1500));

  /* =====================================================
   * LOOP DE AQUISIÇÃO
   * ===================================================== */

  uint64_t last_timestamp_us = 0;

  while (1) {

    /*
     * Processa os pacotes SH2 pendentes.
     */
    bno085_service();

    bno085_linear_accel_t accel;

    if (bno085_get_linear_acceleration(&accel)) {

      /*
       * Só envia quando chega uma nova amostra.
       */
      if (accel.timestamp_us != last_timestamp_us) {

        last_timestamp_us = accel.timestamp_us;

        /*
         * FORMATO PARA EDGE IMPULSE:
         *
         * X,Y,Z
         *
         * Exemplo:
         * 0.027344,-0.015625,-0.031250
         */
        printf("%.6f,%.6f,%.6f\n", accel.x, accel.y, accel.z);
      }
    }

    /*
     * Mantém o SH2 sendo atendido
     * com frequência suficiente.
     */
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}