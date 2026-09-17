#include <cmath>
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

static const char *TAG = "BNO085_VIB_TEST";

/* --------------------------------------------------------------------------
 * P-101 / Edge Guard
 *
 * BNO085
 * I2C1
 * SDA = GPIO6
 * SCL = GPIO7
 * INT = GPIO5
 * RST = GPIO4
 * ADDR = 0x4A
 *
 * Linear Acceleration:
 * 200 Hz -> intervalo = 5000 us
 * -------------------------------------------------------------------------- */

static constexpr gpio_num_t BNO085_SDA_GPIO = GPIO_NUM_6;
static constexpr gpio_num_t BNO085_SCL_GPIO = GPIO_NUM_7;
static constexpr gpio_num_t BNO085_INT_GPIO = GPIO_NUM_5;
static constexpr gpio_num_t BNO085_RST_GPIO = GPIO_NUM_4;

static constexpr uint8_t BNO085_ADDRESS = BNO085_I2C_ADDR_DEFAULT;

static constexpr uint32_t BNO085_SAMPLE_INTERVAL_US = 5000; // 200 Hz

/* Janela de RMS:
 * 200 amostras @ 200 Hz = aproximadamente 1 segundo
 */
static constexpr uint32_t RMS_WINDOW_SAMPLES = 200;

/*
 * Para não inundar o terminal:
 * imprime uma amostra a cada 10 amostras recebidas.
 *
 * 200 Hz / 10 = ~20 linhas/s
 */
static constexpr uint32_t PRINT_DECIMATION = 10;

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "P-101 / Edge Guard");
  ESP_LOGI(TAG, "Teste de vibracao BNO085");
  ESP_LOGI(TAG, "========================================");

  /*
   * ------------------------------------------------------------
   * 1. Inicializa I2C1
   * ------------------------------------------------------------
   */

  i2c_master_bus_handle_t i2c_bus = nullptr;

  initialize_i2c(&i2c_bus, I2C_NUM_1, BNO085_SDA_GPIO, BNO085_SCL_GPIO);

  /*
   * ------------------------------------------------------------
   * 2. Configura BNO085
   * ------------------------------------------------------------
   */

  bno085_dev_t bno_dev = {};

  bno_dev.i2c_bus = i2c_bus;
  bno_dev.i2c_addr = BNO085_ADDRESS;

  bno_dev.reset_gpio = BNO085_RST_GPIO;
  bno_dev.int_gpio = BNO085_INT_GPIO;

  bno_dev.scl_speed_hz = 400000;

  ESP_LOGI(TAG, "BNO085: SDA=%d SCL=%d INT=%d RST=%d ADDR=0x%02X",
           BNO085_SDA_GPIO, BNO085_SCL_GPIO, BNO085_INT_GPIO, BNO085_RST_GPIO,
           BNO085_ADDRESS);

  esp_err_t err = bno085_init(&bno_dev);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Falha ao inicializar BNO085: %s", esp_err_to_name(err));

    return;
  }

  ESP_LOGI(TAG, "BNO085 inicializado");

  /*
   * ------------------------------------------------------------
   * 3. Habilita Linear Acceleration @ 200 Hz
   * ------------------------------------------------------------
   */

  err = bno085_enable_linear_acceleration(BNO085_SAMPLE_INTERVAL_US);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Falha ao habilitar Linear Acceleration: %s",
             esp_err_to_name(err));

    return;
  }

  ESP_LOGI(TAG, "Linear Acceleration habilitada: 200 Hz");

  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Formato:");
  ESP_LOGI(TAG, "X | Y | Z | MAG -> aceleracao linear em m/s^2");

  ESP_LOGI(TAG, "VIB_RMS -> RMS combinado X/Y/Z em janela de ~1 s");

  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "Motor inicialmente DESLIGADO.");
  ESP_LOGI(TAG, "========================================");

  /*
   * ------------------------------------------------------------
   * Variaveis de processamento
   * ------------------------------------------------------------
   */

  uint64_t last_timestamp_us = 0;

  uint32_t sample_counter = 0;
  uint32_t rms_counter = 0;

  double rms_sum_squares = 0.0;

  /*
   * ------------------------------------------------------------
   * 4. Loop principal
   * ------------------------------------------------------------
   */

  while (true) {

    /*
     * Processa pacotes SH2 pendentes.
     */
    bno085_service();

    bno085_linear_accel_t accel = {};

    /*
     * Recupera a ultima amostra valida.
     */
    if (bno085_get_linear_acceleration(&accel) && accel.valid) {

      /*
       * Evita contabilizar várias vezes a mesma leitura.
       */
      if (accel.timestamp_us != last_timestamp_us) {

        last_timestamp_us = accel.timestamp_us;

        const float x = accel.x;
        const float y = accel.y;
        const float z = accel.z;

        /*
         * Magnitude instantanea da aceleracao linear.
         *
         * Como estamos usando Linear Acceleration,
         * a componente gravitacional ja foi removida
         * pelo BNO085.
         */
        const float magnitude = std::sqrt((x * x) + (y * y) + (z * z));

        /*
         * RMS combinado:
         *
         * sqrt(mean(x² + y² + z²))
         */
        rms_sum_squares += static_cast<double>(x * x) +
                           static_cast<double>(y * y) +
                           static_cast<double>(z * z);

        rms_counter++;
        sample_counter++;

        /*
         * Imprime aproximadamente 20 vezes por segundo.
         */
        if ((sample_counter % PRINT_DECIMATION) == 0) {

          printf("X=%+8.4f  "
                 "Y=%+8.4f  "
                 "Z=%+8.4f  "
                 "| MAG=%8.4f m/s^2\n",
                 x, y, z, magnitude);
        }

        /*
         * Aproximadamente 1 segundo de dados.
         */
        if (rms_counter >= RMS_WINDOW_SAMPLES) {

          const float vibration_rms = static_cast<float>(
              std::sqrt(rms_sum_squares / static_cast<double>(rms_counter)));

          printf("\n"
                 "========================================\n"
                 "VIB_RMS = %.5f m/s^2  | samples=%lu\n"
                 "========================================\n\n",
                 vibration_rms, static_cast<unsigned long>(rms_counter));

          rms_sum_squares = 0.0;
          rms_counter = 0;
        }
      }
    }

    /*
     * Pequeno yield para o FreeRTOS.
     * O ritmo real das amostras continua sendo controlado
     * pelos relatórios do BNO085 @ 200 Hz.
     */
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}