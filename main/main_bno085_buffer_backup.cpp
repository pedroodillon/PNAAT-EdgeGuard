#include <cstdint>
#include <cstdio>


#include "bno085.h"
#include "i2c_config.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ============================================================
 * P-101 / EDGE GUARD
 *
 * BNO085 acquisition test
 *
 * Objective:
 *   - Linear Acceleration X/Y/Z
 *   - 200 Hz
 *   - 400 samples
 *   - 2 second acquisition window
 *   - 1200 interleaved float values
 * ============================================================ */

/* BNO085 wiring */
#define BNO085_SDA_GPIO GPIO_NUM_6
#define BNO085_SCL_GPIO GPIO_NUM_7
#define BNO085_INT_GPIO GPIO_NUM_5
#define BNO085_RESET_GPIO GPIO_NUM_4

#define BNO085_I2C_PORT I2C_NUM_1
#define BNO085_I2C_SPEED_HZ 100000

/* 200 Hz = one report every 5000 us */
#define REPORT_INTERVAL_US 5000

/* Edge Impulse acquisition dimensions */
#define SAMPLE_COUNT 400
#define AXES_PER_SAMPLE 3
#define BUFFER_SIZE (SAMPLE_COUNT * AXES_PER_SAMPLE)

static const char *TAG = "P101_BNO085";

/*
 * Interleaved buffer:
 *
 * index 0 = X0
 * index 1 = Y0
 * index 2 = Z0
 * index 3 = X1
 * index 4 = Y1
 * index 5 = Z1
 * ...
 *
 * Total:
 * 400 samples * 3 axes = 1200 floats
 */
static float acquisition_buffer[BUFFER_SIZE];

extern "C" void app_main(void) {
  printf("\n");
  printf("========================================\n");
  printf("P-101 / EDGE GUARD\n");
  printf("BNO085 - teste de buffer\n");
  printf("========================================\n");

  printf("Amostras por janela: %d\n", SAMPLE_COUNT);
  printf("Eixos por amostra..: %d\n", AXES_PER_SAMPLE);
  printf("Buffer total.......: %d floats\n", BUFFER_SIZE);
  printf("Frequencia alvo....: 200 Hz\n");
  printf("Janela alvo........: 2.0 s\n");
  printf("========================================\n\n");

  /*
   * --------------------------------------------------------
   * I2C initialization
   * --------------------------------------------------------
   */
  i2c_master_bus_handle_t i2c_bus = nullptr;

  initialize_i2c(&i2c_bus, BNO085_I2C_PORT, BNO085_SDA_GPIO, BNO085_SCL_GPIO);

  /*
   * --------------------------------------------------------
   * BNO085 configuration
   * --------------------------------------------------------
   */
  bno085_dev_t bno = {
      .i2c_bus = i2c_bus,
      .i2c_addr = BNO085_I2C_ADDR_DEFAULT,
      .reset_gpio = BNO085_RESET_GPIO,
      .int_gpio = BNO085_INT_GPIO,
      .scl_speed_hz = BNO085_I2C_SPEED_HZ,
  };

  esp_err_t err = bno085_init(&bno);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Falha inicializando BNO085: %s", esp_err_to_name(err));

    return;
  }

  ESP_LOGI(TAG, "BNO085 inicializado");

  /*
   * --------------------------------------------------------
   * Enable Linear Acceleration
   * --------------------------------------------------------
   */
  err = bno085_enable_linear_acceleration(REPORT_INTERVAL_US);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Falha habilitando Linear Acceleration: %s",
             esp_err_to_name(err));

    return;
  }

  ESP_LOGI(TAG, "Linear Acceleration habilitada em 200 Hz");

  /*
   * Give SH2/BNO085 some time to stabilize.
   */
  vTaskDelay(pdMS_TO_TICKS(1500));

  /*
   * --------------------------------------------------------
   * Continuous acquisition
   * --------------------------------------------------------
   */
  while (true) {

    printf("\n");
    printf("----------------------------------------\n");
    printf("Iniciando nova janela de 400 amostras\n");
    printf("----------------------------------------\n");

    uint32_t sample_index = 0;

    uint64_t last_sensor_timestamp = 0;

    int64_t acquisition_start_us = 0;
    int64_t acquisition_end_us = 0;

    /*
     * Collect exactly 400 UNIQUE sensor samples.
     */
    while (sample_index < SAMPLE_COUNT) {

      /*
       * Allow SH2 to process incoming BNO085 packets.
       */
      bno085_service();

      bno085_linear_accel_t accel;

      if (bno085_get_linear_acceleration(&accel)) {

        /*
         * The getter can return the most recent reading
         * multiple times.
         *
         * Only accept a reading when its timestamp changed.
         */
        if (accel.timestamp_us != last_sensor_timestamp) {

          last_sensor_timestamp = accel.timestamp_us;

          if (sample_index == 0) {
            acquisition_start_us = esp_timer_get_time();
          }

          /*
           * Interleaved storage:
           *
           * [X0 Y0 Z0 X1 Y1 Z1 ...]
           */
          const uint32_t base = sample_index * AXES_PER_SAMPLE;

          acquisition_buffer[base + 0] = accel.x;
          acquisition_buffer[base + 1] = accel.y;
          acquisition_buffer[base + 2] = accel.z;

          sample_index++;

          /*
           * Small progress indication.
           * Avoid printing every sample because serial
           * output could disturb acquisition timing.
           */
          if ((sample_index % 100) == 0) {
            printf("Coletadas: %lu / %d\n",
                   static_cast<unsigned long>(sample_index), SAMPLE_COUNT);
          }

          if (sample_index == SAMPLE_COUNT) {
            acquisition_end_us = esp_timer_get_time();
          }
        }
      }

      /*
       * Polling delay.
       *
       * Sensor itself determines the ~5 ms reporting
       * interval. We poll faster than that.
       */
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    /*
     * --------------------------------------------------------
     * Acquisition diagnostics
     * --------------------------------------------------------
     */
    const double elapsed_ms =
        static_cast<double>(acquisition_end_us - acquisition_start_us) / 1000.0;

    double effective_frequency_hz = 0.0;

    if (elapsed_ms > 0.0) {

      /*
       * There are 399 intervals between 400 samples.
       */
      effective_frequency_hz =
          static_cast<double>(SAMPLE_COUNT - 1) / (elapsed_ms / 1000.0);
    }

    printf("\n");
    printf("========================================\n");
    printf("JANELA CONCLUIDA\n");
    printf("========================================\n");

    printf("Amostras...........: %d\n", SAMPLE_COUNT);

    printf("Valores no buffer..: %d\n", BUFFER_SIZE);

    printf("Tempo medido........: %.2f ms\n", elapsed_ms);

    printf("Frequencia efetiva..: %.2f Hz\n", effective_frequency_hz);

    /*
     * First sample
     */
    printf("\nPrimeira amostra:\n");

    printf("X = %.6f\n", acquisition_buffer[0]);

    printf("Y = %.6f\n", acquisition_buffer[1]);

    printf("Z = %.6f\n", acquisition_buffer[2]);

    /*
     * Last sample
     */
    const uint32_t last_base = (SAMPLE_COUNT - 1) * AXES_PER_SAMPLE;

    printf("\nUltima amostra:\n");

    printf("X = %.6f\n", acquisition_buffer[last_base + 0]);

    printf("Y = %.6f\n", acquisition_buffer[last_base + 1]);

    printf("Z = %.6f\n", acquisition_buffer[last_base + 2]);

    printf("\n");
    printf("Buffer pronto para Edge Impulse.\n");
    printf("========================================\n");

    /*
     * Pause between test windows.
     */
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}