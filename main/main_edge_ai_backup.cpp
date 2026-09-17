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

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "model-parameters/model_metadata.h"

/* ============================================================
 * P-101 / EDGE GUARD
 *
 * BNO085 + Edge Impulse
 *
 * Pipeline:
 *
 * BNO085 Linear Acceleration
 *        ↓
 * 400 samples @ 200 Hz
 *        ↓
 * 1200 values [X,Y,Z]
 *        ↓
 * Edge Impulse DSP
 *        ↓
 * INT8 neural network
 *        ↓
 * Operational-state classification
 * ============================================================ */

/* ------------------------------------------------------------
 * BNO085 wiring
 * ------------------------------------------------------------ */

#define BNO085_SDA_GPIO GPIO_NUM_6
#define BNO085_SCL_GPIO GPIO_NUM_7
#define BNO085_INT_GPIO GPIO_NUM_5
#define BNO085_RESET_GPIO GPIO_NUM_4

#define BNO085_I2C_PORT I2C_NUM_1
#define BNO085_I2C_SPEED_HZ 100000

/* ------------------------------------------------------------
 * Sampling
 * ------------------------------------------------------------ */

/* 200 Hz = 5000 us */
#define REPORT_INTERVAL_US 5000

#define SAMPLE_COUNT EI_CLASSIFIER_RAW_SAMPLE_COUNT
#define AXES_PER_SAMPLE EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME
#define BUFFER_SIZE EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE

static const char *TAG = "P101_EDGE_AI";

/*
 * Edge Impulse input buffer:
 *
 * [X0, Y0, Z0,
 *  X1, Y1, Z1,
 *  ...
 *  X399, Y399, Z399]
 *
 * 400 × 3 = 1200 floats
 */
static float acquisition_buffer[BUFFER_SIZE];

/* ============================================================
 * Print classification result
 * ============================================================ */

static void print_classification(const ei_impulse_result_t *result) {
  printf("\n");
  printf("========================================\n");
  printf("RESULTADO EDGE AI\n");
  printf("========================================\n");

  size_t best_index = 0;
  float best_value = 0.0f;

  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {

    const float probability = result->classification[i].value;

    printf("%-20s : %.2f %%\n", result->classification[i].label,
           probability * 100.0f);

    if (probability > best_value) {
      best_value = probability;
      best_index = i;
    }
  }

  printf("----------------------------------------\n");

  printf("Predicao............: %s\n",
         result->classification[best_index].label);

  printf("Confianca...........: %.2f %%\n", best_value * 100.0f);

  printf("----------------------------------------\n");

  printf("DSP.................: %lu ms\n",
         static_cast<unsigned long>(result->timing.dsp));

  printf("Classificacao.......: %lu ms\n",
         static_cast<unsigned long>(result->timing.classification));

#if EI_CLASSIFIER_HAS_ANOMALY == 1

  printf("Anomalia............: %.5f\n", result->anomaly);

#endif

  printf("========================================\n");
}

/* ============================================================
 * app_main
 * ============================================================ */

extern "C" void app_main(void) {
  printf("\n");
  printf("========================================\n");
  printf("P-101 / EDGE GUARD\n");
  printf("BNO085 + Edge Impulse\n");
  printf("========================================\n");

  printf("Frequencia modelo...: %.0f Hz\n",
         static_cast<double>(EI_CLASSIFIER_FREQUENCY));

  printf("Intervalo modelo....: %.2f ms\n",
         static_cast<double>(EI_CLASSIFIER_INTERVAL_MS));

  printf("Amostras/janela.....: %d\n", SAMPLE_COUNT);

  printf("Eixos...............: %d\n", AXES_PER_SAMPLE);

  printf("Buffer..............: %d floats\n", BUFFER_SIZE);

  printf("Classes.............: %d\n", EI_CLASSIFIER_LABEL_COUNT);

  printf("========================================\n\n");

  /*
   * --------------------------------------------------------
   * Safety check
   * --------------------------------------------------------
   */

  if (BUFFER_SIZE != (SAMPLE_COUNT * AXES_PER_SAMPLE)) {

    ESP_LOGE(TAG, "Dimensoes Edge Impulse inconsistentes");

    return;
  }

  /*
   * --------------------------------------------------------
   * I2C
   * --------------------------------------------------------
   */

  i2c_master_bus_handle_t i2c_bus = nullptr;

  initialize_i2c(&i2c_bus, BNO085_I2C_PORT, BNO085_SDA_GPIO, BNO085_SCL_GPIO);

  /*
   * --------------------------------------------------------
   * BNO085
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
   * Linear Acceleration @ 200 Hz
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
   * Allow BNO085 / SH2 to stabilize.
   */
  vTaskDelay(pdMS_TO_TICKS(1500));

  /*
   * ========================================================
   * Continuous classification
   * ========================================================
   */

  while (true) {

    printf("\n");
    printf("----------------------------------------\n");
    printf("Coletando janela para inferencia\n");
    printf("----------------------------------------\n");

    uint32_t sample_index = 0;

    uint64_t last_sensor_timestamp = 0;

    int64_t acquisition_start_us = 0;
    int64_t acquisition_end_us = 0;

    /*
     * ----------------------------------------------------
     * Acquire exactly 400 unique samples
     * ----------------------------------------------------
     */

    while (sample_index < SAMPLE_COUNT) {

      bno085_service();

      bno085_linear_accel_t accel;

      if (bno085_get_linear_acceleration(&accel)) {

        /*
         * Ignore duplicated reading returned by
         * repeated getter calls.
         */
        if (accel.timestamp_us != last_sensor_timestamp) {

          last_sensor_timestamp = accel.timestamp_us;

          if (sample_index == 0) {

            acquisition_start_us = esp_timer_get_time();
          }

          const uint32_t base = sample_index * AXES_PER_SAMPLE;

          acquisition_buffer[base + 0] = accel.x;

          acquisition_buffer[base + 1] = accel.y;

          acquisition_buffer[base + 2] = accel.z;

          sample_index++;

          if ((sample_index % 100) == 0) {

            printf("Coletadas: %lu / %d\n",
                   static_cast<unsigned long>(sample_index), SAMPLE_COUNT);
          }

          if (sample_index == SAMPLE_COUNT) {

            acquisition_end_us = esp_timer_get_time();
          }
        }
      }

      vTaskDelay(pdMS_TO_TICKS(1));
    }

    /*
     * ----------------------------------------------------
     * Acquisition diagnostics
     * ----------------------------------------------------
     */

    const double elapsed_ms =
        static_cast<double>(acquisition_end_us - acquisition_start_us) / 1000.0;

    double effective_frequency_hz = 0.0;

    if (elapsed_ms > 0.0) {

      effective_frequency_hz =
          static_cast<double>(SAMPLE_COUNT - 1) / (elapsed_ms / 1000.0);
    }

    printf("\n");

    printf("Janela coletada.....: %.2f ms\n", elapsed_ms);

    printf("Frequencia efetiva..: %.2f Hz\n", effective_frequency_hz);

    /*
     * ----------------------------------------------------
     * Convert buffer to Edge Impulse signal
     * ----------------------------------------------------
     */

    signal_t signal;

    int signal_error =
        numpy::signal_from_buffer(acquisition_buffer, BUFFER_SIZE, &signal);

    if (signal_error != 0) {

      ESP_LOGE(TAG, "Falha criando signal_t: %d", signal_error);

      vTaskDelay(pdMS_TO_TICKS(2000));

      continue;
    }

    /*
     * ----------------------------------------------------
     * Run Edge Impulse classifier
     * ----------------------------------------------------
     */

    ei_impulse_result_t result = {};

    EI_IMPULSE_ERROR inference_error = run_classifier(&signal, &result, false);

    if (inference_error != EI_IMPULSE_OK) {

      ESP_LOGE(TAG, "run_classifier falhou: %d",
               static_cast<int>(inference_error));

      vTaskDelay(pdMS_TO_TICKS(2000));

      continue;
    }

    /*
     * ----------------------------------------------------
     * Display prediction
     * ----------------------------------------------------
     */

    print_classification(&result);

    /*
     * Pause before next independent 2 s window.
     *
     * Later we can remove this delay for continuous
     * classifications.
     */
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}