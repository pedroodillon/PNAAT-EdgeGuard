#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bno085.h"
#include "i2c_config.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * Edge Impulse
 */
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "model-parameters/model_metadata.h"

/*
 * ============================================================================
 * P-101 / EDGE GUARD
 *
 * Teste de inferência embarcada
 *
 * BNO085
 *   SDA = GPIO6
 *   SCL = GPIO7
 *   INT = GPIO5
 *   RST = GPIO4
 *   I2C1
 *
 * Edge Impulse:
 *   200 Hz
 *   400 amostras
 *   3 eixos
 *   1200 valores
 *   janela = 2 segundos
 *
 * Classes:
 *   L1
 *   L2
 *   NORMAL
 * ============================================================================
 */

static const char *TAG = "EDGE_GUARD_AI";

static constexpr gpio_num_t BNO085_SDA_GPIO = GPIO_NUM_6;
static constexpr gpio_num_t BNO085_SCL_GPIO = GPIO_NUM_7;
static constexpr gpio_num_t BNO085_INT_GPIO = GPIO_NUM_5;
static constexpr gpio_num_t BNO085_RST_GPIO = GPIO_NUM_4;

static constexpr uint8_t BNO085_ADDRESS = BNO085_I2C_ADDR_DEFAULT;

/*
 * 200 Hz = 5 ms = 5000 us.
 */
static constexpr uint32_t BNO085_SAMPLE_INTERVAL_US = 5000;

/*
 * Modelo:
 *
 * 400 amostras
 * 3 eixos
 * 1200 floats
 */
static constexpr size_t SAMPLE_COUNT = 400;
static constexpr size_t AXES_PER_SAMPLE = 3;
static constexpr size_t INPUT_FRAME_SIZE = SAMPLE_COUNT * AXES_PER_SAMPLE;

/*
 * Buffer fornecido ao Edge Impulse.
 *
 * Ordem:
 *
 * X0 Y0 Z0
 * X1 Y1 Z1
 * ...
 * X399 Y399 Z399
 */
static float inference_buffer[INPUT_FRAME_SIZE];

/*
 * --------------------------------------------------------------------------
 * Executa o classificador
 * --------------------------------------------------------------------------
 */
static bool run_edge_impulse_inference(float vibration_rms,
                                       const char **predicted_label,
                                       float *predicted_confidence) {
  signal_t signal;

  int signal_error =
      numpy::signal_from_buffer(inference_buffer, INPUT_FRAME_SIZE, &signal);

  if (signal_error != 0) {
    ESP_LOGE(TAG, "Falha ao criar signal_t: %d", signal_error);

    return false;
  }

  ei_impulse_result_t result = {};

  EI_IMPULSE_ERROR classifier_error = run_classifier(&signal, &result, false);

  if (classifier_error != EI_IMPULSE_OK) {
    ESP_LOGE(TAG, "run_classifier falhou: %d",
             static_cast<int>(classifier_error));

    return false;
  }

  printf("\n");
  printf("============================================\n");
  printf("P-101 / EDGE GUARD - EDGE AI\n");
  printf("============================================\n");

  printf("VIB_RMS : %.4f m/s^2\n", vibration_rms);

  printf("\nProbabilidades:\n");

  size_t best_index = 0;
  float best_value = -1.0f;

  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {

    const char *label = result.classification[i].label;

    const float value = result.classification[i].value;

    printf("  %-8s : %6.2f %%\n", label, value * 100.0f);

    if (value > best_value) {
      best_value = value;
      best_index = i;
    }
  }

  *predicted_label = result.classification[best_index].label;

  *predicted_confidence = result.classification[best_index].value;

  printf("\n");

  printf("RESULTADO : %s\n", *predicted_label);

  printf("CONFIANCA : %.2f %%\n", (*predicted_confidence) * 100.0f);

  printf("DSP       : %lu ms\n", static_cast<unsigned long>(result.timing.dsp));

  printf("CLASSIFY  : %lu ms\n",
         static_cast<unsigned long>(result.timing.classification));

#if EI_CLASSIFIER_HAS_ANOMALY == 1
  printf("ANOMALY   : %.5f\n", result.anomaly);
#endif

  printf("============================================\n\n");

  return true;
}

/*
 * --------------------------------------------------------------------------
 * app_main
 * --------------------------------------------------------------------------
 */
extern "C" void app_main(void) {
  ESP_LOGI(TAG, "============================================");

  ESP_LOGI(TAG, "P-101 / Edge Guard");

  ESP_LOGI(TAG, "BNO085 + Edge Impulse v2");

  ESP_LOGI(TAG, "Classes: L1 / L2 / NORMAL");

  ESP_LOGI(TAG, "============================================");

  /*
   * Validação da configuração esperada do modelo.
   */
  ESP_LOGI(TAG, "EI frequency: %.1f Hz",
           static_cast<double>(EI_CLASSIFIER_FREQUENCY));

  ESP_LOGI(TAG, "EI raw sample count: %d", EI_CLASSIFIER_RAW_SAMPLE_COUNT);

  ESP_LOGI(TAG, "EI axes/frame: %d", EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME);

  ESP_LOGI(TAG, "EI DSP input frame: %d", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);

  /*
   * Proteção contra modelo incompatível.
   */
  if (EI_CLASSIFIER_RAW_SAMPLE_COUNT != SAMPLE_COUNT ||
      EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != AXES_PER_SAMPLE ||
      EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE != INPUT_FRAME_SIZE) {

    ESP_LOGE(TAG, "Configuracao do modelo nao corresponde ao firmware.");

    ESP_LOGE(TAG, "Esperado: samples=%u axes=%u frame=%u",
             static_cast<unsigned>(SAMPLE_COUNT),
             static_cast<unsigned>(AXES_PER_SAMPLE),
             static_cast<unsigned>(INPUT_FRAME_SIZE));

    return;
  }

  /*
   * ----------------------------------------------------------------------
   * Inicialização I2C1
   * ----------------------------------------------------------------------
   */

  i2c_master_bus_handle_t i2c_bus = nullptr;

  initialize_i2c(&i2c_bus, I2C_NUM_1, BNO085_SDA_GPIO, BNO085_SCL_GPIO);

  /*
   * ----------------------------------------------------------------------
   * Inicialização BNO085
   * ----------------------------------------------------------------------
   */

  bno085_dev_t bno_dev = {};

  bno_dev.i2c_bus = i2c_bus;
  bno_dev.i2c_addr = BNO085_ADDRESS;

  bno_dev.reset_gpio = BNO085_RST_GPIO;

  bno_dev.int_gpio = BNO085_INT_GPIO;

  bno_dev.scl_speed_hz = 400000;

  ESP_LOGI(TAG, "Inicializando BNO085...");

  esp_err_t err = bno085_init(&bno_dev);

  if (err != ESP_OK) {

    ESP_LOGE(TAG, "Falha BNO085: %s", esp_err_to_name(err));

    return;
  }

  err = bno085_enable_linear_acceleration(BNO085_SAMPLE_INTERVAL_US);

  if (err != ESP_OK) {

    ESP_LOGE(TAG, "Falha ao habilitar aceleracao: %s", esp_err_to_name(err));

    return;
  }

  ESP_LOGI(TAG, "BNO085 pronto @ 200 Hz");

  ESP_LOGI(TAG, "Aguardando primeira janela de 2 segundos...");

  /*
   * ----------------------------------------------------------------------
   * Aquisição
   * ----------------------------------------------------------------------
   */

  size_t sample_index = 0;

  uint64_t last_timestamp_us = 0;

  double sum_squares = 0.0;

  while (true) {

    /*
     * Alimenta o protocolo SH2.
     */
    bno085_service();

    bno085_linear_accel_t accel = {};

    if (!bno085_get_linear_acceleration(&accel)) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    if (!accel.valid) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    /*
     * Não reutiliza a mesma amostra.
     */
    if (accel.timestamp_us == last_timestamp_us) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    last_timestamp_us = accel.timestamp_us;

    /*
     * Índice dentro do vetor flattened:
     *
     * sample 0 = [X Y Z]
     * sample 1 = [X Y Z]
     */
    const size_t buffer_index = sample_index * AXES_PER_SAMPLE;

    inference_buffer[buffer_index + 0] = accel.x;

    inference_buffer[buffer_index + 1] = accel.y;

    inference_buffer[buffer_index + 2] = accel.z;

    /*
     * Acumula RMS combinado.
     */
    sum_squares += static_cast<double>(accel.x) * static_cast<double>(accel.x);

    sum_squares += static_cast<double>(accel.y) * static_cast<double>(accel.y);

    sum_squares += static_cast<double>(accel.z) * static_cast<double>(accel.z);

    sample_index++;

    /*
     * Quando temos 400 amostras:
     *
     * 400 / 200 Hz = 2 segundos
     */
    if (sample_index >= SAMPLE_COUNT) {

      const float vibration_rms = static_cast<float>(
          std::sqrt(sum_squares / static_cast<double>(SAMPLE_COUNT)));

      const char *predicted_label = "UNKNOWN";

      float predicted_confidence = 0.0f;

      run_edge_impulse_inference(vibration_rms, &predicted_label,
                                 &predicted_confidence);

      /*
       * Reinicia a próxima janela.
       *
       * Aqui usamos janelas não sobrepostas:
       * uma inferência nova a cada ~2 segundos.
       */
      sample_index = 0;
      sum_squares = 0.0;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}