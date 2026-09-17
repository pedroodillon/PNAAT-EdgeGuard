#include <cmath>
#include <cstdio>


#include "esp_err.h"
#include "esp_log.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ACS712_ADC_CHANNEL ADC_CHANNEL_0 // GPIO1 = ADC1_CH0
#define ACS712_ADC_UNIT ADC_UNIT_1

/*
 * ACS712-20A:
 * sensibilidade nominal = ~100 mV/A
 *
 * Divisor:
 * OUT -> 22k -> GPIO1 -> 10k -> GND
 *
 * Razao nominal:
 * 10 / (22 + 10) = 0.3125
 *
 * Sensibilidade vista pelo ADC:
 * 100 mV/A * 0.3125 = 31.25 mV/A
 */
#define ACS712_SENSITIVITY_ADC_MV_PER_A 31.25f

#define ZERO_CALIBRATION_SAMPLES 500
#define CURRENT_AVERAGE_SAMPLES 100

static const char *TAG = "P101_ACS712";

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle = NULL;

static float zero_voltage_mv = 0.0f;

static bool init_adc_calibration(void) {
  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = ACS712_ADC_UNIT,
      .chan = ACS712_ADC_CHANNEL,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };

  esp_err_t err =
      adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);

  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Calibracao ADC habilitada");
    return true;
  }

  ESP_LOGW(TAG, "Calibracao ADC indisponivel: %s", esp_err_to_name(err));

  adc_cali_handle = NULL;

  return false;
}

static float read_adc_voltage_mv(void) {
  int raw = 0;

  esp_err_t err = adc_oneshot_read(adc_handle, ACS712_ADC_CHANNEL, &raw);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Erro lendo ADC: %s", esp_err_to_name(err));

    return 0.0f;
  }

  if (adc_cali_handle != NULL) {

    int voltage_mv = 0;

    err = adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_mv);

    if (err == ESP_OK) {
      return (float)voltage_mv;
    }
  }

  /*
   * Fallback aproximado caso a calibracao
   * nao esteja disponivel.
   */
  return ((float)raw / 4095.0f) * 3300.0f;
}

static float average_voltage_mv(size_t samples) {
  double sum = 0.0;

  for (size_t i = 0; i < samples; i++) {

    sum += read_adc_voltage_mv();

    vTaskDelay(pdMS_TO_TICKS(2));
  }

  return (float)(sum / (double)samples);
}

static void calibrate_zero_current(void) {
  printf("\n");
  printf("========================================\n");
  printf("CALIBRACAO DO ZERO\n");
  printf("Mantenha o motor DESLIGADO.\n");
  printf("========================================\n");

  vTaskDelay(pdMS_TO_TICKS(2000));

  zero_voltage_mv = average_voltage_mv(ZERO_CALIBRATION_SAMPLES);

  printf("Zero ADC medido: %.2f mV\n", zero_voltage_mv);

  printf("Zero ADC medido: %.3f V\n", zero_voltage_mv / 1000.0f);

  printf("========================================\n\n");
}

extern "C" void app_main(void) {
  printf("\n");
  printf("========================================\n");
  printf("P-101 / EDGE GUARD\n");
  printf("Teste ACS712-20A\n");
  printf("========================================\n");
  printf("Entrada ADC.....: GPIO1 / ADC1_CH0\n");
  printf("Divisor.........: 22k / 10k\n");
  printf("Sensibilidade...: %.2f mV/A no GPIO\n",
         ACS712_SENSITIVITY_ADC_MV_PER_A);
  printf("========================================\n\n");

  adc_oneshot_unit_init_cfg_t unit_config = {
      .unit_id = ACS712_ADC_UNIT,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc_handle));

  adc_oneshot_chan_cfg_t channel_config = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };

  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ACS712_ADC_CHANNEL,
                                             &channel_config));

  init_adc_calibration();

  calibrate_zero_current();

  printf("Agora ligue o motor.\n\n");

  while (true) {

    const float voltage_mv = average_voltage_mv(CURRENT_AVERAGE_SAMPLES);

    const float delta_mv = zero_voltage_mv - voltage_mv;

    const float current_a = fabsf(delta_mv) / ACS712_SENSITIVITY_ADC_MV_PER_A;

    printf("ADC = %.3f V | Delta = %.1f mV | Corrente = %.3f A\n",
           voltage_mv / 1000.0f, delta_mv, current_a);

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}