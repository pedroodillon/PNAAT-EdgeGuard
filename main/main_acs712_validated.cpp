#include <cmath>
#include <cstdint>
#include <cstdio>


#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================
// P-101 / EDGE GUARD
// ACS712-20A
// ============================================================

// Heltec WiFi LoRa 32 V3
// GPIO1 = ADC1_CH0
#define ACS712_ADC_UNIT ADC_UNIT_1
#define ACS712_ADC_CHANNEL ADC_CHANNEL_0

// ============================================================
// DIVISOR RESISTIVO
//
// ACS712 OUT
//     |
//    22k
//     |
//     +------ GPIO1
//     |
//    10k
//     |
//    GND
//
// Razao nominal:
//
// 10 / (22 + 10) = 0.3125
//
// ACS712-20A nominal:
// aproximadamente 100 mV/A
//
// Sensibilidade vista pelo ADC:
//
// 100 mV/A * 0.3125
// = 31.25 mV/A
// ============================================================

#define ACS712_SENSITIVITY_ADC_MV_PER_A 31.25f

// Quantidade de amostras usadas para calibrar o zero
#define ZERO_CALIBRATION_SAMPLES 500

// Quantidade de amostras para cada leitura de corrente
#define CURRENT_AVERAGE_SAMPLES 100

// Intervalo entre amostras individuais
#define ADC_SAMPLE_INTERVAL_MS 2

// Tempo ignorado como regime após partida do motor
#define MOTOR_STARTUP_TIME_MS 3000

// Corrente abaixo disso é considerada zero
#define CURRENT_DEADBAND_A 0.08f

// Histerese para detecção do motor
#define MOTOR_ON_THRESHOLD_A 0.25f
#define MOTOR_OFF_THRESHOLD_A 0.15f

// Intervalo aproximado entre relatórios serial
#define SERIAL_REPORT_INTERVAL_MS 500

static const char *TAG = "P101_ACS712";

// ============================================================
// ESTADO DO MOTOR
// ============================================================

typedef enum {
  MOTOR_STATE_OFF = 0,
  MOTOR_STATE_STARTUP,
  MOTOR_STATE_STEADY
} motor_state_t;

// ============================================================
// VARIAVEIS GLOBAIS
// ============================================================

static adc_oneshot_unit_handle_t adc_handle;

static adc_cali_handle_t adc_cali_handle = NULL;

static float zero_voltage_mv = 0.0f;

static motor_state_t motor_state = MOTOR_STATE_OFF;

static int64_t motor_start_time_us = 0;

static float startup_peak_current_a = 0.0f;

static float steady_current_a = 0.0f;

// ============================================================
// CALIBRACAO DO ADC
// ============================================================

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

// ============================================================
// LEITURA DE UMA AMOSTRA ADC
// ============================================================

static float read_adc_voltage_mv(void) {
  int raw = 0;

  esp_err_t err = adc_oneshot_read(adc_handle, ACS712_ADC_CHANNEL, &raw);

  if (err != ESP_OK) {

    ESP_LOGE(TAG, "Erro lendo ADC: %s", esp_err_to_name(err));

    return 0.0f;
  }

  // --------------------------------------------------------
  // Usa calibracao do ESP-IDF quando disponivel
  // --------------------------------------------------------

  if (adc_cali_handle != NULL) {

    int voltage_mv = 0;

    err = adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_mv);

    if (err == ESP_OK) {

      return (float)voltage_mv;
    }
  }

  // --------------------------------------------------------
  // Fallback aproximado
  // --------------------------------------------------------

  return ((float)raw / 4095.0f) * 3300.0f;
}

// ============================================================
// MEDIA DE TENSAO
// ============================================================

static float average_voltage_mv(size_t samples) {
  double sum = 0.0;

  for (size_t i = 0; i < samples; i++) {

    sum += read_adc_voltage_mv();

    vTaskDelay(pdMS_TO_TICKS(ADC_SAMPLE_INTERVAL_MS));
  }

  return (float)(sum / (double)samples);
}

// ============================================================
// TENSAO -> CORRENTE
// ============================================================

static float voltage_to_current_a(float voltage_mv) {
  /*
   * No nosso sentido de corrente:
   *
   * mais corrente
   *     ↓
   * tensão ADC menor
   *
   * Portanto:
   *
   * delta = zero - leitura
   */

  float delta_mv = zero_voltage_mv - voltage_mv;

  float current_a = fabsf(delta_mv) / ACS712_SENSITIVITY_ADC_MV_PER_A;

  // --------------------------------------------------------
  // Deadband
  //
  // Pequenos resíduos próximos de zero são descartados.
  // --------------------------------------------------------

  if (current_a < CURRENT_DEADBAND_A) {

    current_a = 0.0f;
  }

  return current_a;
}

// ============================================================
// CALIBRACAO AUTOMATICA DO ZERO
// ============================================================

static void calibrate_zero_current(void) {
  printf("\n");
  printf("========================================\n");
  printf("CALIBRACAO ACS712\n");
  printf("========================================\n");
  printf("\n");
  printf("IMPORTANTE:\n");
  printf("Mantenha o motor DESLIGADO.\n");
  printf("\n");

  printf("Calibracao inicia em 3 segundos...\n");

  vTaskDelay(pdMS_TO_TICKS(3000));

  zero_voltage_mv = average_voltage_mv(ZERO_CALIBRATION_SAMPLES);

  printf("\n");
  printf("----------------------------------------\n");
  printf("Zero ADC : %.2f mV\n", zero_voltage_mv);

  printf("Zero ADC : %.3f V\n", zero_voltage_mv / 1000.0f);

  printf("----------------------------------------\n");
  printf("\n");
}

// ============================================================
// NOME DO ESTADO
// ============================================================

static const char *motor_state_name(motor_state_t state) {
  switch (state) {

  case MOTOR_STATE_OFF:
    return "OFF";

  case MOTOR_STATE_STARTUP:
    return "STARTUP";

  case MOTOR_STATE_STEADY:
    return "STEADY";

  default:
    return "UNKNOWN";
  }
}

// ============================================================
// MAQUINA DE ESTADOS DO MOTOR
// ============================================================

static void update_motor_state(float current_a) {
  const int64_t now_us = esp_timer_get_time();

  // ========================================================
  // MOTOR OFF
  // ========================================================

  if (motor_state == MOTOR_STATE_OFF) {

    steady_current_a = 0.0f;

    if (current_a >= MOTOR_ON_THRESHOLD_A) {

      motor_state = MOTOR_STATE_STARTUP;

      motor_start_time_us = now_us;

      startup_peak_current_a = current_a;

      printf("\n");
      printf("========================================\n");
      printf("MOTOR DETECTADO\n");
      printf("Estado: STARTUP\n");
      printf("Aguardando estabilizacao por 3 s...\n");
      printf("========================================\n");
      printf("\n");
    }

    return;
  }

  // ========================================================
  // STARTUP
  // ========================================================

  if (motor_state == MOTOR_STATE_STARTUP) {

    // ----------------------------------------------------
    // Atualiza pico de partida
    // ----------------------------------------------------

    if (current_a > startup_peak_current_a) {

      startup_peak_current_a = current_a;
    }

    // ----------------------------------------------------
    // Motor desligou durante partida
    // ----------------------------------------------------

    if (current_a <= MOTOR_OFF_THRESHOLD_A) {

      motor_state = MOTOR_STATE_OFF;

      startup_peak_current_a = 0.0f;

      steady_current_a = 0.0f;

      printf("\n");
      printf("Motor desligado durante STARTUP.\n");
      printf("\n");

      return;
    }

    // ----------------------------------------------------
    // Tempo decorrido desde a partida
    // ----------------------------------------------------

    const int64_t elapsed_ms = (now_us - motor_start_time_us) / 1000;

    // ----------------------------------------------------
    // Depois de 3 segundos entra em regime
    // ----------------------------------------------------

    if (elapsed_ms >= MOTOR_STARTUP_TIME_MS) {

      motor_state = MOTOR_STATE_STEADY;

      steady_current_a = current_a;

      printf("\n");
      printf("========================================\n");
      printf("REGIME ESTAVEL\n");
      printf("========================================\n");

      printf("Pico de partida : %.3f A\n", startup_peak_current_a);

      printf("Corrente regime : %.3f A\n", steady_current_a);

      printf("========================================\n");
      printf("\n");
    }

    return;
  }

  // ========================================================
  // STEADY
  // ========================================================

  if (motor_state == MOTOR_STATE_STEADY) {

    // ----------------------------------------------------
    // Detecta desligamento
    // ----------------------------------------------------

    if (current_a <= MOTOR_OFF_THRESHOLD_A) {

      printf("\n");
      printf("========================================\n");
      printf("MOTOR DESLIGADO\n");
      printf("========================================\n");

      printf("Ultimo pico partida : %.3f A\n", startup_peak_current_a);

      printf("Ultima corrente      : %.3f A\n", steady_current_a);

      printf("========================================\n");
      printf("\n");

      motor_state = MOTOR_STATE_OFF;

      startup_peak_current_a = 0.0f;

      steady_current_a = 0.0f;

      return;
    }

    // ----------------------------------------------------
    // Atualiza corrente de regime
    // ----------------------------------------------------

    steady_current_a = current_a;
  }
}

// ============================================================
// APP MAIN
// ============================================================

extern "C" void app_main(void) {
  printf("\n");
  printf("========================================\n");
  printf("P-101 / EDGE GUARD\n");
  printf("ACS712-20A - Monitor de Corrente\n");
  printf("========================================\n");

  printf("ADC.............: GPIO1 / ADC1_CH0\n");

  printf("Divisor.........: 22k / 10k\n");

  printf("Sensibilidade...: %.2f mV/A no ADC\n",
         ACS712_SENSITIVITY_ADC_MV_PER_A);

  printf("Startup.........: %d ms\n", MOTOR_STARTUP_TIME_MS);

  printf("Deadband........: %.2f A\n", CURRENT_DEADBAND_A);

  printf("Motor ON........: >= %.2f A\n", MOTOR_ON_THRESHOLD_A);

  printf("Motor OFF.......: <= %.2f A\n", MOTOR_OFF_THRESHOLD_A);

  printf("========================================\n");

  // ========================================================
  // INICIALIZA ADC
  // ========================================================

  adc_oneshot_unit_init_cfg_t unit_config = {
      .unit_id = ACS712_ADC_UNIT,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc_handle));

  // ========================================================
  // CONFIGURA CANAL
  // ========================================================

  adc_oneshot_chan_cfg_t channel_config = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };

  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ACS712_ADC_CHANNEL,
                                             &channel_config));

  // ========================================================
  // CALIBRACAO ADC
  // ========================================================

  init_adc_calibration();

  // ========================================================
  // CALIBRACAO DE ZERO ACS712
  // ========================================================

  calibrate_zero_current();

  printf("Sistema pronto.\n");
  printf("Pode ligar o motor.\n\n");

  // ========================================================
  // LOOP PRINCIPAL
  // ========================================================

  while (true) {

    // ----------------------------------------------------
    // Faz media de 100 amostras
    // ----------------------------------------------------

    const float voltage_mv = average_voltage_mv(CURRENT_AVERAGE_SAMPLES);

    // ----------------------------------------------------
    // Calcula corrente
    // ----------------------------------------------------

    const float current_a = voltage_to_current_a(voltage_mv);

    // ----------------------------------------------------
    // Atualiza maquina de estados
    // ----------------------------------------------------

    update_motor_state(current_a);

    // ----------------------------------------------------
    // Informacoes adicionais
    // ----------------------------------------------------

    const float delta_mv = zero_voltage_mv - voltage_mv;

    float startup_elapsed_s = 0.0f;

    if (motor_state == MOTOR_STATE_STARTUP) {

      startup_elapsed_s =
          (esp_timer_get_time() - motor_start_time_us) / 1000000.0f;
    }

    // ----------------------------------------------------
    // Saida serial
    // ----------------------------------------------------

    if (motor_state == MOTOR_STATE_STARTUP) {

      printf("Estado=%-7s | "
             "ADC=%.3f V | "
             "I=%.3f A | "
             "Pico=%.3f A | "
             "t=%.1f/3.0 s\n",
             motor_state_name(motor_state), voltage_mv / 1000.0f, current_a,
             startup_peak_current_a, startup_elapsed_s);

    } else if (motor_state == MOTOR_STATE_STEADY) {

      printf("Estado=%-7s | "
             "ADC=%.3f V | "
             "Delta=%.1f mV | "
             "I_regime=%.3f A | "
             "Pico=%.3f A\n",
             motor_state_name(motor_state), voltage_mv / 1000.0f, delta_mv,
             steady_current_a, startup_peak_current_a);

    } else {

      printf("Estado=%-7s | "
             "ADC=%.3f V | "
             "I=%.3f A\n",
             motor_state_name(motor_state), voltage_mv / 1000.0f, current_a);
    }

    vTaskDelay(pdMS_TO_TICKS(SERIAL_REPORT_INTERVAL_MS));
  }
}