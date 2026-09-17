#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "aws_iot_config.h"
#include "wifi_config.h"

#include "bno085.h"
#include "i2c_config.h"
#include "inmp441.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "mqtt_client.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "lwip/ip4_addr.h"
#include "lwip/netdb.h"

/*
 * Edge Impulse
 */
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "model-parameters/model_metadata.h"

// ============================================================
// P-101 / EDGE GUARD
// Firmware final integrado
//
// ACS712  -> corrente / estado operacional
// INMP441 -> audio RMS
// BNO085  -> vibracao X/Y/Z
// Edge AI -> NORMAL / L1 / L2
// AWS IoT -> MQTT/TLS
// ============================================================

// ============================================================
// TAGS
// ============================================================

static const char *TAG_WIFI = "P101_WIFI";
static const char *TAG_DNS = "P101_DNS";
static const char *TAG_AWS = "P101_AWS";
static const char *TAG_ACS = "P101_ACS712";
static const char *TAG_AUDIO = "P101_AUDIO";
static const char *TAG_AI = "P101_EDGE_AI";

// ============================================================
// ACS712
// ============================================================

#define ACS712_ADC_UNIT ADC_UNIT_1
#define ACS712_ADC_CHANNEL ADC_CHANNEL_0

#define ACS712_SENSITIVITY_ADC_MV_PER_A 31.25f

#define ZERO_CALIBRATION_SAMPLES 500
#define CURRENT_AVERAGE_SAMPLES 100
#define ADC_SAMPLE_INTERVAL_MS 2

#define CURRENT_DEADBAND_A 0.08f

#define MOTOR_ON_THRESHOLD_A 0.25f
#define MOTOR_OFF_THRESHOLD_A 0.15f

#define MOTOR_STARTUP_TIME_MS 3000

// ============================================================
// INMP441
// ============================================================

#define INMP441_BCLK_GPIO GPIO_NUM_39
#define INMP441_WS_GPIO GPIO_NUM_40
#define INMP441_DATA_GPIO GPIO_NUM_41

#define INMP441_SAMPLE_RATE 16000

#define AUDIO_FRAMES 256
#define AUDIO_WORDS (AUDIO_FRAMES * 2)

// ============================================================
// BNO085
// ============================================================

static constexpr gpio_num_t BNO085_SDA_GPIO = GPIO_NUM_6;
static constexpr gpio_num_t BNO085_SCL_GPIO = GPIO_NUM_7;
static constexpr gpio_num_t BNO085_INT_GPIO = GPIO_NUM_5;
static constexpr gpio_num_t BNO085_RST_GPIO = GPIO_NUM_4;

static constexpr uint8_t BNO085_ADDRESS = BNO085_I2C_ADDR_DEFAULT;

static constexpr uint32_t BNO085_SAMPLE_INTERVAL_US = 5000;

// ============================================================
// EDGE IMPULSE
//
// 200 Hz
// 400 samples
// 3 axes
// 1200 values
// 2 second window
// ============================================================

static constexpr size_t SAMPLE_COUNT = 400;
static constexpr size_t AXES_PER_SAMPLE = 3;

static constexpr size_t INPUT_FRAME_SIZE = SAMPLE_COUNT * AXES_PER_SAMPLE;

static float inference_buffer[INPUT_FRAME_SIZE];

// ============================================================
// AWS CERTIFICATES
// ============================================================

extern "C" {

extern const uint8_t
    amazon_root_ca_start[] asm("_binary_AmazonRootCA1_pem_start");

extern const uint8_t amazon_root_ca_end[] asm("_binary_AmazonRootCA1_pem_end");

extern const uint8_t
    device_cert_start[] asm("_binary_device_cert_pem_crt_start");

extern const uint8_t device_cert_end[] asm("_binary_device_cert_pem_crt_end");

extern const uint8_t
    device_private_key_start[] asm("_binary_device_private_pem_key_start");

extern const uint8_t
    device_private_key_end[] asm("_binary_device_private_pem_key_end");
}

// ============================================================
// WIFI / MQTT
// ============================================================

#define WIFI_CONNECTED_BIT BIT0

static EventGroupHandle_t wifi_event_group = nullptr;

static esp_netif_t *wifi_sta_netif = nullptr;

static esp_mqtt_client_handle_t mqtt_client = nullptr;

static volatile bool mqtt_connected = false;

// ============================================================
// MOTOR STATE
// ============================================================

typedef enum {
  MOTOR_STATE_OFF = 0,
  MOTOR_STATE_STARTUP,
  MOTOR_STATE_STEADY
} motor_state_t;

static volatile motor_state_t motor_state = MOTOR_STATE_OFF;

static const char *motor_state_internal_string(motor_state_t state) {
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

/*
 * Estado apresentado no projeto final.
 *
 * STARTUP e STEADY significam que a máquina está girando.
 */
static const char *operational_state_string(motor_state_t state) {
  if (state == MOTOR_STATE_OFF) {
    return "OFF";
  }

  return "RUNNING";
}

// ============================================================
// SHARED TELEMETRY
// ============================================================

static volatile float current_a = 0.0f;

static volatile float startup_peak_current_a = 0.0f;

static volatile float audio_rms = 0.0f;

static volatile float vibration_rms = 0.0f;

static volatile float ai_confidence = 0.0f;

static volatile float probability_l1 = 0.0f;
static volatile float probability_l2 = 0.0f;
static volatile float probability_normal = 0.0f;

static char mechanical_state[16] = "UNKNOWN";

// ============================================================
// ADC
// ============================================================

static adc_oneshot_unit_handle_t adc_handle = nullptr;

static adc_cali_handle_t adc_cali_handle = nullptr;

static float zero_voltage_mv = 0.0f;

static int64_t motor_start_time_us = 0;

// ============================================================
// AUDIO BUFFER
// ============================================================

static int32_t audio_buffer[AUDIO_WORDS];

// ============================================================
// ADC CALIBRATION
// ============================================================

static bool init_adc_calibration() {
  adc_cali_curve_fitting_config_t cali_config = {};

  cali_config.unit_id = ACS712_ADC_UNIT;

  cali_config.chan = ACS712_ADC_CHANNEL;

  cali_config.atten = ADC_ATTEN_DB_12;

  cali_config.bitwidth = ADC_BITWIDTH_DEFAULT;

  esp_err_t err =
      adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);

  if (err == ESP_OK) {

    ESP_LOGI(TAG_ACS, "Calibracao ADC habilitada");

    return true;
  }

  ESP_LOGW(TAG_ACS, "Calibracao ADC indisponivel: %s", esp_err_to_name(err));

  adc_cali_handle = nullptr;

  return false;
}

// ============================================================
// ADC INIT
// ============================================================

static void init_adc() {
  adc_oneshot_unit_init_cfg_t unit_config = {};

  unit_config.unit_id = ACS712_ADC_UNIT;

  unit_config.clk_src = static_cast<adc_oneshot_clk_src_t>(0);

  unit_config.ulp_mode = ADC_ULP_MODE_DISABLE;

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc_handle));

  adc_oneshot_chan_cfg_t channel_config = {};

  channel_config.atten = ADC_ATTEN_DB_12;

  channel_config.bitwidth = ADC_BITWIDTH_DEFAULT;

  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ACS712_ADC_CHANNEL,
                                             &channel_config));

  init_adc_calibration();
}

// ============================================================
// ADC READ
// ============================================================

static float read_adc_voltage_mv() {
  int raw = 0;

  esp_err_t err = adc_oneshot_read(adc_handle, ACS712_ADC_CHANNEL, &raw);

  if (err != ESP_OK) {

    ESP_LOGE(TAG_ACS, "Erro ADC: %s", esp_err_to_name(err));

    return 0.0f;
  }

  if (adc_cali_handle != nullptr) {

    int voltage_mv = 0;

    err = adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_mv);

    if (err == ESP_OK) {
      return static_cast<float>(voltage_mv);
    }
  }

  return (static_cast<float>(raw) / 4095.0f) * 3300.0f;
}

static float average_voltage_mv(size_t samples) {
  double sum = 0.0;

  for (size_t i = 0; i < samples; i++) {

    sum += read_adc_voltage_mv();

    vTaskDelay(pdMS_TO_TICKS(ADC_SAMPLE_INTERVAL_MS));
  }

  return static_cast<float>(sum / static_cast<double>(samples));
}

// ============================================================
// ACS712 ZERO
// ============================================================

static void calibrate_acs712_zero() {
  printf("\n");
  printf("============================================\n");
  printf("ACS712 ZERO CALIBRATION\n");
  printf("MOTOR MUST BE OFF\n");
  printf("============================================\n");

  zero_voltage_mv = average_voltage_mv(ZERO_CALIBRATION_SAMPLES);

  printf("ACS712 ZERO : %.2f mV\n", zero_voltage_mv);

  printf("============================================\n\n");
}

static float voltage_to_current_a(float voltage_mv) {
  float delta_mv = zero_voltage_mv - voltage_mv;

  float calculated_current =
      std::fabs(delta_mv) / ACS712_SENSITIVITY_ADC_MV_PER_A;

  if (calculated_current < CURRENT_DEADBAND_A) {

    calculated_current = 0.0f;
  }

  return calculated_current;
}

static float read_current_a() {
  const float voltage_mv = average_voltage_mv(CURRENT_AVERAGE_SAMPLES);

  return voltage_to_current_a(voltage_mv);
}

// ============================================================
// MOTOR STATE
// ============================================================

static void update_motor_state(float measured_current) {
  const int64_t now_us = esp_timer_get_time();

  switch (motor_state) {

  case MOTOR_STATE_OFF:

    if (measured_current >= MOTOR_ON_THRESHOLD_A) {

      motor_state = MOTOR_STATE_STARTUP;

      motor_start_time_us = now_us;

      startup_peak_current_a = measured_current;
    }

    break;

  case MOTOR_STATE_STARTUP:

    if (measured_current > startup_peak_current_a) {

      startup_peak_current_a = measured_current;
    }

    if (measured_current <= MOTOR_OFF_THRESHOLD_A) {

      motor_state = MOTOR_STATE_OFF;

      startup_peak_current_a = 0.0f;

      break;
    }

    if (((now_us - motor_start_time_us) / 1000) >= MOTOR_STARTUP_TIME_MS) {

      motor_state = MOTOR_STATE_STEADY;
    }

    break;

  case MOTOR_STATE_STEADY:

    if (measured_current <= MOTOR_OFF_THRESHOLD_A) {

      motor_state = MOTOR_STATE_OFF;

      startup_peak_current_a = 0.0f;
    }

    break;

  default:

    motor_state = MOTOR_STATE_OFF;

    break;
  }
}

// ============================================================
// INMP441
// ============================================================

static void init_audio() {
  inmp441_config_t config = {.bclk_gpio = INMP441_BCLK_GPIO,

                             .ws_gpio = INMP441_WS_GPIO,

                             .data_gpio = INMP441_DATA_GPIO,

                             .sample_rate = INMP441_SAMPLE_RATE};

  ESP_ERROR_CHECK(inmp441_init(&config));

  ESP_LOGI(TAG_AUDIO, "INMP441 pronto @ %d Hz", INMP441_SAMPLE_RATE);
}

static float read_audio_rms() {
  size_t words_read = 0;

  esp_err_t err = inmp441_read_samples(audio_buffer, AUDIO_WORDS, &words_read);

  if (err != ESP_OK || words_read < 2) {

    return audio_rms;
  }

  double sum = 0.0;

  size_t count = 0;

  /*
   * L/R do INMP441 está em HIGH,
   * portanto utilizamos o canal RIGHT:
   * índices ímpares.
   */
  for (size_t i = 1; i < words_read; i += 2) {

    const int32_t sample = audio_buffer[i];

    const double sample24 = static_cast<double>(sample >> 8);

    const double normalized = sample24 / 8388608.0;

    sum += normalized * normalized;

    count++;
  }

  if (count == 0) {
    return 0.0f;
  }

  return static_cast<float>(std::sqrt(sum / static_cast<double>(count)));
}

// ============================================================
// DNS
// ============================================================

static void configure_dns() {
  if (wifi_sta_netif == nullptr) {
    return;
  }

  esp_netif_dns_info_t dns_main = {};

  dns_main.ip.type = ESP_IPADDR_TYPE_V4;

  IP4_ADDR(&dns_main.ip.u_addr.ip4, 8, 8, 8, 8);

  esp_netif_set_dns_info(wifi_sta_netif, ESP_NETIF_DNS_MAIN, &dns_main);
}

// ============================================================
// WIFI EVENTS
// ============================================================

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  (void)arg;

  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {

    esp_wifi_connect();

    return;
  }

  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {

    mqtt_connected = false;

    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);

    esp_wifi_connect();

    return;
  }

  if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {

    const auto *event = static_cast<ip_event_got_ip_t *>(event_data);

    ESP_LOGI(TAG_WIFI, "IP: " IPSTR, IP2STR(&event->ip_info.ip));

    configure_dns();

    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

// ============================================================
// WIFI
// ============================================================

static void wifi_init() {
  wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());

  wifi_sta_netif = esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             wifi_event_handler, nullptr));

  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             wifi_event_handler, nullptr));

  wifi_config_t wifi_config = {};

  std::strncpy(reinterpret_cast<char *>(wifi_config.sta.ssid), WIFI_SSID,
               sizeof(wifi_config.sta.ssid) - 1);

  std::strncpy(reinterpret_cast<char *>(wifi_config.sta.password),
               WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);

  wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;

  wifi_config.sta.pmf_cfg.capable = true;

  wifi_config.sta.pmf_cfg.required = false;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

  ESP_ERROR_CHECK(esp_wifi_start());

  xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE,
                      portMAX_DELAY);
}

// ============================================================
// MQTT
// ============================================================

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  (void)handler_args;
  (void)base;

  auto event = static_cast<esp_mqtt_event_handle_t>(event_data);

  switch (static_cast<esp_mqtt_event_id_t>(event_id)) {

  case MQTT_EVENT_CONNECTED:

    mqtt_connected = true;

    ESP_LOGI(TAG_AWS, "AWS IoT conectado");

    break;

  case MQTT_EVENT_DISCONNECTED:

    mqtt_connected = false;

    break;

  case MQTT_EVENT_PUBLISHED:

    ESP_LOGI(TAG_AWS, "Publicado msg_id=%d", event->msg_id);

    break;

  default:

    break;
  }
}

static void mqtt_init() {
  esp_mqtt_client_config_t mqtt_cfg = {};

  mqtt_cfg.broker.address.hostname = AWS_IOT_HOST;

  mqtt_cfg.broker.address.port = AWS_IOT_PORT;

  mqtt_cfg.broker.address.transport = MQTT_TRANSPORT_OVER_SSL;

  mqtt_cfg.credentials.client_id = AWS_IOT_CLIENT_ID;

  mqtt_cfg.broker.verification.certificate =
      reinterpret_cast<const char *>(amazon_root_ca_start);

  mqtt_cfg.credentials.authentication.certificate =
      reinterpret_cast<const char *>(device_cert_start);

  mqtt_cfg.credentials.authentication.key =
      reinterpret_cast<const char *>(device_private_key_start);

  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

  ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY,
                                                 mqtt_event_handler, nullptr));

  ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
}

// ============================================================
// MQTT TELEMETRY
// ============================================================

static void publish_telemetry() {
  if (mqtt_client == nullptr || !mqtt_connected) {

    return;
  }

  char payload[512];

  std::snprintf(payload, sizeof(payload),

                "{"
                "\"device_id\":\"%s\","
                "\"status\":\"online\","
                "\"current_a\":%.3f,"
                "\"audio_rms\":%.6f,"
                "\"vibration_rms\":%.4f,"
                "\"operational_state\":\"%s\","
                "\"mechanical_state\":\"%s\","
                "\"ai_confidence\":%.4f,"
                "\"startup_peak_a\":%.3f"
                "}",

                AWS_IOT_CLIENT_ID,

                static_cast<double>(current_a),

                static_cast<double>(audio_rms),

                static_cast<double>(vibration_rms),

                operational_state_string(motor_state),

                mechanical_state,

                static_cast<double>(ai_confidence),

                static_cast<double>(startup_peak_current_a));

  const int msg_id =
      esp_mqtt_client_publish(mqtt_client, AWS_IOT_TOPIC, payload, 0, 1, 0);

  printf("MQTT | id=%d | %s\n", msg_id, payload);
}

// ============================================================
// SENSOR / MQTT TASK
// ============================================================

static void telemetry_task(void *arg) {
  (void)arg;

  int publish_counter = 0;

  while (true) {

    const float new_current = read_current_a();

    current_a = new_current;

    update_motor_state(new_current);

    audio_rms = read_audio_rms();

    /*
     * Uma leitura completa de corrente já
     * consome aproximadamente 200 ms.
     *
     * Publicamos aproximadamente a cada
     * 2 segundos.
     */
    publish_counter++;

    if (publish_counter >= 8) {

      publish_counter = 0;

      publish_telemetry();
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ============================================================
// EDGE IMPULSE
// ============================================================

static bool run_edge_impulse_inference() {
  signal_t signal = {};

  int signal_error =
      numpy::signal_from_buffer(inference_buffer, INPUT_FRAME_SIZE, &signal);

  if (signal_error != 0) {

    ESP_LOGE(TAG_AI, "signal_from_buffer=%d", signal_error);

    return false;
  }

  ei_impulse_result_t result = {};

  EI_IMPULSE_ERROR error = run_classifier(&signal, &result, false);

  if (error != EI_IMPULSE_OK) {

    ESP_LOGE(TAG_AI, "run_classifier=%d", static_cast<int>(error));

    return false;
  }

  float best_value = -1.0f;

  const char *best_label = "UNKNOWN";

  probability_l1 = 0.0f;
  probability_l2 = 0.0f;
  probability_normal = 0.0f;

  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {

    const char *label = result.classification[i].label;

    const float value = result.classification[i].value;

    if (std::strcmp(label, "L1") == 0) {

      probability_l1 = value;
    }

    else if (std::strcmp(label, "L2") == 0) {

      probability_l2 = value;
    }

    else if (std::strcmp(label, "NORMAL") == 0) {

      probability_normal = value;
    }

    if (value > best_value) {

      best_value = value;

      best_label = label;
    }
  }

  std::snprintf(mechanical_state, sizeof(mechanical_state), "%s", best_label);

  ai_confidence = best_value;

  return true;
}

// ============================================================
// FINAL SERIAL PANEL
// ============================================================

static void print_system_status() {
  printf("\n");
  printf("============================================================\n");
  printf("P-101 / EDGE GUARD\n");
  printf("============================================================\n");

  printf("CURRENT_A          : %.3f A\n", static_cast<double>(current_a));

  printf("AUDIO_RMS          : %.6f\n", static_cast<double>(audio_rms));

  printf("OPERATIONAL_STATE  : %s\n", operational_state_string(motor_state));

  printf("\n");

  printf("VIBRATION_RMS      : %.4f m/s^2\n",
         static_cast<double>(vibration_rms));

  printf("MECHANICAL_STATE   : %s\n", mechanical_state);

  printf("AI_CONFIDENCE      : %.2f %%\n",
         static_cast<double>(ai_confidence * 100.0f));

  printf("\n");

  printf("L1                 : %.2f %%\n",
         static_cast<double>(probability_l1 * 100.0f));

  printf("L2                 : %.2f %%\n",
         static_cast<double>(probability_l2 * 100.0f));

  printf("NORMAL             : %.2f %%\n",
         static_cast<double>(probability_normal * 100.0f));

  printf("============================================================\n");
}

// ============================================================
// APP MAIN
// ============================================================

extern "C" void app_main() {
  printf("\n");
  printf("============================================================\n");
  printf("P-101 / EDGE GUARD - FINAL INTEGRATED FIRMWARE\n");
  printf("============================================================\n");

  /*
   * Validate model.
   */
  if (EI_CLASSIFIER_RAW_SAMPLE_COUNT != SAMPLE_COUNT ||

      EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != AXES_PER_SAMPLE ||

      EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE != INPUT_FRAME_SIZE) {

    ESP_LOGE(TAG_AI, "Modelo Edge Impulse incompativel");

    return;
  }

  /*
   * NVS
   */
  esp_err_t ret = nvs_flash_init();

  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||

      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

    ESP_ERROR_CHECK(nvs_flash_erase());

    ret = nvs_flash_init();
  }

  ESP_ERROR_CHECK(ret);

  /*
   * ACS712
   *
   * Motor OFF during boot.
   */
  init_adc();

  calibrate_acs712_zero();

  /*
   * INMP441
   */
  init_audio();

  /*
   * BNO085
   */
  i2c_master_bus_handle_t i2c_bus = nullptr;

  initialize_i2c(&i2c_bus, I2C_NUM_1, BNO085_SDA_GPIO, BNO085_SCL_GPIO);

  bno085_dev_t bno_dev = {};

  bno_dev.i2c_bus = i2c_bus;

  bno_dev.i2c_addr = BNO085_ADDRESS;

  bno_dev.reset_gpio = BNO085_RST_GPIO;

  bno_dev.int_gpio = BNO085_INT_GPIO;

  bno_dev.scl_speed_hz = 400000;

  ESP_ERROR_CHECK(bno085_init(&bno_dev));

  ESP_ERROR_CHECK(bno085_enable_linear_acceleration(BNO085_SAMPLE_INTERVAL_US));

  ESP_LOGI(TAG_AI, "BNO085 pronto @ 200 Hz");

  /*
   * Network.
   */
  wifi_init();

  mqtt_init();

  /*
   * Separate task:
   * current + audio + AWS.
   */
  xTaskCreate(telemetry_task, "telemetry_task", 6144, nullptr, 4, nullptr);

  /*
   * BNO085 + Edge AI acquisition loop.
   */
  size_t sample_index = 0;

  uint64_t last_timestamp_us = 0;

  double sum_squares = 0.0;

  while (true) {

    bno085_service();

    bno085_linear_accel_t accel = {};

    if (!bno085_get_linear_acceleration(&accel) || !accel.valid) {

      vTaskDelay(pdMS_TO_TICKS(1));

      continue;
    }

    if (accel.timestamp_us == last_timestamp_us) {

      vTaskDelay(pdMS_TO_TICKS(1));

      continue;
    }

    last_timestamp_us = accel.timestamp_us;

    const size_t index = sample_index * AXES_PER_SAMPLE;

    inference_buffer[index + 0] = accel.x;

    inference_buffer[index + 1] = accel.y;

    inference_buffer[index + 2] = accel.z;

    sum_squares += static_cast<double>(accel.x) * static_cast<double>(accel.x);

    sum_squares += static_cast<double>(accel.y) * static_cast<double>(accel.y);

    sum_squares += static_cast<double>(accel.z) * static_cast<double>(accel.z);

    sample_index++;

    if (sample_index >= SAMPLE_COUNT) {

      vibration_rms = static_cast<float>(
          std::sqrt(sum_squares / static_cast<double>(SAMPLE_COUNT)));

      if (run_edge_impulse_inference()) {

        print_system_status();
      }

      sample_index = 0;

      sum_squares = 0.0;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}