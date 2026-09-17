#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

#include "mqtt_client.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "lwip/ip4_addr.h"
#include "lwip/netdb.h"

#include "aws_iot_config.h"
#include "wifi_config.h"

// ============================================================
// P-101 / EDGE GUARD
// AWS IoT Core + ACS712-20A
// ============================================================

// ============================================================
// ACS712
//
// Heltec WiFi LoRa 32 V3
// GPIO1 = ADC1_CH0
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
// CERTIFICADOS AWS
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
// TAGS
// ============================================================

static const char *TAG_WIFI = "P101_WIFI";
static const char *TAG_DNS = "P101_DNS";
static const char *TAG_AWS = "P101_AWS";
static const char *TAG_ACS = "P101_ACS712";

// ============================================================
// WIFI / MQTT
// ============================================================

#define WIFI_CONNECTED_BIT BIT0

static EventGroupHandle_t wifi_event_group = nullptr;

static esp_netif_t *wifi_sta_netif = nullptr;

static esp_mqtt_client_handle_t mqtt_client = nullptr;

static bool mqtt_connected = false;

// ============================================================
// ACS712 - ESTADO
// ============================================================

typedef enum {
  MOTOR_STATE_OFF = 0,
  MOTOR_STATE_STARTUP,
  MOTOR_STATE_STEADY

} motor_state_t;

static motor_state_t motor_state = MOTOR_STATE_OFF;

static adc_oneshot_unit_handle_t adc_handle = nullptr;

static adc_cali_handle_t adc_cali_handle = nullptr;

static float zero_voltage_mv = 0.0f;

static float current_a = 0.0f;

static float startup_peak_current_a = 0.0f;

static int64_t motor_start_time_us = 0;

// ============================================================
// NOME DO ESTADO
// ============================================================

static const char *motor_state_to_string(motor_state_t state) {
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
// ADC CALIBRATION
// ============================================================

static bool init_adc_calibration(void) {
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

static void init_adc(void) {
  printf("\n");
  printf("========================================\n");
  printf("ACS712-20A\n");
  printf("========================================\n");
  printf("GPIO       : 1\n");
  printf("ADC        : ADC1_CH0\n");

  printf("Sensibilidade ADC: %.2f mV/A\n", ACS712_SENSITIVITY_ADC_MV_PER_A);

  printf("========================================\n");

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
// LEITURA ADC
// ============================================================

static float read_adc_voltage_mv(void) {
  int raw = 0;

  esp_err_t err = adc_oneshot_read(adc_handle, ACS712_ADC_CHANNEL, &raw);

  if (err != ESP_OK) {
    ESP_LOGE(TAG_ACS, "Erro lendo ADC: %s", esp_err_to_name(err));

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

// ============================================================
// MEDIA ADC
// ============================================================

static float average_voltage_mv(size_t samples) {
  double sum = 0.0;

  for (size_t i = 0; i < samples; i++) {
    sum += read_adc_voltage_mv();

    vTaskDelay(pdMS_TO_TICKS(ADC_SAMPLE_INTERVAL_MS));
  }

  return static_cast<float>(sum / static_cast<double>(samples));
}

// ============================================================
// CALIBRACAO ZERO
// ============================================================

static void calibrate_acs712_zero(void) {
  printf("\n");
  printf("========================================\n");
  printf("ACS712 - CALIBRACAO ZERO\n");
  printf("========================================\n");
  printf("Mantenha o motor desligado.\n");

  printf("Coletando %d amostras...\n", ZERO_CALIBRATION_SAMPLES);

  zero_voltage_mv = average_voltage_mv(ZERO_CALIBRATION_SAMPLES);

  printf("Zero ACS712: %.2f mV\n", zero_voltage_mv);

  printf("========================================\n\n");
}

// ============================================================
// TENSAO -> CORRENTE
// ============================================================

static float voltage_to_current_a(float voltage_mv) {
  float delta_mv = zero_voltage_mv - voltage_mv;

  float calculated_current = fabsf(delta_mv) / ACS712_SENSITIVITY_ADC_MV_PER_A;

  if (calculated_current < CURRENT_DEADBAND_A) {
    calculated_current = 0.0f;
  }

  return calculated_current;
}

// ============================================================
// LEITURA CORRENTE
// ============================================================

static float read_current_a(void) {
  float voltage_mv = average_voltage_mv(CURRENT_AVERAGE_SAMPLES);

  return voltage_to_current_a(voltage_mv);
}

// ============================================================
// ESTADO DO MOTOR
// ============================================================

static void update_motor_state(float measured_current) {
  int64_t now_us = esp_timer_get_time();

  switch (motor_state) {
  case MOTOR_STATE_OFF: {
    if (measured_current >= MOTOR_ON_THRESHOLD_A) {
      motor_state = MOTOR_STATE_STARTUP;

      motor_start_time_us = now_us;

      startup_peak_current_a = measured_current;

      ESP_LOGI(TAG_ACS, "Motor detectado -> STARTUP");
    }

    break;
  }

  case MOTOR_STATE_STARTUP: {
    if (measured_current > startup_peak_current_a) {
      startup_peak_current_a = measured_current;
    }

    if (measured_current <= MOTOR_OFF_THRESHOLD_A) {
      motor_state = MOTOR_STATE_OFF;

      startup_peak_current_a = 0.0f;

      ESP_LOGI(TAG_ACS, "Motor -> OFF");

      break;
    }

    int64_t elapsed_ms = (now_us - motor_start_time_us) / 1000;

    if (elapsed_ms >= MOTOR_STARTUP_TIME_MS) {
      motor_state = MOTOR_STATE_STEADY;

      ESP_LOGI(TAG_ACS, "Motor -> STEADY");
    }

    break;
  }

  case MOTOR_STATE_STEADY: {
    if (measured_current <= MOTOR_OFF_THRESHOLD_A) {
      motor_state = MOTOR_STATE_OFF;

      startup_peak_current_a = 0.0f;

      ESP_LOGI(TAG_ACS, "Motor -> OFF");
    }

    break;
  }

  default: {
    motor_state = MOTOR_STATE_OFF;

    break;
  }
  }
}

// ============================================================
// DNS
// ============================================================

static void configure_dns(void) {
  if (wifi_sta_netif == nullptr) {
    ESP_LOGE(TAG_DNS, "Interface Wi-Fi invalida");

    return;
  }

  esp_netif_dns_info_t dns_main = {};

  dns_main.ip.type = ESP_IPADDR_TYPE_V4;

  IP4_ADDR(&dns_main.ip.u_addr.ip4, 8, 8, 8, 8);

  esp_err_t err =
      esp_netif_set_dns_info(wifi_sta_netif, ESP_NETIF_DNS_MAIN, &dns_main);

  if (err == ESP_OK) {
    ESP_LOGI(TAG_DNS, "DNS principal: 8.8.8.8");
  } else {
    ESP_LOGW(TAG_DNS, "Falha configurando DNS: %s", esp_err_to_name(err));
  }
}

// ============================================================
// WIFI EVENT
// ============================================================

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  (void)arg;

  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    ESP_LOGI(TAG_WIFI, "Wi-Fi iniciado");

    esp_wifi_connect();

    return;
  }

  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    const wifi_event_sta_disconnected_t *event =
        static_cast<wifi_event_sta_disconnected_t *>(event_data);

    mqtt_connected = false;

    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);

    ESP_LOGW(TAG_WIFI, "Wi-Fi desconectado. reason=%d", event->reason);

    esp_wifi_connect();

    return;
  }

  if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    const ip_event_got_ip_t *event =
        static_cast<ip_event_got_ip_t *>(event_data);

    ESP_LOGI(TAG_WIFI, "IP: " IPSTR, IP2STR(&event->ip_info.ip));

    ESP_LOGI(TAG_WIFI, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));

    configure_dns();

    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);

    return;
  }
}

// ============================================================
// WIFI INIT
// ============================================================

static void wifi_init(void) {
  printf("\n");
  printf("========================================\n");
  printf("WIFI\n");
  printf("========================================\n");
  printf("SSID: %s\n", WIFI_SSID);

  wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());

  wifi_sta_netif = esp_netif_create_default_wifi_sta();

  if (wifi_sta_netif == nullptr) {
    ESP_LOGE(TAG_WIFI, "Falha criando interface Wi-Fi");

    return;
  }

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

  printf("Wi-Fi conectado.\n");

  printf("========================================\n");
}

// ============================================================
// MQTT EVENT
// ============================================================

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  (void)handler_args;
  (void)base;

  esp_mqtt_event_handle_t event =
      static_cast<esp_mqtt_event_handle_t>(event_data);

  switch (static_cast<esp_mqtt_event_id_t>(event_id)) {
  case MQTT_EVENT_CONNECTED: {
    mqtt_connected = true;

    printf("\n");
    printf("========================================\n");
    printf("AWS IOT CORE CONECTADO\n");
    printf("========================================\n");

    printf("Client ID: %s\n", AWS_IOT_CLIENT_ID);

    printf("Topic    : %s\n", AWS_IOT_TOPIC);

    printf("========================================\n");

    ESP_LOGI(TAG_AWS, "AWS IoT Core conectado");

    break;
  }

  case MQTT_EVENT_DISCONNECTED: {
    mqtt_connected = false;

    ESP_LOGW(TAG_AWS, "AWS desconectado");

    break;
  }

  case MQTT_EVENT_PUBLISHED: {
    ESP_LOGI(TAG_AWS, "Publicado msg_id=%d", event->msg_id);

    break;
  }

  case MQTT_EVENT_ERROR: {
    ESP_LOGE(TAG_AWS, "MQTT_EVENT_ERROR");

    if (event->error_handle != nullptr) {
      ESP_LOGE(TAG_AWS, "error_type=%d", event->error_handle->error_type);

      ESP_LOGE(TAG_AWS, "tls_err=0x%x",
               event->error_handle->esp_tls_last_esp_err);

      ESP_LOGE(TAG_AWS, "stack_err=0x%x",
               event->error_handle->esp_tls_stack_err);

      ESP_LOGE(TAG_AWS, "sock_errno=%d",
               event->error_handle->esp_transport_sock_errno);
    }

    break;
  }

  default:
    break;
  }
}

// ============================================================
// MQTT INIT
// ============================================================

static void mqtt_init(void) {
  printf("\n");
  printf("========================================\n");
  printf("AWS IoT Core\n");
  printf("========================================\n");

  printf("Host : %s\n", AWS_IOT_HOST);

  printf("Port : %d\n", AWS_IOT_PORT);

  printf("Topic: %s\n", AWS_IOT_TOPIC);

  printf("========================================\n");

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

  if (mqtt_client == nullptr) {
    ESP_LOGE(TAG_AWS, "Falha criando cliente MQTT");

    return;
  }

  ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY,
                                                 mqtt_event_handler, nullptr));

  ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
}

// ============================================================
// PUBLICACAO AWS
// ============================================================

static void publish_telemetry(void) {
  if (mqtt_client == nullptr || !mqtt_connected) {
    ESP_LOGW(TAG_AWS, "AWS ainda nao conectado");

    return;
  }

  char payload[384];

  std::snprintf(payload, sizeof(payload),

                "{"
                "\"device_id\":\"%s\","
                "\"status\":\"online\","
                "\"current_a\":%.3f,"
                "\"motor_state\":\"%s\","
                "\"startup_peak_a\":%.3f"
                "}",

                AWS_IOT_CLIENT_ID,

                current_a,

                motor_state_to_string(motor_state),

                startup_peak_current_a);

  int msg_id =
      esp_mqtt_client_publish(mqtt_client, AWS_IOT_TOPIC, payload, 0, 1, 0);

  printf("AWS PUB | id=%d | %s\n", msg_id, payload);
}

// ============================================================
// APP MAIN
// ============================================================

extern "C" void app_main(void) {
  printf("\n");
  printf("========================================\n");
  printf("P-101 / EDGE GUARD\n");
  printf("AWS IoT + ACS712\n");
  printf("========================================\n");

  esp_err_t ret = nvs_flash_init();

  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());

    ret = nvs_flash_init();
  }

  ESP_ERROR_CHECK(ret);

  // Motor deve estar desligado durante a calibracao.

  init_adc();

  calibrate_acs712_zero();

  wifi_init();

  mqtt_init();

  while (true) {
    current_a = read_current_a();

    update_motor_state(current_a);

    printf("ACS712 | I=%.3f A | STATE=%s | PEAK=%.3f A\n", current_a,
           motor_state_to_string(motor_state), startup_peak_current_a);

    publish_telemetry();

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}