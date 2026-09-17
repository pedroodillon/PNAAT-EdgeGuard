#include <cstdio>
#include <cstring>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

// ============================================================
// P-101 / EDGE GUARD
// TESTE MQTT
// ============================================================

// ------------------------------------------------------------
// ALTERE SOMENTE ESTAS DUAS LINHAS
// ------------------------------------------------------------

#define WIFI_SSID "EdgeGuard"
#define WIFI_PASSWORD "EdgeGuard-2@26"

// ------------------------------------------------------------
// BROKER MQTT
// ------------------------------------------------------------
//
// Broker público apenas para validação.
// NÃO publicar senhas, certificados ou dados sensíveis.
//
// Tópico exclusivo do P-101:
//
// p101/edgeguard/telemetry
//
// ------------------------------------------------------------

#define MQTT_BROKER_URI "mqtt://broker.emqx.io:1883"

#define MQTT_TOPIC "p101/edgeguard/telemetry"

// ============================================================
// EVENTOS
// ============================================================

#define WIFI_CONNECTED_BIT BIT0

static EventGroupHandle_t wifi_event_group;

static const char *TAG_WIFI = "P101_WIFI";

static const char *TAG_MQTT = "P101_MQTT";

static esp_mqtt_client_handle_t mqtt_client = nullptr;

static bool mqtt_connected = false;

// ============================================================
// WIFI EVENT HANDLER
// ============================================================

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {

    ESP_LOGI(TAG_WIFI, "Wi-Fi iniciado. Conectando...");

    esp_wifi_connect();

    return;
  }

  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {

    ESP_LOGW(TAG_WIFI, "Wi-Fi desconectado. Reconectando...");

    mqtt_connected = false;

    esp_wifi_connect();

    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);

    return;
  }

  if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {

    const ip_event_got_ip_t *event =
        static_cast<ip_event_got_ip_t *>(event_data);

    ESP_LOGI(TAG_WIFI, "IP obtido: " IPSTR, IP2STR(&event->ip_info.ip));

    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);

    return;
  }
}

// ============================================================
// INICIALIZACAO DO WIFI
// ============================================================

static void wifi_init(void) {
  printf("\n");
  printf("========================================\n");
  printf("WIFI\n");
  printf("========================================\n");

  wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());

  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             &wifi_event_handler, nullptr));

  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             &wifi_event_handler, nullptr));

  wifi_config_t wifi_config = {};

  std::strncpy(reinterpret_cast<char *>(wifi_config.sta.ssid), WIFI_SSID,
               sizeof(wifi_config.sta.ssid) - 1);

  std::strncpy(reinterpret_cast<char *>(wifi_config.sta.password),
               WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);

  wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

  ESP_ERROR_CHECK(esp_wifi_start());

  printf("SSID.............: %s\n", WIFI_SSID);

  printf("Aguardando conexao...\n");

  xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE,
                      portMAX_DELAY);

  printf("Wi-Fi conectado.\n");

  printf("========================================\n\n");
}

// ============================================================
// MQTT EVENT HANDLER
// ============================================================

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  esp_mqtt_event_handle_t event =
      static_cast<esp_mqtt_event_handle_t>(event_data);

  switch (static_cast<esp_mqtt_event_id_t>(event_id)) {

  case MQTT_EVENT_CONNECTED: {
    mqtt_connected = true;

    ESP_LOGI(TAG_MQTT, "MQTT conectado");

    printf("\n");
    printf("========================================\n");
    printf("MQTT CONECTADO\n");
    printf("Broker: %s\n", MQTT_BROKER_URI);
    printf("Topic : %s\n", MQTT_TOPIC);
    printf("========================================\n\n");

    break;
  }

  case MQTT_EVENT_DISCONNECTED: {
    mqtt_connected = false;

    ESP_LOGW(TAG_MQTT, "MQTT desconectado");

    break;
  }

  case MQTT_EVENT_PUBLISHED: {
    ESP_LOGI(TAG_MQTT, "Mensagem publicada. ID=%d", event->msg_id);

    break;
  }

  case MQTT_EVENT_ERROR: {
    ESP_LOGE(TAG_MQTT, "MQTT_EVENT_ERROR");

    break;
  }

  default: {
    break;
  }
  }
}

// ============================================================
// INICIALIZACAO MQTT
// ============================================================

static void mqtt_init(void) {
  printf("\n");
  printf("========================================\n");
  printf("MQTT\n");
  printf("========================================\n");

  esp_mqtt_client_config_t mqtt_cfg = {};

  mqtt_cfg.broker.address.uri = MQTT_BROKER_URI;

  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

  if (mqtt_client == nullptr) {

    ESP_LOGE(TAG_MQTT, "Falha criando cliente MQTT");

    return;
  }

  ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY,
                                                 mqtt_event_handler, nullptr));

  ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));

  printf("Cliente MQTT iniciado.\n");

  printf("Aguardando conexao com broker...\n");

  printf("========================================\n\n");
}

// ============================================================
// PUBLICACAO MQTT
// ============================================================

static void mqtt_publish_test(uint32_t counter) {
  if (!mqtt_connected || mqtt_client == nullptr) {

    ESP_LOGW(TAG_MQTT, "MQTT ainda nao conectado");

    return;
  }

  char payload[256];

  std::snprintf(payload, sizeof(payload),

                "{"
                "\"device\":\"P-101\","
                "\"status\":\"online\","
                "\"counter\":%lu"
                "}",

                static_cast<unsigned long>(counter));

  int msg_id =
      esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, payload, 0, 1, 0);

  printf("MQTT PUB | id=%d | %s\n", msg_id, payload);
}

// ============================================================
// APP MAIN
// ============================================================

extern "C" void app_main(void) {
  printf("\n");
  printf("========================================\n");
  printf("P-101 / EDGE GUARD\n");
  printf("Teste MQTT\n");
  printf("========================================\n");
  printf("Broker.....: %s\n", MQTT_BROKER_URI);
  printf("Topic......: %s\n", MQTT_TOPIC);
  printf("Periodo....: 2 segundos\n");
  printf("========================================\n");

  // ========================================================
  // NVS
  // ========================================================

  esp_err_t ret = nvs_flash_init();

  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

    ESP_ERROR_CHECK(nvs_flash_erase());

    ret = nvs_flash_init();
  }

  ESP_ERROR_CHECK(ret);

  // ========================================================
  // WIFI
  // ========================================================

  wifi_init();

  // ========================================================
  // MQTT
  // ========================================================

  mqtt_init();

  // ========================================================
  // LOOP
  // ========================================================

  uint32_t counter = 0;

  while (true) {

    counter++;

    mqtt_publish_test(counter);

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}