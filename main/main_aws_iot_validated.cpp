#include <cstdint>
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

#include "lwip/ip4_addr.h"
#include "lwip/netdb.h"


#include "aws_iot_config.h"
#include "wifi_config.h"


// ============================================================
// CERTIFICADOS EMBUTIDOS
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
// GLOBAIS
// ============================================================

#define WIFI_CONNECTED_BIT BIT0

static EventGroupHandle_t wifi_event_group = nullptr;

static const char *TAG_WIFI = "P101_WIFI";
static const char *TAG_DNS = "P101_DNS";
static const char *TAG_MQTT = "P101_AWS";

static esp_mqtt_client_handle_t mqtt_client = nullptr;
static esp_netif_t *wifi_sta_netif = nullptr;

static bool mqtt_connected = false;

// ============================================================
// CONFIGURA DNS MANUAL
// ============================================================

static void configure_dns(void) {
  if (wifi_sta_netif == nullptr) {
    ESP_LOGE(TAG_DNS, "Interface Wi-Fi invalida");

    return;
  }

  // --------------------------------------------------------
  // DNS principal: Google
  // --------------------------------------------------------

  esp_netif_dns_info_t dns_main = {};

  dns_main.ip.type = ESP_IPADDR_TYPE_V4;

  IP4_ADDR(&dns_main.ip.u_addr.ip4, 8, 8, 8, 8);

  esp_err_t ret =
      esp_netif_set_dns_info(wifi_sta_netif, ESP_NETIF_DNS_MAIN, &dns_main);

  if (ret == ESP_OK) {
    ESP_LOGI(TAG_DNS, "DNS principal configurado: 8.8.8.8");
  } else {
    ESP_LOGE(TAG_DNS, "Falha configurando DNS principal: %s",
             esp_err_to_name(ret));
  }

  // --------------------------------------------------------
  // DNS secundario: Cloudflare
  // --------------------------------------------------------

  esp_netif_dns_info_t dns_backup = {};

  dns_backup.ip.type = ESP_IPADDR_TYPE_V4;

  IP4_ADDR(&dns_backup.ip.u_addr.ip4, 1, 1, 1, 1);

  ret =
      esp_netif_set_dns_info(wifi_sta_netif, ESP_NETIF_DNS_BACKUP, &dns_backup);

  if (ret == ESP_OK) {
    ESP_LOGI(TAG_DNS, "DNS secundario configurado: 1.1.1.1");
  } else {
    ESP_LOGW(TAG_DNS, "Falha configurando DNS secundario: %s",
             esp_err_to_name(ret));
  }
}

// ============================================================
// TESTE DE DNS
// ============================================================

static bool test_dns_resolution(void) {
  printf("\n");
  printf("========================================\n");
  printf("TESTE DNS\n");
  printf("========================================\n");

  printf("Hostname:\n%s\n", AWS_IOT_HOST);

  struct addrinfo hints = {};
  struct addrinfo *result = nullptr;

  hints.ai_family = AF_INET;

  hints.ai_socktype = SOCK_STREAM;

  int err = getaddrinfo(AWS_IOT_HOST, nullptr, &hints, &result);

  if (err != 0 || result == nullptr) {
    ESP_LOGE(TAG_DNS, "Falha resolvendo hostname. getaddrinfo=%d", err);

    printf("DNS: FALHA\n");
    printf("========================================\n\n");

    if (result != nullptr) {
      freeaddrinfo(result);
    }

    return false;
  }

  char ip_string[INET_ADDRSTRLEN] = {};

  struct sockaddr_in *addr =
      reinterpret_cast<struct sockaddr_in *>(result->ai_addr);

  inet_ntop(AF_INET, &addr->sin_addr, ip_string, sizeof(ip_string));

  ESP_LOGI(TAG_DNS, "Hostname resolvido");

  ESP_LOGI(TAG_DNS, "%s -> %s", AWS_IOT_HOST, ip_string);

  printf("DNS: OK\nIP AWS: %s\n", ip_string);

  printf("========================================\n\n");

  freeaddrinfo(result);

  return true;
}

// ============================================================
// WIFI EVENT HANDLER
// ============================================================

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  (void)arg;

  // --------------------------------------------------------
  // Wi-Fi iniciado
  // --------------------------------------------------------

  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    ESP_LOGI(TAG_WIFI, "Wi-Fi iniciado. Conectando...");

    esp_wifi_connect();

    return;
  }

  // --------------------------------------------------------
  // Wi-Fi desconectado
  // --------------------------------------------------------

  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    const wifi_event_sta_disconnected_t *event =
        static_cast<wifi_event_sta_disconnected_t *>(event_data);

    ESP_LOGW(TAG_WIFI, "Wi-Fi desconectado. reason=%d", event->reason);

    mqtt_connected = false;

    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);

    vTaskDelay(pdMS_TO_TICKS(1000));

    esp_wifi_connect();

    return;
  }

  // --------------------------------------------------------
  // IP obtido
  // --------------------------------------------------------

  if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    const ip_event_got_ip_t *event =
        static_cast<ip_event_got_ip_t *>(event_data);

    ESP_LOGI(TAG_WIFI, "IP obtido: " IPSTR, IP2STR(&event->ip_info.ip));

    ESP_LOGI(TAG_WIFI, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));

    // ----------------------------------------------------
    // Configura DNS antes de liberar o programa
    // ----------------------------------------------------

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
  printf("Aguardando conexao...\n");

  wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());

  // Guarda handle da interface Wi-Fi
  wifi_sta_netif = esp_netif_create_default_wifi_sta();

  if (wifi_sta_netif == nullptr) {
    ESP_LOGE(TAG_WIFI, "Falha criando interface Wi-Fi");

    return;
  }

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

  wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;

  wifi_config.sta.pmf_cfg.capable = true;

  wifi_config.sta.pmf_cfg.required = false;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

  ESP_ERROR_CHECK(esp_wifi_start());

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
    printf("========================================\n\n");

    ESP_LOGI(TAG_MQTT, "Conectado ao AWS IoT Core");

    break;
  }

  case MQTT_EVENT_DISCONNECTED: {
    mqtt_connected = false;

    ESP_LOGW(TAG_MQTT, "AWS MQTT desconectado");

    break;
  }

  case MQTT_EVENT_PUBLISHED: {
    ESP_LOGI(TAG_MQTT, "Publicado. msg_id=%d", event->msg_id);

    break;
  }

  case MQTT_EVENT_ERROR: {
    ESP_LOGE(TAG_MQTT, "MQTT_EVENT_ERROR");

    if (event->error_handle != nullptr) {
      ESP_LOGE(TAG_MQTT, "error_type=%d", event->error_handle->error_type);

      ESP_LOGE(TAG_MQTT, "esp_tls_last_esp_err=0x%x",
               event->error_handle->esp_tls_last_esp_err);

      ESP_LOGE(TAG_MQTT, "esp_tls_stack_err=0x%x",
               event->error_handle->esp_tls_stack_err);

      ESP_LOGE(TAG_MQTT, "sock_errno=%d",
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
  printf("AWS MQTT\n");
  printf("========================================\n");

  printf("Host:\n%s\n", AWS_IOT_HOST);

  printf("Porta:\n%d\n", AWS_IOT_PORT);

  printf("Client ID:\n%s\n", AWS_IOT_CLIENT_ID);

  printf("Topic:\n%s\n", AWS_IOT_TOPIC);

  printf("Aguardando conexao TLS...\n");

  printf("========================================\n\n");

  esp_mqtt_client_config_t mqtt_cfg = {};

  // --------------------------------------------------------
  // Host AWS
  // --------------------------------------------------------

  mqtt_cfg.broker.address.hostname = AWS_IOT_HOST;

  mqtt_cfg.broker.address.port = AWS_IOT_PORT;

  mqtt_cfg.broker.address.transport = MQTT_TRANSPORT_OVER_SSL;

  // --------------------------------------------------------
  // Client ID
  // --------------------------------------------------------

  mqtt_cfg.credentials.client_id = AWS_IOT_CLIENT_ID;

  // --------------------------------------------------------
  // CA Amazon
  // --------------------------------------------------------

  mqtt_cfg.broker.verification.certificate =
      reinterpret_cast<const char *>(amazon_root_ca_start);

  // --------------------------------------------------------
  // Certificado do dispositivo
  // --------------------------------------------------------

  mqtt_cfg.credentials.authentication.certificate =
      reinterpret_cast<const char *>(device_cert_start);

  // --------------------------------------------------------
  // Chave privada
  // --------------------------------------------------------

  mqtt_cfg.credentials.authentication.key =
      reinterpret_cast<const char *>(device_private_key_start);

  // --------------------------------------------------------
  // Cria cliente MQTT
  // --------------------------------------------------------

  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

  if (mqtt_client == nullptr) {
    ESP_LOGE(TAG_MQTT, "Falha criando cliente MQTT");

    return;
  }

  ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY,
                                                 mqtt_event_handler, nullptr));

  ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
}

// ============================================================
// PUBLICAÇÃO MQTT
// ============================================================

static void mqtt_publish_test(uint32_t counter) {
  if (mqtt_client == nullptr || !mqtt_connected) {
    ESP_LOGW(TAG_MQTT, "AWS MQTT ainda nao conectado");

    return;
  }

  char payload[256];

  std::snprintf(payload, sizeof(payload),
                "{"
                "\"device_id\":\"%s\","
                "\"status\":\"online\","
                "\"counter\":%lu"
                "}",
                AWS_IOT_CLIENT_ID, static_cast<unsigned long>(counter));

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
  printf("AWS IoT Core MQTT TLS Test\n");
  printf("========================================\n");

  printf("Client ID: %s\n", AWS_IOT_CLIENT_ID);

  printf("Topic    : %s\n", AWS_IOT_TOPIC);

  printf("Periodo  : 2 segundos\n");

  printf("========================================\n");

  // --------------------------------------------------------
  // NVS
  // --------------------------------------------------------

  esp_err_t ret = nvs_flash_init();

  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());

    ret = nvs_flash_init();
  }

  ESP_ERROR_CHECK(ret);

  // --------------------------------------------------------
  // Wi-Fi
  // --------------------------------------------------------

  wifi_init();

  // --------------------------------------------------------
  // Teste DNS antes do MQTT
  // --------------------------------------------------------

  bool dns_ok = test_dns_resolution();

  if (!dns_ok) {
    ESP_LOGE(TAG_DNS, "DNS ainda nao esta funcional");

    ESP_LOGE(TAG_DNS, "MQTT nao sera iniciado");

    while (true) {
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }

  // --------------------------------------------------------
  // AWS IoT
  // --------------------------------------------------------

  mqtt_init();

  // --------------------------------------------------------
  // Telemetria
  // --------------------------------------------------------

  uint32_t counter = 0;

  while (true) {
    counter++;

    mqtt_publish_test(counter);

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}