#include "inmp441.h"

#include <math.h>
#include <stdint.h>

#include "driver/i2s_std.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"

static const char *TAG = "INMP441";

static i2s_chan_handle_t s_rx_channel = NULL;

esp_err_t inmp441_init(const inmp441_config_t *config) {
  if (config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  i2s_chan_config_t channel_config =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

  esp_err_t err = i2s_new_channel(&channel_config, NULL, &s_rx_channel);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Falha criando canal I2S: %s", esp_err_to_name(err));
    return err;
  }

  i2s_std_config_t std_config = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(config->sample_rate),

      /*
       * Para diagnostico usamos STEREO.
       *
       * Assim conseguimos observar tanto
       * LEFT quanto RIGHT independentemente
       * do estado do pino L/R.
       */
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                      I2S_SLOT_MODE_STEREO),

      .gpio_cfg =
          {
              .mclk = I2S_GPIO_UNUSED,
              .bclk = config->bclk_gpio,
              .ws = config->ws_gpio,
              .dout = I2S_GPIO_UNUSED,
              .din = config->data_gpio,

              .invert_flags =
                  {
                      .mclk_inv = false,
                      .bclk_inv = false,
                      .ws_inv = false,
                  },
          },
  };

  std_config.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;

  err = i2s_channel_init_std_mode(s_rx_channel, &std_config);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Falha configurando I2S: %s", esp_err_to_name(err));
    return err;
  }

  err = i2s_channel_enable(s_rx_channel);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Falha habilitando I2S: %s", esp_err_to_name(err));
    return err;
  }

  ESP_LOGI(TAG, "INMP441 I2S iniciado em %lu Hz",
           (unsigned long)config->sample_rate);

  ESP_LOGI(TAG, "BCLK GPIO%d | WS GPIO%d | DATA GPIO%d", config->bclk_gpio,
           config->ws_gpio, config->data_gpio);

  return ESP_OK;
}

esp_err_t inmp441_read_samples(int32_t *buffer, size_t sample_count,
                               size_t *samples_read) {
  if (buffer == NULL || samples_read == NULL || s_rx_channel == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  size_t bytes_read = 0;

  esp_err_t err =
      i2s_channel_read(s_rx_channel, buffer, sample_count * sizeof(int32_t),
                       &bytes_read, portMAX_DELAY);

  if (err != ESP_OK) {
    return err;
  }

  *samples_read = bytes_read / sizeof(int32_t);

  return ESP_OK;
}

float inmp441_calculate_rms(const int32_t *buffer, size_t sample_count) {
  if (buffer == NULL || sample_count == 0) {
    return 0.0f;
  }

  double sum = 0.0;

  for (size_t i = 0; i < sample_count; i++) {

    const int32_t sample = buffer[i] >> 8;

    const double normalized = (double)sample / 8388608.0;

    sum += normalized * normalized;
  }

  return (float)sqrt(sum / (double)sample_count);
}