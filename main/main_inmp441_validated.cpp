#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>


#include "inmp441.h"

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define INMP441_BCLK_GPIO GPIO_NUM_39
#define INMP441_WS_GPIO GPIO_NUM_40
#define INMP441_DATA_GPIO GPIO_NUM_41

#define INMP441_SAMPLE_RATE 16000

/*
 * Stereo:
 *
 * índice par   = LEFT
 * índice ímpar = RIGHT
 */
#define AUDIO_FRAMES 256
#define AUDIO_WORDS (AUDIO_FRAMES * 2)

static const char *TAG = "P101_AUDIO_DIAG";

static int32_t audio_buffer[AUDIO_WORDS];

extern "C" void app_main(void) {
  printf("\n");
  printf("========================================\n");
  printf("P-101 / EDGE GUARD\n");
  printf("Diagnostico INMP441 I2S\n");
  printf("========================================\n");

  printf("BCLK : GPIO%d\n", INMP441_BCLK_GPIO);
  printf("WS   : GPIO%d\n", INMP441_WS_GPIO);
  printf("DATA : GPIO%d\n", INMP441_DATA_GPIO);
  printf("FS   : %d Hz\n", INMP441_SAMPLE_RATE);

  printf("========================================\n\n");

  inmp441_config_t config = {
      .bclk_gpio = INMP441_BCLK_GPIO,
      .ws_gpio = INMP441_WS_GPIO,
      .data_gpio = INMP441_DATA_GPIO,
      .sample_rate = INMP441_SAMPLE_RATE,
  };

  esp_err_t err = inmp441_init(&config);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Falha inicializando I2S: %s", esp_err_to_name(err));
    return;
  }

  printf("\n");
  printf("Fale perto do microfone durante o teste.\n");
  printf("\n");

  while (true) {

    size_t words_read = 0;

    err = inmp441_read_samples(audio_buffer, AUDIO_WORDS, &words_read);

    if (err != ESP_OK) {

      ESP_LOGE(TAG, "Erro I2S: %s", esp_err_to_name(err));

      vTaskDelay(pdMS_TO_TICKS(500));

      continue;
    }

    int32_t left_min = INT32_MAX;
    int32_t left_max = INT32_MIN;

    int32_t right_min = INT32_MAX;
    int32_t right_max = INT32_MIN;

    uint64_t left_nonzero = 0;
    uint64_t right_nonzero = 0;

    double left_sum = 0.0;
    double right_sum = 0.0;

    size_t left_count = 0;
    size_t right_count = 0;

    for (size_t i = 0; i + 1 < words_read; i += 2) {

      const int32_t left = audio_buffer[i];

      const int32_t right = audio_buffer[i + 1];

      if (left < left_min) {
        left_min = left;
      }

      if (left > left_max) {
        left_max = left;
      }

      if (right < right_min) {
        right_min = right;
      }

      if (right > right_max) {
        right_max = right;
      }

      if (left != 0) {
        left_nonzero++;
      }

      if (right != 0) {
        right_nonzero++;
      }

      const double left24 = (double)(left >> 8);

      const double right24 = (double)(right >> 8);

      left_sum += left24 * left24;

      right_sum += right24 * right24;

      left_count++;
      right_count++;
    }

    float left_rms = 0.0f;
    float right_rms = 0.0f;

    if (left_count > 0) {
      left_rms = (float)sqrt(left_sum / (double)left_count) / 8388608.0f;
    }

    if (right_count > 0) {
      right_rms = (float)sqrt(right_sum / (double)right_count) / 8388608.0f;
    }

    printf("LEFT  RMS=%0.6f NZ=%llu MIN=%ld MAX=%ld\n", left_rms, left_nonzero,
           (long)left_min, (long)left_max);

    printf("RIGHT RMS=%0.6f NZ=%llu MIN=%ld MAX=%ld\n", right_rms,
           right_nonzero, (long)right_min, (long)right_max);

    printf("RAW: %ld %ld %ld %ld\n\n", (long)audio_buffer[0],
           (long)audio_buffer[1], (long)audio_buffer[2], (long)audio_buffer[3]);

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}