#ifndef __INMP441_H__
#define __INMP441_H__

#include "driver/gpio.h"
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  gpio_num_t bclk_gpio;
  gpio_num_t ws_gpio;
  gpio_num_t data_gpio;
  uint32_t sample_rate;
} inmp441_config_t;

esp_err_t inmp441_init(const inmp441_config_t *config);

esp_err_t inmp441_read_samples(int32_t *buffer, size_t sample_count,
                               size_t *samples_read);

float inmp441_calculate_rms(const int32_t *buffer, size_t sample_count);

#ifdef __cplusplus
}
#endif

#endif