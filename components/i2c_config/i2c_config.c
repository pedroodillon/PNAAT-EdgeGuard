#include "i2c_config.h"

#include <stdbool.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char TAG[] = "i2c_config";

void enable_vext_rail(void) {
  static bool s_vext_enabled = false;

  if (s_vext_enabled) {
    return;
  }

  gpio_config_t vext_conf = {
      .pin_bit_mask = 1ULL << VEXT_CTRL_PIN,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  ESP_ERROR_CHECK(gpio_config(&vext_conf));
  ESP_ERROR_CHECK(gpio_set_level(VEXT_CTRL_PIN, 0));

  vTaskDelay(pdMS_TO_TICKS(50));

  s_vext_enabled = true;

  ESP_LOGI(TAG, "Vext rail enabled");
}

void initialize_i2c(i2c_master_bus_handle_t *i2c_bus, i2c_port_num_t i2c_port,
                    gpio_num_t sda_gpio, gpio_num_t scl_gpio) {
  ESP_LOGI(TAG, "Initializing I2C port %d, SDA=%d, SCL=%d", i2c_port, sda_gpio,
           scl_gpio);

  i2c_master_bus_config_t bus_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = i2c_port,
      .sda_io_num = sda_gpio,
      .scl_io_num = scl_gpio,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, i2c_bus));

  ESP_LOGI(TAG, "I2C initialized");
}