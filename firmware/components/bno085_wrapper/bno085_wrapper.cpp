#include "bno085_wrapper.hpp"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "bno085.h"
#include <i2c.hpp>

static const char *TAG = "BNO085_WRAPPER";

static bno085_handle_t s_handle = NULL;
static void *s_device = NULL;

static void sensor_callback(bno085_handle_t handle,
							const bno085_sensor_value_t *value,
							void *user_ctx) {
	if (value->sensor_id != BNO085_SENSOR_RAW_ACCELEROMETER)
		return;
}

esp_err_t bno085::init() {
	esp_err_t err;

	bno085_config_t cfg;
	bno085_config_default(&cfg);

	s_device = i2c::master::add_device(0x4A, 400000);

	err = bno085_init(&cfg, (i2c_master_dev_handle_t)s_device,
					  (gpio_num_t)CONFIG_BNO085_WRAPPER_INT_PIN,
					  (gpio_num_t)CONFIG_BNO085_WRAPPER_RST_PIN, &s_handle);
	if (err == ESP_ERR_INVALID_ARG) {
		ESP_LOGE(TAG, "failed on init: i2c_device or bno085 handle is NULL");
		return ESP_ERR_INVALID_ARG;
	} else if (err == ESP_ERR_NO_MEM) {
		ESP_LOGE(TAG, "failed on init: no more bno085 instances slots or heap "
					  "allocation fail");
		return ESP_ERR_NO_MEM;
	} else if (err == ESP_FAIL) {
		ESP_LOGE(TAG, "failed on init: fail on sh2 initialization or hardware "
					  "communication failure");
		return ESP_ERR_NO_MEM;
	}

	err = bno085_register_sensor_callback(s_handle, sensor_callback, NULL);
	if (err == ESP_ERR_INVALID_ARG) {
		ESP_LOGE(TAG,
				 "failed on register sensor callback: bno085 handle is null");
		return ESP_ERR_NO_MEM;
	} else if (err == ESP_ERR_INVALID_STATE) {
		ESP_LOGE(TAG,
				 "failed on register sensor callback: device not initialized");
		return ESP_ERR_NO_MEM;
	}

	err = bno085_enable_sensor(s_handle, BNO085_SENSOR_RAW_ACCELEROMETER, 2000);
	if (err == ESP_ERR_INVALID_ARG) {
		ESP_LOGE(TAG, "failed on enable sensor: bno085 handle is null or "
					  "sensor_id is out of range");
		return ESP_ERR_NO_MEM;
	} else if (err == ESP_ERR_INVALID_STATE) {
		ESP_LOGE(TAG, "failed on enable sensor: device not initialized");
		return ESP_ERR_NO_MEM;
	} else if (err == ESP_FAIL) {
		ESP_LOGE(TAG, "failed on enable sensor: sh2 communication failure");
		return ESP_ERR_NO_MEM;
	}

	return ESP_OK;
}

void bno085::run_service() { bno085_service(s_handle); }
