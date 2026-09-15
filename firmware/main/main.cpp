#include <cstdio>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <bno085_wrapper.hpp>
#include <ds3231.hpp>
#include <i2c.hpp>
#include <inmp441.hpp>
#include <mqttw.hpp>
#include <sdcard.hpp>
#include <wifi.hpp>

#define BNO085_ELEMENTS_PER_HZ 3
#define BNO085_CAPACITY 500
#define BNO085_STRIDE 750

// static int16_t bno085_buffer[BNO085_CAPACITY * BNO085_ELEMENTS_PER_HZ];

static SemaphoreHandle_t s_audio_semphr = NULL;
static int32_t *s_sample_buffer = NULL;
static int16_t *s_audio_ctx = NULL;
static int16_t s_stride = 500;
static int16_t s_audio_ctx_idx = 0;

static void audio_reader() {
	while (true) {
		inmp441::read_samples(s_sample_buffer);

		for (int16_t i = 0; i < CONFIG_INMP441_BLOCK_SIZE; i++) {
			s_audio_ctx[i] = s_sample_buffer[i] >> 16;
			s_audio_ctx_idx++;
		}

		if (s_audio_ctx_idx >= CONFIG_INMP441_SAMPLE_RATE) {
			s_audio_ctx_idx -= s_stride;
			xSemaphoreGive(s_audio_semphr);
		}
	}
}

extern "C" void app_main(void) {
	s_sample_buffer =
		(int32_t *)malloc(CONFIG_INMP441_BLOCK_SIZE * sizeof(int32_t));
	s_audio_ctx =
		(int16_t *)malloc(CONFIG_INMP441_SAMPLE_RATE * sizeof(int16_t));

	ESP_ERROR_CHECK(nvs::init());
	ESP_ERROR_CHECK(i2c::master::init());
	ESP_ERROR_CHECK(ds3231::init());
	ESP_ERROR_CHECK(bno085::init());
	ESP_ERROR_CHECK(inmp441::init());
	ESP_ERROR_CHECK(sdcard::init());
	ESP_ERROR_CHECK(wifi::init());
	ESP_ERROR_CHECK(mqttw::start());

	// set ds3231
}
