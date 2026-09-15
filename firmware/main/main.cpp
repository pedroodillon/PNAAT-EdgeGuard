#include <cstdio>

#include <ds3231.hpp>
#include <i2c.hpp>
#include <inmp441.hpp>
#include <mqttw.hpp>
#include <sdcard.hpp>
#include <wifi.hpp>

extern "C" void app_main(void) {
	ESP_ERROR_CHECK(i2c::master::init());
	ESP_ERROR_CHECK(nvs::init());
	ESP_ERROR_CHECK(ds3231::init());
	ESP_ERROR_CHECK(sdcard::init());
	ESP_ERROR_CHECK(inmp441::init());
	ESP_ERROR_CHECK(wifi::init());
	ESP_ERROR_CHECK(mqttw::start());
}
