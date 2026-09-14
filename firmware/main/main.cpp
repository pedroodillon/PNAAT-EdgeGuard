#include <cstdio>

#include <ds3231.h>
#include <i2c.hpp>
#include <inmp441.h>
#include <mqttw.hpp>
#include <sdcard.hpp>
#include <wifi.hpp>

extern "C" void app_main(void) {
	uint16_t sample_rate = 16000, buffer_size = 512;

	i2c::master::init();
	ds3231_init();
	sdcard::init(SDCARD_API_SDSPI);
	inmp441_init(sample_rate, buffer_size);
	wifi::init();
	mqttw::start();
}
