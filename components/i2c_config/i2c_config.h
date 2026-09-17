#ifndef __I2C_CONFIG_H__
#define __I2C_CONFIG_H__

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PIN_NUM_SDA 17
#define PIN_NUM_SCL 18
#define VEXT_CTRL_PIN 36

void enable_vext_rail(void);

void initialize_i2c(i2c_master_bus_handle_t *i2c_bus, i2c_port_num_t i2c_port,
                    gpio_num_t sda_gpio, gpio_num_t scl_gpio);

#ifdef __cplusplus
}
#endif

#endif