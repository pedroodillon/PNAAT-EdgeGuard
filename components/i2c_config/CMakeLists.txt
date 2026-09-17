idf_component_register(
    SRCS "i2c_config.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio
        esp_driver_i2c
    PRIV_REQUIRES
        freertos
        log
)