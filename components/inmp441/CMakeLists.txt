idf_component_register(
    SRCS
        "inmp441.c"

    INCLUDE_DIRS
        "."

    REQUIRES
        esp_driver_gpio
        esp_driver_i2s
        freertos
        log
)