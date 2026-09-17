idf_component_register(
    SRCS
        "bno085.c"

    INCLUDE_DIRS
        "."

    REQUIRES
        esp_driver_gpio
        esp_driver_i2c

    PRIV_REQUIRES
        sh2
        esp_timer
        log
        freertos
)