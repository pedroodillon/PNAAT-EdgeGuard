idf_component_register(
    SRCS
        "main.cpp"

    INCLUDE_DIRS
        "."

    REQUIRES
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        mqtt
        esp_adc
        esp_timer
        driver
        freertos
        log
        inmp441
        bno085
        i2c_config
        edge_impulse
)

target_compile_options(
    ${COMPONENT_LIB}
    PRIVATE
        -Wno-error=missing-field-initializers
        -Wno-error=deprecated-declarations
)