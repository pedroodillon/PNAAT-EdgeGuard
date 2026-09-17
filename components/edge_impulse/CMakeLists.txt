# ============================================================
# P-101 / EDGE GUARD
# Edge Impulse component
# ESP-IDF 6.1 / ESP32-S3
# ============================================================

file(GLOB_RECURSE EI_CPP_SOURCES
    "edge-impulse-sdk/*.cpp"
    "edge-impulse-sdk/*.cc"
    "tflite-model/*.cpp"
    "tflite-model/*.cc"
)

file(GLOB_RECURSE EI_C_SOURCES
    "edge-impulse-sdk/*.c"
    "tflite-model/*.c"
)

idf_component_register(
    SRCS
        ${EI_CPP_SOURCES}
        ${EI_C_SOURCES}

    INCLUDE_DIRS
        "."
        "edge-impulse-sdk"
        "edge-impulse-sdk/classifier"
        "edge-impulse-sdk/dsp"
        "edge-impulse-sdk/porting"
        "model-parameters"
        "tflite-model"

    REQUIRES
        esp_timer
        freertos
        log
)

target_compile_options(
    ${COMPONENT_LIB}
    PRIVATE
        -Wno-unused-function
        -Wno-unused-variable
        -Wno-missing-field-initializers
        -Wno-error=maybe-uninitialized
        -Wno-error=deprecated-declarations
        -Wno-error=stringop-overflow
        -Wno-error=array-bounds
)

if(CONFIG_IDF_TARGET_ESP32S3)
    target_compile_options(
        ${COMPONENT_LIB}
        PRIVATE
            -mlongcalls
            -fno-unroll-loops
    )
endif()

target_compile_definitions(
    ${COMPONENT_LIB}
    PUBLIC
        EIDSP_USE_ESP_DSP=0

    PRIVATE
        EI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN=0
)