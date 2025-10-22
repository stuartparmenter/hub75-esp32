# ESP32 HUB75 DMA Driver
# CMakeLists.txt for ESP-IDF component

idf_component_register(
    SRCS
        "src/core/hub75_driver.cpp"
        "src/color/color_lut.cpp"
        "src/color/color_convert.cpp"
        "src/platforms/platform_detect.cpp"
        # Platform-specific sources added conditionally below
    INCLUDE_DIRS
        "include"
        "src"
    REQUIRES
        driver
        esp_timer
        freertos
)

# Platform-specific source files
if(CONFIG_IDF_TARGET_ESP32 OR CONFIG_IDF_TARGET_ESP32S2)
    target_sources(${COMPONENT_LIB} PRIVATE
        "src/platforms/i2s/dma_i2s.cpp"
    )
elseif(CONFIG_IDF_TARGET_ESP32S3)
    target_sources(${COMPONENT_LIB} PRIVATE
        "src/platforms/gdma/dma_gdma.cpp"
    )
elseif(CONFIG_IDF_TARGET_ESP32P4)
    target_sources(${COMPONENT_LIB} PRIVATE
        "src/platforms/parlio/dma_parlio.cpp"
    )
endif()

# Compiler flags
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
)

# Enable debug logging for the library
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    LOG_LOCAL_LEVEL=ESP_LOG_DEBUG
)
