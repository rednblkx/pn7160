idf_component_register(
    SRCS "src/pn7160.cpp" "src/spi.cpp" "src/i2c.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_spi esp_driver_i2c esp_timer
)
target_compile_definitions(${COMPONENT_LIB} PRIVATE "-DLOG_LOCAL_LEVEL=ESP_LOG_VERBOSE")
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_23)
