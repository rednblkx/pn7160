idf_component_register(
    SRCS "src/pn7160.cpp" "src/spi.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_spi esp_timer
)
target_compile_definitions(${COMPONENT_LIB} PUBLIC "-DLOG_LOCAL_LEVEL=ESP_LOG_VERBOSE")
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_23)
