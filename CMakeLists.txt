idf_component_register(
    SRCS "src/pn7160.cpp" "src/spi.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
)
target_compile_definitions(${COMPONENT_LIB} PUBLIC "-DLOG_LOCAL_LEVEL=ESP_LOG_VERBOSE")
