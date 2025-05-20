idf_component_register(
    SRCS "src/pn7160_spi.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
)