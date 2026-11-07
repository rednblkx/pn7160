# Keep existing SPI-only builds usable on older SDKs.
set(pn7160_sources "src/pn7160.cpp" "src/spi.cpp")
set(pn7160_requires esp_driver_gpio esp_driver_spi esp_timer)
if(IDF_VERSION_MAJOR GREATER 5 OR
   (IDF_VERSION_MAJOR EQUAL 5 AND IDF_VERSION_MINOR GREATER_EQUAL 4))
    list(APPEND pn7160_sources "src/i2c.cpp")
    list(APPEND pn7160_requires esp_driver_i2c)
endif()
idf_component_register(
    SRCS ${pn7160_sources}
    INCLUDE_DIRS "include"
    REQUIRES ${pn7160_requires}
)
target_compile_definitions(${COMPONENT_LIB} PRIVATE "-DLOG_LOCAL_LEVEL=ESP_LOG_VERBOSE")
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_23)
