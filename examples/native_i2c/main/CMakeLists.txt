# Derive this here as well: IDF discovers component requirements in a separate
# CMake pass, where variables from the project CMakeLists need not be present.
get_filename_component(PN7160_COMPONENT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
get_filename_component(PN7160_COMPONENT_NAME "${PN7160_COMPONENT_DIR}" NAME)
idf_component_register(SRCS "main.cpp" INCLUDE_DIRS "."
    REQUIRES ${PN7160_COMPONENT_NAME} esp_driver_i2c esp_driver_gpio)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_23)
