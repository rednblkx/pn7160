#include "i2c.hpp"
#include "pn7160.hpp"
#include "esp_log.h"

// Example ESP32 wiring; adapt to your board. The application owns this bus.
extern "C" void app_main() {
    i2c_master_bus_config_t bus_config{};
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.sda_io_num = GPIO_NUM_21;
    bus_config.scl_io_num = GPIO_NUM_22;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;
    i2c_master_bus_handle_t bus = nullptr;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus));
    {
        PN7160_I2C_Config config;
        config.bus = bus;
        config.irq = GPIO_NUM_14;
        config.ven = GPIO_NUM_13;
        config.dwl_req = GPIO_NUM_19;
        PN7160_I2C transport(config);
        PN7160_NCI nci(transport);
        // Includes CORE_RESET and CORE_INIT, without discovery or enrollment.
        const esp_err_t error = nci.initialize();
        if (error == ESP_OK)
            ESP_LOGI("example", "CORE_RESET/CORE_INIT succeeded; firmware 0x%04X", nci.get_firmware_version());
        else ESP_LOGE("example", "NCI initialization failed: 0x%X", error);
        nci.stop();
    } // NCI and transport destroyed before caller deletes its own bus.
    ESP_ERROR_CHECK(i2c_del_master_bus(bus));
}
