#pragma once
#include "transport.hpp"
#include <cassert>
using gpio_num_t = int;
inline constexpr int SPI2_HOST = 2;
struct PN7160_SPI_Config { gpio_num_t miso, mosi, sclk, cs, irq, ven; };
class PN7160_SPI final : public IPN7160Transport {
public:
    inline static int constructed = 0;
    PN7160_SPI(int host, const PN7160_SPI_Config& pins, int clock) {
        assert(host == SPI2_HOST && pins.irq >= 0 && pins.ven >= 0 && clock == 4);
        ++constructed;
    }
    esp_err_t init() override { return ESP_OK; }
    void deinit() override {}
    esp_err_t read(uint8_t*, size_t) override { return ESP_OK; }
    esp_err_t write(const uint8_t*, size_t) override { return ESP_OK; }
    esp_err_t wait_for_irq(bool, TickType_t) override { return ESP_OK; }
    bool read_irq_level() const override { return false; }
    void set_ven(bool) override {}
    bool has_ven() const override { return true; }
};
