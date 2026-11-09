#pragma once

#include "transport.hpp"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

struct PN7160_I2C_Config {
    i2c_master_bus_handle_t bus = nullptr; // Borrowed; owner must outlive transport.
    uint16_t address = 0x28;              // Unshifted 7-bit address.
    uint32_t clock_hz = 400000;
    gpio_num_t irq = GPIO_NUM_NC;
    gpio_num_t ven = GPIO_NUM_NC;
    gpio_num_t dwl_req = GPIO_NUM_NC;
    uint32_t io_timeout_ms = 20;
};

// ESP-IDF 5.4+. No bus allocation, pin remapping, or global ISR service changes.
// Lifecycle calls require exclusive access: stop the NCI worker before deinit.
// The owner must serialize bus/device lifecycle changes by other bus users.
// Synchronous transfers use the IDF driver's bus operation semaphore.
class PN7160_I2C final : public IPN7160Transport {
public:
    explicit PN7160_I2C(const PN7160_I2C_Config& config) : config_(config) {}
    ~PN7160_I2C() override { deinit(); }
    PN7160_I2C(const PN7160_I2C&) = delete;
    PN7160_I2C& operator=(const PN7160_I2C&) = delete;
    PN7160_I2C(PN7160_I2C&&) = delete;
    PN7160_I2C& operator=(PN7160_I2C&&) = delete;

    esp_err_t init() override;
    void deinit() override;
    esp_err_t read(uint8_t* buffer, size_t length) override;
    esp_err_t write(const uint8_t* buffer, size_t length) override;
    esp_err_t wait_for_irq(bool expected, TickType_t timeout) override;
    bool read_irq_level() const override;
    void set_ven(bool enabled) override;
    bool has_ven() const override { return config_.ven != GPIO_NUM_NC; }

private:
    PN7160_I2C_Config config_;
    i2c_master_dev_handle_t device_ = nullptr;
    uint64_t acquired_pins_ = 0;
    bool ready_ = false;
};
