#include "esp_idf_version.h"
#include "i2c.hpp"
#include "freertos/task.h"
#include "esp_log.h"
#include <cstring>
#include <climits>
#include <initializer_list>

namespace {
constexpr const char* TAG = "PN7160_I2C";
constexpr size_t max_frame = 3 + 255;
TickType_t delay_ticks(uint32_t ms) {
    const auto ticks = pdMS_TO_TICKS(ms);
    return ticks ? ticks : 1;
}
bool nack(esp_err_t error) {
    // These versions report an ACK error with these codes. Other errors,
    // especially timeouts, may mean a partially accepted frame: never replay.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    return error == ESP_ERR_INVALID_RESPONSE;
#else
    return error == ESP_ERR_INVALID_STATE;
#endif
}
}

esp_err_t PN7160_I2C::init() {
    if (ready_) return ESP_OK;
    // A failed device removal must be resolved before acquiring more resources.
    if (device_) return ESP_ERR_INVALID_STATE;
    const auto& c = config_;
    if (!c.bus || c.address < 8 || c.address > 0x77 || !c.clock_hz ||
        c.clock_hz > 400000 || !c.io_timeout_ms || c.io_timeout_ms > INT_MAX ||
        !GPIO_IS_VALID_GPIO(c.irq) ||
        (has_ven() && !GPIO_IS_VALID_OUTPUT_GPIO(c.ven)) ||
        (c.dwl_req != GPIO_NUM_NC && !GPIO_IS_VALID_OUTPUT_GPIO(c.dwl_req)) ||
        c.irq == c.ven || c.irq == c.dwl_req ||
        (has_ven() && c.ven == c.dwl_req)) return ESP_ERR_INVALID_ARG;

    // Configure only exclusive control pins, never SDA/SCL. No ISR is needed:
    // level polling also handles IRQ already high before wait_for_irq().
    for (auto pin : {c.irq, c.ven, c.dwl_req}) {
        if (pin == GPIO_NUM_NC) continue;
        const uint64_t mask = uint64_t{1} << pin;
        acquired_pins_ |= mask;
        esp_err_t error = ESP_OK;
        if (pin != c.irq) error = gpio_set_level(pin, 0);
        gpio_config_t gpio{};
        gpio.pin_bit_mask = mask;
        gpio.mode = pin == c.irq ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT;
        gpio.intr_type = GPIO_INTR_DISABLE;
        if (error == ESP_OK) error = gpio_config(&gpio);
        if (error != ESP_OK) { deinit(); return error; }
    }
    i2c_device_config_t device{};
    device.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device.device_address = c.address;
    device.scl_speed_hz = c.clock_hz;
    const esp_err_t error = i2c_master_bus_add_device(c.bus, &device, &device_);
    if (error != ESP_OK) { deinit(); return error; }
    ready_ = true;
    return ESP_OK;
}

void PN7160_I2C::deinit() {
    if (has_ven() && (acquired_pins_ & (uint64_t{1} << config_.ven)))
        gpio_set_level(config_.ven, 0);
    ready_ = false;
    if (device_) {
        const esp_err_t error = i2c_master_bus_rm_device(device_);
        if (error == ESP_OK) device_ = nullptr;
        else ESP_LOGE(TAG, "Device removal failed: 0x%X", error);
    }
    for (unsigned pin = 0; pin < GPIO_NUM_MAX; ++pin)
        if (acquired_pins_ & (uint64_t{1} << pin)) gpio_reset_pin(static_cast<gpio_num_t>(pin));
    acquired_pins_ = 0;
}

esp_err_t PN7160_I2C::read(uint8_t* buffer, size_t length) {
    if (!buffer || !length || length > max_frame) return ESP_ERR_INVALID_ARG;
    std::memset(buffer, 0, length);
    if (!ready_) return ESP_ERR_INVALID_STATE;
    for (unsigned attempt = 0; attempt < 3; ++attempt) {
        if (!read_irq_level()) return ESP_ERR_TIMEOUT;
        // Preserve separate NCI header and payload transactions; do not add a
        // register byte, SPI TDD prefix, or silently merge reads.
        const esp_err_t error = i2c_master_receive(device_, buffer, length, config_.io_timeout_ms);
        if (error == ESP_OK) return ESP_OK;
        std::memset(buffer, 0, length);
        // A receive NACK is at the address phase, so no FIFO bytes were read.
        if (!nack(error) || attempt == 2) return error;
        vTaskDelay(delay_ticks(5));
    }
    return ESP_FAIL;
}

esp_err_t PN7160_I2C::write(const uint8_t* buffer, size_t length) {
    if (!buffer || !length || length > max_frame) return ESP_ERR_INVALID_ARG;
    if (!ready_) return ESP_ERR_INVALID_STATE;
    for (unsigned attempt = 0; attempt < 3; ++attempt) {
        // Do not hold a private mutex while the receiver drains pending NCI.
        const esp_err_t idle = wait_for_irq(false, delay_ticks(20));
        if (idle != ESP_OK) return idle;
        const esp_err_t error = i2c_master_transmit(device_, buffer, length, config_.io_timeout_ms);
        if (error == ESP_OK || !nack(error) || attempt == 2) return error;
        // PN716x NCI I2C (UM11495 4.2.3) requires replay of the complete
        // frame on NACK, including standby wake-up. Never replay a timeout.
        vTaskDelay(delay_ticks(5));
    }
    return ESP_FAIL;
}

esp_err_t PN7160_I2C::wait_for_irq(bool expected, TickType_t timeout) {
    if (!ready_) return ESP_ERR_INVALID_STATE;
    const TickType_t start = xTaskGetTickCount();
    while (read_irq_level() != expected) {
        if (timeout != portMAX_DELAY && TickType_t(xTaskGetTickCount() - start) >= timeout)
            return ESP_ERR_TIMEOUT;
        vTaskDelay(1);
    }
    return ESP_OK;
}

bool PN7160_I2C::read_irq_level() const {
    return ready_ && gpio_get_level(config_.irq) != 0;
}
void PN7160_I2C::set_ven(bool enabled) {
    if (ready_ && has_ven()) {
        gpio_set_level(config_.ven, enabled ? 1 : 0);
        // Preserve the existing SPI transport's VEN settling interval.
        vTaskDelay(delay_ticks(10));
    }
}
