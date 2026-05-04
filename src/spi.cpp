#include "spi.hpp"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_log_buffer.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"

#include <cstring>

// Max application payload (255) + NCI header (3) + SPI TDD (1)
static constexpr size_t MAX_SPI_TRANSFER = 260;

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

PN7160_SPI::PN7160_SPI(spi_host_device_t host,
                                           const PN7160_SPI_PinConfig& pins,
                                           int clock_mhz)
    : host_(host),
      pins_(pins),
      clock_hz_(clock_mhz * 1000 * 1000) {
    irq_sem_ = xSemaphoreCreateBinary();
    if (!irq_sem_) {
        ESP_LOGE(TAG, "Failed to create IRQ semaphore");
    }
}

PN7160_SPI::~PN7160_SPI() {
    deinit();
}

// ---------------------------------------------------------------------------
// IPN7160Transport — Lifecycle
// ---------------------------------------------------------------------------

esp_err_t PN7160_SPI::init() {
    if (initialized_) return ESP_OK;

    if (!irq_sem_) return ESP_ERR_NO_MEM;

    // --- VEN (output, start low = chip in reset) ---
    if (pins_.ven != GPIO_NUM_NC) {
        ESP_RETURN_ON_ERROR(configure_gpio_output(pins_.ven, /*level=*/false),
                            TAG, "VEN pin config failed");
    } else {
        ESP_LOGW(TAG, "VEN pin not configured (GPIO_NUM_NC)");
    }

    // --- CS (output, start high = deselected, pull-up to keep stable) ---
    ESP_RETURN_ON_ERROR(configure_gpio_output(pins_.cs, /*level=*/true,
                                              /*pullup=*/true),
                        TAG, "CS pin config failed");

    // --- Optional: DWL_REQ ---
    if (pins_.dwl_req != GPIO_NUM_NC) {
        ESP_RETURN_ON_ERROR(configure_gpio_output(pins_.dwl_req, false),
                            TAG, "DWL_REQ pin config failed");
    }

    // --- Optional: WKUP_REQ ---
    if (pins_.wkup_req != GPIO_NUM_NC) {
        ESP_RETURN_ON_ERROR(configure_gpio_output(pins_.wkup_req, false),
                            TAG, "WKUP_REQ pin config failed");
    }

    // --- SPI device ---
    spi_device_interface_config_t devcfg = {};
    devcfg.command_bits   = 0;
    devcfg.address_bits   = 0;
    devcfg.dummy_bits     = 0;
    devcfg.mode           = 0;             // CPOL=0, CPHA=0
    devcfg.clock_speed_hz = clock_hz_;
    devcfg.spics_io_num   = -1;            // Manual CS
    devcfg.flags          = 0;
    devcfg.queue_size     = 3;
    ESP_RETURN_ON_ERROR(spi_bus_add_device(host_, &devcfg, &device_),
                        TAG, "Failed to add SPI device");
    ESP_LOGI(TAG, "SPI device added at %d MHz", clock_hz_ / 1'000'000);

    // --- IRQ ---
    ESP_RETURN_ON_ERROR(setup_irq(), TAG, "IRQ setup failed");

    initialized_ = true;
    ESP_LOGI(TAG, "SPI transport initialized");
    return ESP_OK;
}

void PN7160_SPI::deinit() {
    if (isr_installed_) {
        gpio_isr_handler_remove(pins_.irq);
        isr_installed_ = false;
        ESP_LOGI(TAG, "IRQ handler removed");
    }

    if (device_) {
        spi_bus_remove_device(device_);
        device_ = nullptr;
        ESP_LOGI(TAG, "SPI device removed");
    }

    // Drive VEN low to keep chip in reset
    if (pins_.ven != GPIO_NUM_NC) {
        gpio_set_level(pins_.ven, 0);
    }

    if (irq_sem_) {
        vSemaphoreDelete(irq_sem_);
        irq_sem_ = nullptr;
    }

    initialized_ = false;
}

// ---------------------------------------------------------------------------
// IPN7160Transport — Data Transfer
// ---------------------------------------------------------------------------

esp_err_t PN7160_SPI::read(uint8_t* buffer, size_t length) {
    if (!buffer || length == 0) return ESP_ERR_INVALID_ARG;
    if (length + 1 > MAX_SPI_TRANSFER) return ESP_ERR_INVALID_SIZE;

    // Full-duplex: transmit TDD_READ + dummy bytes while clocking in data.
    const size_t total = length + 1; // TDD byte + payload

    alignas(4) uint8_t tx[MAX_SPI_TRANSFER];
    alignas(4) uint8_t rx[MAX_SPI_TRANSFER];

    tx[0] = PN7160_SPI_TDD_READ;
    memset(tx + 1, 0xFF, length);

    spi_transaction_t trans = {};
    trans.length    = total * 8;
    trans.rxlength  = total * 8;
    trans.tx_buffer = tx;
    trans.rx_buffer = rx;

    ESP_RETURN_ON_ERROR(spi_transfer(&trans), TAG, "SPI read transfer failed");

    ESP_LOGD(TAG, "SPI read raw (%zu bytes):", total);
    ESP_LOG_BUFFER_HEXDUMP(TAG, rx, total, ESP_LOG_DEBUG);

    // Byte 0 received opposite TDD is discarded; bytes 1..N are data.
    memcpy(buffer, rx + 1, length);
    return ESP_OK;
}

esp_err_t PN7160_SPI::write(const uint8_t* buffer, size_t length) {
    if (!buffer || length == 0) return ESP_ERR_INVALID_ARG;
    if (length + 1 > MAX_SPI_TRANSFER) return ESP_ERR_INVALID_SIZE;

    alignas(4) uint8_t tx[MAX_SPI_TRANSFER];
    tx[0] = PN7160_SPI_TDD_WRITE;
    memcpy(tx + 1, buffer, length);

    spi_transaction_t trans = {
        .flags     = 0,
        .length    = (length + 1) * 8,
        .rxlength  = 0,
        .tx_buffer = tx,
        .rx_buffer = nullptr,
    };

    return spi_transfer(&trans);
}

// ---------------------------------------------------------------------------
// IPN7160Transport — IRQ
// ---------------------------------------------------------------------------

esp_err_t PN7160_SPI::wait_for_irq(bool expected_level,
                                              TickType_t timeout_ticks) {
    if (gpio_get_level(pins_.irq) == expected_level) return ESP_OK;

    if (!expected_level) {
        // The semaphore signals rising edge only; we cannot efficiently wait
        // for a falling edge here, so just report current state.
        ESP_LOGW(TAG, "wait_for_irq(LOW): pin is still HIGH");
        return ESP_ERR_INVALID_STATE;
    }

    // Drain any stale semaphore count before blocking.
    xSemaphoreTake(irq_sem_, 0);

    if (xSemaphoreTake(irq_sem_, timeout_ticks) == pdTRUE) {
        if (gpio_get_level(pins_.irq) == expected_level) return ESP_OK;
        ESP_LOGW(TAG, "IRQ semaphore received but pin is LOW?");
        return ESP_FAIL;
    }

    // One last level check — the edge may have arrived just as the timeout fired.
    if (gpio_get_level(pins_.irq) == expected_level) {
        ESP_LOGW(TAG, "IRQ went HIGH right at timeout boundary");
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

bool PN7160_SPI::read_irq_level() const {
    return gpio_get_level(pins_.irq) != 0;
}

// ---------------------------------------------------------------------------
// IPN7160Transport — Power
// ---------------------------------------------------------------------------

void PN7160_SPI::set_ven(bool enable) {
    if (pins_.ven != GPIO_NUM_NC) {
        gpio_set_level(pins_.ven, enable ? 1 : 0);
        ESP_LOGD(TAG, "VEN -> %d", enable ? 1 : 0);
    }
}

// ---------------------------------------------------------------------------
// Private — GPIO helpers
// ---------------------------------------------------------------------------

esp_err_t PN7160_SPI::configure_gpio_output(gpio_num_t pin,
                                                       bool initial_level,
                                                       bool pullup) {
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "gpio_config output failed");
    gpio_set_level(pin, initial_level ? 1 : 0);
    return ESP_OK;
}

esp_err_t PN7160_SPI::configure_gpio_input(gpio_num_t pin) {
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    return gpio_config(&cfg);
}

void PN7160_SPI::chip_select(bool assert) {
    // CS is active-low.
    gpio_set_level(pins_.cs, assert ? 0 : 1);
}

// ---------------------------------------------------------------------------
// Private — Raw SPI transfer
// ---------------------------------------------------------------------------

esp_err_t PN7160_SPI::spi_transfer(spi_transaction_t* trans) {
    esp_rom_delay_us(5); // Tcs_setup
    chip_select(true);
    esp_rom_delay_us(5); // Tcsh_setup

    esp_err_t ret = spi_device_polling_transmit(device_, trans);

    esp_rom_delay_us(5); // Tcsh_hold
    chip_select(false);
    esp_rom_delay_us(5); // Tcs_hold

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_device_polling_transmit failed: 0x%X", ret);
    }
    return ret;
}

// ---------------------------------------------------------------------------
// Private — IRQ setup & ISR
// ---------------------------------------------------------------------------

esp_err_t PN7160_SPI::setup_irq() {
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pins_.irq,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE, // IRQ is active-high
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "IRQ pin config failed");

    esp_err_t ret = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "GPIO ISR service already installed");
    } else {
        ESP_RETURN_ON_ERROR(ret, TAG, "Failed to install GPIO ISR service");
    }

    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_add(pins_.irq, isr_handler, this),
        TAG, "Failed to add ISR handler");

    isr_installed_ = true;
    ESP_LOGI(TAG, "IRQ configured on GPIO %d", pins_.irq);
    return ESP_OK;
}

void IRAM_ATTR PN7160_SPI::isr_handler(void* arg) {
    auto* self = static_cast<PN7160_SPI*>(arg);
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(self->irq_sem_, &woken);
    if (woken == pdTRUE) portYIELD_FROM_ISR();
}
