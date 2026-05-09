#pragma once

#include "transport.hpp"

#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "soc/gpio_num.h"

#include <cstdint>

// PN7160 SPI Transfer Direction Discriminator bytes (UM11495 Table 7)
static constexpr uint8_t PN7160_SPI_TDD_READ  = 0xFF;
static constexpr uint8_t PN7160_SPI_TDD_WRITE = 0x7F; // Chip only looks for MSB == 0, rest helps reduce line toggling
static constexpr uint8_t PN7160_SPI_MISO_ACK  = 0xFF; // Used to verify if the chip accepted a write

// Max application payload (255) + NCI header (3) + SPI TDD (1)
static constexpr size_t MAX_SPI_TRANSFER = 260;

/**
 * @brief Pin assignment for the SPI transport.
 *
 * MISO/MOSI/SCLK are passed to the SPI bus; the driver owns CS manually
 * so it can insert inter-byte delays mandated by UM11495.
 * Set optional pins to GPIO_NUM_NC to disable them.
 */
struct PN7160_SPI_Config {
    gpio_num_t miso;
    gpio_num_t mosi;
    gpio_num_t sclk;
    gpio_num_t cs;
    gpio_num_t irq;
    gpio_num_t ven;                         ///< VEN / enable
    gpio_num_t dwl_req  = GPIO_NUM_NC;      ///< Download-request (optional)
    gpio_num_t wkup_req = GPIO_NUM_NC;      ///< Wake-up request  (optional)
};

/**
 * @brief SPI-based transport for the PN7160 NCI driver.
 *
 * Handles all SPI bus transactions, manual CS assertion, GPIO configuration,
 * and rising-edge IRQ detection via a FreeRTOS binary semaphore.
 */
class PN7160_SPI final : public IPN7160Transport {
public:
    /**
     * @param host          SPI host (e.g. SPI2_HOST).
     * @param pins          GPIO pin assignments.
     * @param clock_mhz     SPI clock speed in MHz (default 4 MHz).
     */
    PN7160_SPI(spi_host_device_t host,
               const PN7160_SPI_Config& pins,
               int clock_mhz = 4);
    ~PN7160_SPI() override;

    [[nodiscard]] esp_err_t init()   override;
    void                    deinit() override;

    [[nodiscard]] esp_err_t read(uint8_t* buffer, size_t length) override;
    [[nodiscard]] esp_err_t write(const uint8_t* buffer, size_t length) override;

    [[nodiscard]] esp_err_t wait_for_irq(bool expected_level,
                                         TickType_t timeout_ticks) override;
    bool read_irq_level() const override;

    void set_ven(bool enable) override;
    bool has_ven() const override { return pins_.ven != GPIO_NUM_NC; }

private:
    // --- GPIO helpers ---
    [[nodiscard]] esp_err_t configure_gpio_output(gpio_num_t pin, bool initial_level,
                                                  bool pullup = false);
    void chip_select(bool assert); ///< assert=true drives CS low

    // --- SPI raw transfer ---
    [[nodiscard]] esp_err_t spi_transfer(spi_transaction_t* trans);

    // --- IRQ ISR ---
    [[nodiscard]] esp_err_t setup_irq();
    static void IRAM_ATTR isr_handler(void* arg);

    // --- State ---
    spi_host_device_t    host_;
    PN7160_SPI_Config    pins_;
    int                  clock_hz_;
    spi_device_handle_t  device_          = nullptr;
    SemaphoreHandle_t    irq_sem_         = nullptr;
    SemaphoreHandle_t    spi_mutex_       = nullptr;
    bool                 isr_installed_   = false;
    bool                 initialized_     = false;
    bool                 bus_initialized_ = false;

    // DMA-aligned transfer buffers (heap-allocated as class members to
    // avoid blowing the 4 KiB FreeRTOS task stack with 520-byte locals).
    alignas(4) uint8_t   tx_buf_[MAX_SPI_TRANSFER];
    alignas(4) uint8_t   rx_buf_[MAX_SPI_TRANSFER];

    static constexpr const char* TAG = "PN7160_SPI";
};