#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"

#include <cstddef>
#include <cstdint>

/**
 * @brief Abstract transport interface for PN7160 physical communication.
 *
 * Implementations provide the hardware-specific layer (SPI, I2C)
 * while the NCI protocol driver operates purely against this interface.
 *
 * All methods must be safe to call from a FreeRTOS task context.
 * IRQ-related methods must be ISR-safe where noted.
 */
class IPN7160Transport {
public:
    virtual ~IPN7160Transport() = default;

    // -------------------------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------------------------

    /**
     * @brief Initialize the transport: configure GPIOs, bus, ISR, etc.
     *        Called once before any read/write operations.
     * @return ESP_OK on success.
     */
    virtual esp_err_t init() = 0;

    /**
     * @brief Release all hardware resources acquired during init().
     *        Safe to call even if init() was never called or partially failed.
     */
    virtual void deinit() = 0;

    // -------------------------------------------------------------------------
    // Data Transfer
    // -------------------------------------------------------------------------

    /**
     * @brief Read @p length bytes from the device into @p buffer.
     *        The transport is responsible for any framing (e.g. TDD byte on SPI).
     * @param buffer  Destination buffer (must be at least @p length bytes).
     * @param length  Number of application-layer bytes to read.
     * @return ESP_OK on success.
     */
    virtual esp_err_t read(uint8_t* buffer, size_t length) = 0;

    /**
     * @brief Write @p length bytes from @p buffer to the device.
     *        The transport is responsible for any framing (e.g. TDD byte on SPI).
     * @param buffer  Source buffer.
     * @param length  Number of application-layer bytes to write.
     * @return ESP_OK on success.
     */
    virtual esp_err_t write(const uint8_t* buffer, size_t length) = 0;

    // -------------------------------------------------------------------------
    // IRQ / Data-Ready Signaling
    // -------------------------------------------------------------------------

    /**
     * @brief Block until the IRQ/data-ready line reaches @p expected_level,
     *        or until @p timeout_ticks elapses.
     *
     * @param expected_level  true = active/asserted, false = deasserted.
     * @param timeout_ticks   FreeRTOS tick count to wait (portMAX_DELAY = forever).
     * @return ESP_OK          Line is at the expected level.
     *         ESP_ERR_TIMEOUT Timeout elapsed before the line changed.
     *         other           Transport-specific error.
     */
    virtual esp_err_t wait_for_irq(bool expected_level,
                                   TickType_t timeout_ticks) = 0;

    /**
     * @brief Sample the IRQ/data-ready line without blocking.
     * @return true if the line is currently active/asserted.
     */
    virtual bool read_irq_level() const = 0;

    // -------------------------------------------------------------------------
    // Power / Enable Control
    // -------------------------------------------------------------------------

    /**
     * @brief Drive the VEN (enable) line.
     *        A low-to-high-to-low cycle hard-resets the PN7160.
     * @param enable  true = drive high (chip enabled), false = drive low (reset).
     */
    virtual void set_ven(bool enable) = 0;

    /**
     * @brief Returns true if a VEN pin is wired and controllable.
     *        When false, power-cycle resets are unavailable.
     */
    virtual bool has_ven() const = 0;
};
