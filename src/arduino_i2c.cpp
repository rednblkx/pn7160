#if defined(ARDUINO_ARCH_ESP32)
#include "arduino_i2c.hpp"
#include <Arduino.h>
#include <esp32-hal-i2c.h>
#include <esp_idf_version.h>
#include <esp_log.h>
#include <freertos/task.h>
#include <cstring>

static constexpr const char* TAG = "PN7160_I2C";

static bool isNack(esp_err_t status)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    return status == ESP_ERR_INVALID_RESPONSE;
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
    return status == ESP_ERR_INVALID_STATE;
#else
    return status == ESP_FAIL;
#endif
}

esp_err_t PN7160_ArduinoI2C::init()
{
    if (ready) return ESP_OK;
    if (sda < 0 || scl < 0 || irq < 0 || ven < 0 || address < 8 || address > 0x77)
        return ESP_ERR_INVALID_ARG;
    if (dwl >= 0)
    {
        pinMode(dwl, OUTPUT);
        digitalWrite(dwl, LOW);
    }
    pinMode(ven, OUTPUT);
    digitalWrite(ven, LOW);
    pinMode(irq, INPUT);
    if (wire.setBufferSize(258) < 258) return ESP_ERR_NO_MEM;
    if (!wire.begin(sda, scl, clock)) return ESP_FAIL;
    wire.setTimeOut(20);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
    // Reuse Arduino's initialized bus. Its driver semaphore serializes these
    // synchronous reads and writes. ACK checking remains enabled.
    auto bus = static_cast<i2c_master_bus_handle_t>(i2cBusHandle(wire.getBusNum()));
    if (!bus) return ESP_ERR_INVALID_STATE;
    i2c_device_config_t config{};
    config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    config.device_address = address;
    config.scl_speed_hz = clock;
    const esp_err_t status = i2c_master_bus_add_device(bus, &config, &deviceHandle);
    if (status != ESP_OK) return status;
#endif
    ready = true;
    return ESP_OK;
}

void PN7160_ArduinoI2C::deinit()
{
    if (ready) digitalWrite(ven, LOW);
    ready = false;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
    if (deviceHandle)
    {
        const esp_err_t status = i2c_master_bus_rm_device(deviceHandle);
        if (status != ESP_OK) ESP_LOGE(TAG, "Cannot release I2C write device: 0x%X", status);
        deviceHandle = nullptr;
    }
#endif
}

esp_err_t PN7160_ArduinoI2C::read(uint8_t* buffer, size_t length)
{
    if (!ready) return ESP_ERR_INVALID_STATE;
    if (!buffer || length == 0 || length > 258) return ESP_ERR_INVALID_ARG;
    // A slave NACK on a receive occurs at the address phase: no NCI bytes
    // were consumed, so this same header/payload chunk can safely be retried.
    // Never retry a timeout or another error: a partial read could have advanced
    // the FIFO. Clear the entire output on failure to prevent stale packets.
    std::memset(buffer, 0, length);
    if (!i2cIsInit(wire.getBusNum())) return ESP_ERR_INVALID_STATE;
    if (!read_irq_level()) return ESP_ERR_TIMEOUT;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
    if (!deviceHandle) return ESP_ERR_INVALID_STATE;
    for (unsigned attempt = 0; attempt < 3; ++attempt)
    {
        const esp_err_t status = i2c_master_receive(deviceHandle, buffer, length, wire.getTimeOut());
        if (status == ESP_OK)
        {
            if (attempt) ESP_LOGD(TAG, "NCI read recovered after %u address NACK(s)", attempt);
            return ESP_OK;
        }
        std::memset(buffer, 0, length);
        if (!isNack(status))
        {
            ESP_LOGE(TAG, "NCI I2C read failed: 0x%X", status);
            return status;
        }
        // IRQ may have dropped between the runner's check and the transaction.
        // No pending packet is an idle poll, not a broken NCI connection.
        if (!read_irq_level()) return ESP_ERR_TIMEOUT;
        if (attempt == 2)
        {
            ESP_LOGE(TAG, "NCI I2C read failed after 3 address NACKs (IRQ still high)");
            return status;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
        if (!read_irq_level()) return ESP_ERR_TIMEOUT;
    }
    return ESP_FAIL;
#else
    if (wire.requestFrom(address, length) != length) return ESP_FAIL;
    for (size_t i = 0; i < length; ++i)
    {
        const int value = wire.read();
        if (value < 0) { std::memset(buffer, 0, length); return ESP_FAIL; }
        buffer[i] = static_cast<uint8_t>(value);
    }
    return ESP_OK;
#endif
}

esp_err_t PN7160_ArduinoI2C::write(const uint8_t* buffer, size_t length)
{
    if (!ready) return ESP_ERR_INVALID_STATE;
    if (!buffer || length == 0 || length > 258) return ESP_ERR_INVALID_ARG;
    // Arduino's i2cWrite logs every NACK as an error before we can retry it.
    // Use the synchronous IDF API on the same bus so only final failures are
    // reported. A recovered standby NACK is normal NCI flow control.
    if (!i2cIsInit(wire.getBusNum())) return ESP_ERR_INVALID_STATE;
    esp_err_t status = ESP_FAIL;
    for (unsigned attempt = 0; attempt < 3; ++attempt)
    {
        // Let the NCI receiver drain pending notifications before writing.
        if (wait_for_irq(false, pdMS_TO_TICKS(20)) != ESP_OK) return ESP_ERR_TIMEOUT;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
        if (!deviceHandle) return ESP_ERR_INVALID_STATE;
        status = i2c_master_transmit(deviceHandle, buffer, length, wire.getTimeOut());
#else
        status = i2cWrite(wire.getBusNum(), address, buffer, length, wire.getTimeOut());
#endif
        if (status == ESP_OK)
        {
            if (attempt) ESP_LOGD(TAG, "NCI write recovered after %u NACK(s)", attempt);
            return ESP_OK;
        }
        const bool nack = isNack(status);
        // UM11495 section 4.2.3 requires resending the complete frame after
        // any byte is NACKed (including standby wake-up). Never retry a timeout.
        if (!nack || attempt == 2)
        {
            ESP_LOGE(TAG, "NCI I2C write failed after %u attempt(s): 0x%X", attempt + 1, status);
            return status;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return status;
}

esp_err_t PN7160_ArduinoI2C::wait_for_irq(bool expected, TickType_t timeout)
{
    if (!ready) return ESP_ERR_INVALID_STATE;
    const TickType_t start = xTaskGetTickCount();
    while (read_irq_level() != expected)
    {
        if (timeout != portMAX_DELAY && xTaskGetTickCount() - start >= timeout)
            return ESP_ERR_TIMEOUT;
        vTaskDelay(1);
    }
    return ESP_OK;
}

bool PN7160_ArduinoI2C::read_irq_level() const
{
    return ready && digitalRead(irq) == HIGH;
}

void PN7160_ArduinoI2C::set_ven(bool enabled)
{
    if (ready)
    {
        digitalWrite(ven, enabled ? HIGH : LOW);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#endif
