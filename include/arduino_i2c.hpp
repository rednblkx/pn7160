#pragma once

#include <Wire.h>
#include <esp_idf_version.h>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
#include <driver/i2c_master.h>
#endif
#include "transport.hpp"

// PN7160-family NCI I2C framing (UM11495, section 4.2). Wire owns the shared bus;
// this transport owns VEN/DWL and a device handle, never the Wire bus.
// The NCI worker must be stopped before deinit().
class PN7160_ArduinoI2C final : public IPN7160Transport
{
  public:
    PN7160_ArduinoI2C(TwoWire& wire, int sda, int scl, int irq, int ven,
                      int dwl, uint8_t address, uint32_t clock)
        : wire(wire), sda(sda), scl(scl), irq(irq), ven(ven), dwl(dwl),
          address(address), clock(clock) {}
    PN7160_ArduinoI2C(const PN7160_ArduinoI2C&) = delete;
    PN7160_ArduinoI2C& operator=(const PN7160_ArduinoI2C&) = delete;
    PN7160_ArduinoI2C(PN7160_ArduinoI2C&&) = delete;
    PN7160_ArduinoI2C& operator=(PN7160_ArduinoI2C&&) = delete;
    ~PN7160_ArduinoI2C() override { deinit(); }
    esp_err_t init() override;
    void deinit() override;
    esp_err_t read(uint8_t* buffer, size_t length) override;
    esp_err_t write(const uint8_t* buffer, size_t length) override;
    esp_err_t wait_for_irq(bool expected, TickType_t timeout) override;
    bool read_irq_level() const override;
    void set_ven(bool enabled) override;
    bool has_ven() const override { return ven >= 0; }

  private:
    TwoWire& wire;
    int sda, scl, irq, ven, dwl;
    uint8_t address;
    uint32_t clock;
    bool ready = false;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
    i2c_master_dev_handle_t deviceHandle = nullptr;
#endif
};
