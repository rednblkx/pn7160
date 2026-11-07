// Adapted from rednblkx/HomeKey-ESP32; MIT, see LICENSE.HomeKey-ESP32.
#pragma once

#include "pn7160.hpp"
#include "portmacro.h"
#include "spi.hpp"

#include <Wire.h>
#include <array>
#include <memory>
#include <vector>

/**
 * @brief PN7160-family Arduino SPI/I2C reader with explicit hardware configuration.
 *
 * Wraps the NCI transport and PN7160_NCI engine, managing the background
 * task_runner task and event-driven tag discovery.
 */
class PN7160_ArduinoReader {
public:
    struct Config {
        TwoWire* wire = nullptr; // nullptr selects SPI.
        int sda = -1, scl = -1, irq = -1, ven = -1, dwl = -1;
        uint8_t address = 0x28;
        uint32_t clock = 400000;
        uint16_t vantMv = 0; // 0 = existing automatic PMU configuration.
        std::array<uint8_t,4> spiPins{}; // CS, SCLK, MISO, MOSI
    };
    explicit PN7160_ArduinoReader(const Config& config);
    // Copies a complete 18-byte ECP frame including CRC-A; nullptr/0 disables it.
    bool setEcpFrame(const uint8_t* frame, size_t length);
    ~PN7160_ArduinoReader();
		PN7160_ArduinoReader(const PN7160_ArduinoReader&) = delete;
		PN7160_ArduinoReader& operator=(const PN7160_ArduinoReader&) = delete;
		PN7160_ArduinoReader(PN7160_ArduinoReader&&) = delete;
		PN7160_ArduinoReader& operator=(PN7160_ArduinoReader&&) = delete;

    bool init();
    void stop();
    bool isConnected() const;

    uint8_t getFwMajor() const { return m_fwMajor; }
    uint8_t getFwMinor() const { return m_fwMinor; }

    bool beginDiscovery();
    bool pollForTag(std::vector<uint8_t>& uid,
                    std::array<uint8_t, 2>& atqa,
                    uint8_t& sak,
                    uint32_t timeoutMs);
    bool isTagStillPresent();
    void releaseTag();
    void endDiscovery();

    bool updateECP();
    bool exchangeApdu(const std::vector<uint8_t>& send,
                      std::vector<uint8_t>& recv,
                      uint32_t timeoutMs);
    bool healthCheck();

private:
    Config m_config;
    std::array<uint8_t,18> m_ecpData{};
    std::unique_ptr<IPN7160Transport> m_transport;
    std::unique_ptr<PN7160_NCI> m_nci;

    bool m_connected = false;
    uint32_t m_lastPresenceCheck = 0; // Tick count of last rf_iso_dep_presence_check()
    TickType_t m_lastHealthCheckTick = 0;
    uint8_t m_fwMajor = 0;
    uint8_t m_fwMinor = 0;
    uint8_t m_currentProtocol = 0;    // RF protocol of the active tag (nci::PROT_*)

    static constexpr uint32_t kPresenceCheckIntervalMs = 500;
    static constexpr uint32_t kHealthCheckIntervalMs = 10000;

    static constexpr const char* TAG = "PN7160_ArduinoReader";
};
