#pragma once
#include "transport.hpp"
#include "nci/message.hpp"
#include <deque>
#include <cassert>
#include <optional>
#include <vector>

enum class NciEventType { RF_INTF_ACTIVATED, RF_DISCOVER, RF_DEACTIVATE, SHUTDOWN };
struct NciEvent { NciEventType type; NciMessage msg; };

// Scripted NCI engine; reader.cpp and the packet representation remain real.
class PN7160_NCI {
    IPN7160Transport& transport;
    bool initialized = false;
public:
    inline static PN7160_NCI* active = nullptr;
    inline static int live = 0, starts = 0, stops = 0, probes = 0, discoveries = 0;
    inline static esp_err_t initStatus = ESP_OK, startStatus = ESP_OK,
        eventStatus = ESP_ERR_TIMEOUT, probeStatus = ESP_OK, healthStatus = ESP_OK,
        configStatus = ESP_OK;
    inline static bool present = true, apduOk = true;
    inline static std::deque<NciEvent> events;
    inline static std::vector<std::vector<uint8_t>> configs;
    inline static size_t failConfigAt = 0;

    explicit PN7160_NCI(IPN7160Transport& t) : transport(t) { active = this; ++live; }
    ~PN7160_NCI() { assert(!initialized); active = nullptr; --live; }
    esp_err_t initialize() {
        if (initStatus != ESP_OK) return initStatus;
        auto result = transport.init();
        initialized = result == ESP_OK;
        return result;
    }
    esp_err_t start() { ++starts; return startStatus; }
    void stop() { ++stops; initialized = false; }
    bool is_initialized() const { return initialized; }
    uint16_t get_firmware_version() const { return 0x1234; }
    esp_err_t core_set_config(std::span<const uint8_t> data) {
        configs.emplace_back(data.begin(), data.end());
        if (configs.size() == failConfigAt) return ESP_FAIL;
        return configStatus;
    }
    esp_err_t core_get_config(std::span<const uint8_t>, NciMessage& response) {
        std::vector<uint8_t> payload(17, 0xFF);
        response.build(nci::PKT_MT_CTRL_RESPONSE, nci::CORE_GID, nci::CORE_GET_CONFIG_OID, payload);
        return ESP_OK;
    }
    esp_err_t rf_start_discovery(std::span<const uint8_t>) { ++discoveries; return ESP_OK; }
    esp_err_t rf_stop_discovery() { return ESP_OK; }
    esp_err_t rf_deactivate(uint8_t) { return ESP_OK; }
    esp_err_t get_event(NciEvent& event, uint32_t) {
        if (events.empty()) return eventStatus;
        event = events.front(); events.pop_front(); return ESP_OK;
    }
    esp_err_t rf_iso_dep_presence_check() { ++probes; return probeStatus; }
    bool tag_in_field() const { return present; }
    std::optional<std::vector<uint8_t>> send_apdu_sync(const std::vector<uint8_t>& data, uint32_t) {
        if (!apduOk) return std::nullopt;
        return data;
    }
    esp_err_t send_command_wait_response(const NciMessage&, NciMessage&, uint32_t) {
        return healthStatus;
    }
};
