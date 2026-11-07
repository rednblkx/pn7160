#include "reader.hpp"
#include <freertos/task.h>
#include <cassert>
#include <iostream>

NciEvent activation(size_t length, uint8_t protocol = nci::PROT_ISODEP) {
    std::vector<uint8_t> tech = {0x44, 0x00, static_cast<uint8_t>(length)};
    for (size_t i = 0; i < length; ++i) tech.push_back(static_cast<uint8_t>(i + 1));
    tech.insert(tech.end(), {1, 0x20, 0});
    std::vector<uint8_t> payload = {1, nci::INTF_ISODEP, protocol, 0, 0xFF, 1,
                                    static_cast<uint8_t>(tech.size())};
    payload.insert(payload.end(), tech.begin(), tech.end());
    return {NciEventType::RF_INTF_ACTIVATED,
            NciMessage(nci::PKT_MT_CTRL_NOTIFICATION, 1, 5, payload)};
}

int main() {
    using Engine = PN7160_NCI;
    PN7160_ArduinoReader::Config config;
    config.irq = 14; config.ven = 13; config.vantMv = 3300;
    config.spiPins = {5, 18, 19, 23};
    PN7160_ArduinoReader reader(config);
    std::vector<uint8_t> uid{99}, response;
    std::array<uint8_t, 2> atqa{}; uint8_t sak = 0;
    assert(!reader.isConnected() && !reader.healthCheck());
    assert(!reader.pollForTag(uid, atqa, sak, 1) && uid.empty());
    assert(reader.init() && reader.isConnected());
    assert(PN7160_SPI::constructed == 1 && Engine::live == 1);
    assert(Engine::configs.front()[4] == 0); // ECP disabled by default.
    assert(reader.getFwMajor() == 0x12 && reader.getFwMinor() == 4);
    assert(reader.beginDiscovery());
    for (size_t length : {4, 7, 10}) {
        Engine::events.push_back(activation(length));
        assert(reader.pollForTag(uid, atqa, sak, 10));
        assert(uid.size() == length && uid.front() == 1 && uid.back() == length);
        assert(atqa[0] == 0x44 && atqa[1] == 0 && sak == 0x20);
        // Presentation suppression is caller-owned: report the same activation again.
        Engine::events.push_back(activation(length));
        assert(reader.pollForTag(uid, atqa, sak, 10));
    }
    assert(!reader.pollForTag(uid, atqa, sak, 10) && uid.empty() && reader.isConnected());
    const std::vector<uint8_t> apdu{0x00, 0xA4, 0x04, 0x00};
    assert(reader.exchangeApdu(apdu, response, 100) && response == apdu);
    Engine::apduOk = false;
    assert(!reader.exchangeApdu(apdu, response, 100) && response.empty());
    Engine::apduOk = true;
    Engine::events.push_back(activation(4, nci::PROT_T2T));
    assert(reader.pollForTag(uid, atqa, sak, 10));
    response = {99};
    assert(!reader.exchangeApdu(apdu, response, 100) && response.empty());
    Engine::events.push_back(activation(4));
    assert(reader.pollForTag(uid, atqa, sak, 10));

    ticks = 500; assert(reader.isTagStillPresent() && Engine::probes == 1);
    Engine::present = false; ticks = 501; assert(!reader.isTagStillPresent());
    Engine::present = true; Engine::probeStatus = ESP_FAIL; ticks = 1000;
    assert(!reader.isTagStillPresent() && !reader.isConnected());
    assert(!reader.healthCheck()); // Transport faults stay latched until reinitialization.
    response = {99};
    assert(!reader.exchangeApdu(apdu, response, 100) && response.empty());
    assert(!reader.beginDiscovery());
    Engine::probeStatus = ESP_OK;
    assert(reader.init() && reader.isConnected());
    Engine::eventStatus = ESP_FAIL;
    assert(!reader.pollForTag(uid, atqa, sak, 10) && !reader.isConnected());
    Engine::eventStatus = ESP_ERR_TIMEOUT;
    assert(reader.init());

    std::array<uint8_t, 18> ecp{}; ecp[0] = 0x6A; ecp[1] = 0xA5;
    assert(!reader.setEcpFrame(nullptr, 18));
    assert(!reader.setEcpFrame(ecp.data(), 17));
    assert(reader.setEcpFrame(ecp.data(), ecp.size()));
    ecp[1] = 0; // Setter must retain its own copy.
    assert(reader.updateECP());
    assert(Engine::configs.back()[4] == 16 && Engine::configs.back()[6] == 0xA5);
    assert(reader.setEcpFrame(nullptr, 0) && reader.updateECP());
    assert(Engine::configs.back()[4] == 0);
    ticks += 10000; Engine::healthStatus = ESP_FAIL;
    assert(!reader.healthCheck() && !reader.isConnected());
    Engine::healthStatus = ESP_OK;
    reader.stop(); reader.stop();
    assert(!reader.isConnected() && Engine::live == 0);

    Engine::initStatus = ESP_FAIL; assert(!reader.init() && Engine::live == 0);
    Engine::initStatus = ESP_OK;
    Engine::startStatus = ESP_FAIL; assert(!reader.init() && Engine::live == 0);
    Engine::startStatus = ESP_OK;
    Engine::configStatus = ESP_FAIL; assert(!reader.init() && Engine::live == 0);
    Engine::configStatus = ESP_OK;
    Engine::failConfigAt = Engine::configs.size() + 3; // Fail TOTAL_DURATION after ECP/PMU.
    assert(!reader.init() && Engine::live == 0);
    Engine::failConfigAt = 0;

    config.vantMv = 5000;
    PN7160_ArduinoReader fiveVolt(config);
    assert(fiveVolt.init());
    const auto& pmu = Engine::configs[Engine::configs.size() - 2];
    assert(pmu[1] == 0xA0 && pmu[2] == 0x0E && pmu[6] == 1 && pmu[7] == 1);
    fiveVolt.stop();
    config.vantMv = 3300;

    TwoWire wire;
    config.wire = &wire; config.sda = 21; config.scl = 22; config.dwl = 19;
    PN7160_ArduinoReader i2c(config);
    assert(i2c.init() && wire.liveDevices == 1);
    i2c.stop();
    assert(wire.liveDevices == 0 && wire.busInitialized && Engine::live == 0);
    std::cout << "Reader SPI/I2C lifecycle, UID, APDU, ECP and fault tests passed\n";
}
