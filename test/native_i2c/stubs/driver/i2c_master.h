#pragma once
#include "esp_err.h"
#include <cstdint>
#include <cstddef>
#include <deque>
#include <vector>
#include <algorithm>
#include <cassert>
constexpr int I2C_ADDR_BIT_LEN_7=7;
struct FakeBus { unsigned devices=0; };
struct FakeDevice { FakeBus* bus; uint16_t address; };
using i2c_master_bus_handle_t=FakeBus*;
using i2c_master_dev_handle_t=FakeDevice*;
struct i2c_device_config_t { int dev_addr_length; uint16_t device_address; uint32_t scl_speed_hz; };
inline unsigned adds=0, removes=0, reads=0, writes=0;
inline esp_err_t addError=ESP_OK, removeError=ESP_OK;
inline std::deque<esp_err_t> readErrors, writeErrors;
inline std::deque<std::vector<uint8_t>> incoming;
inline std::vector<std::vector<uint8_t>> sent;
inline esp_err_t nextError(std::deque<esp_err_t>& errors) {
    if(errors.empty())return ESP_OK;
    const auto error=errors.front();errors.pop_front();return error;
}
inline esp_err_t i2c_master_bus_add_device(FakeBus* bus, const i2c_device_config_t* config, FakeDevice** device) {
    assert(config->dev_addr_length==7 && config->device_address==0x28);
    if(addError!=ESP_OK)return addError;
    ++adds;++bus->devices;*device=new FakeDevice{bus,config->device_address};return ESP_OK;
}
inline esp_err_t i2c_master_bus_rm_device(FakeDevice* device) {
    if(removeError!=ESP_OK)return removeError;
    ++removes;--device->bus->devices;delete device;return ESP_OK;
}
inline esp_err_t i2c_master_receive(FakeDevice*, uint8_t* data,size_t size,int timeout) {
    assert(timeout>0);++reads;std::fill_n(data,size,0xCC);
    const auto error=nextError(readErrors);if(error!=ESP_OK)return error;
    assert(!incoming.empty() && incoming.front().size()==size);
    std::copy(incoming.front().begin(),incoming.front().end(),data);incoming.pop_front();return ESP_OK;
}
inline esp_err_t i2c_master_transmit(FakeDevice*,const uint8_t* data,size_t size,int timeout) {
    assert(timeout>0);++writes;sent.emplace_back(data,data+size);return nextError(writeErrors);
}
// Deliberately no bus creation/deletion API: the transport must not own it.
