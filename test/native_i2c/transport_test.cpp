#include "i2c.hpp"
#include "freertos/task.h"
#include "esp_idf_version.h"
#include <array>
#include <cassert>
#include <iostream>
#include <type_traits>

int main() {
    static_assert(!std::is_copy_constructible_v<PN7160_I2C>);
    static_assert(!std::is_move_constructible_v<PN7160_I2C>);
    FakeBus bus{1}; // A second device belongs to the caller, not the transport.
    PN7160_I2C_Config config;
    config.bus=&bus; config.irq=14; config.ven=13; config.dwl_req=19;
    PN7160_I2C transport(config);
    std::array<uint8_t,258> frame{};
    assert(transport.read(frame.data(),3)==ESP_ERR_INVALID_STATE);
    assert(transport.init()==ESP_OK && transport.init()==ESP_OK);
    assert(adds==1 && bus.devices==2 && levels[13]==0 && levels[19]==0);
    transport.set_ven(true);assert(levels[13]==1);
    levels[14]=1;
    assert(transport.wait_for_irq(true,0)==ESP_OK);
    incoming={{0x60,5,255},std::vector<uint8_t>(255,0xA5)};
    assert(transport.read(frame.data(),3)==ESP_OK && frame[2]==255);
    assert(transport.read(frame.data()+3,255)==ESP_OK && frame[257]==0xA5);
    incoming={{0x60,0,0}}; // Zero payload consumes only the header transaction.
    assert(transport.read(frame.data(),3)==ESP_OK && incoming.empty());
    assert(transport.read(nullptr,3)==ESP_ERR_INVALID_ARG);
    assert(transport.read(frame.data(),259)==ESP_ERR_INVALID_ARG);
    assert(transport.read(frame.data(),0)==ESP_ERR_INVALID_ARG);
    const uint8_t command[]={0x20,0,1,1};
    const auto before=writes;
    assert(transport.write(command,sizeof(command))==ESP_ERR_TIMEOUT && writes==before);
    levels[14]=0;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6,0,0)
    constexpr auto nack=ESP_ERR_INVALID_RESPONSE;
#else
    constexpr auto nack=ESP_ERR_INVALID_STATE;
#endif
    writeErrors={nack,ESP_OK};
    assert(transport.write(command,sizeof(command))==ESP_OK);
    assert(writes==before+2 && sent[sent.size()-1]==sent[sent.size()-2]);
    for(auto error:{ESP_ERR_TIMEOUT,ESP_FAIL,ESP_ERR_NO_MEM}) {
        writeErrors={error};const auto count=writes;
        assert(transport.write(command,sizeof(command))==error && writes==count+1);
    }
    writeErrors={nack,nack,nack};const auto count=writes;
    assert(transport.write(command,sizeof(command))==nack && writes==count+3);
    std::fill(frame.begin(),frame.end(),0x7F);
    assert(transport.write(frame.data(),frame.size())==ESP_OK && sent.back().size()==258);
    levels[14]=1;
    readErrors={nack,ESP_OK};incoming={{1,2,3}};
    assert(transport.read(frame.data(),3)==ESP_OK && frame[2]==3);
    for(auto error:{ESP_ERR_TIMEOUT,ESP_FAIL,ESP_ERR_NO_MEM}) {
        readErrors={error};const auto count=reads;
        assert(transport.read(frame.data(),3)==error && reads==count+1);
        assert(frame[0]==0 && frame[1]==0 && frame[2]==0);
    }
    readErrors={nack,nack,nack};const auto readCount=reads;
    assert(transport.read(frame.data(),3)==nack && reads==readCount+3);
    levels[14]=0;
    const auto start=ticks;
    onDelay=[&]{if(TickType_t(ticks-start)>=3)levels[14]=1;};
    assert(transport.wait_for_irq(true,5)==ESP_OK && ticks-start==3);
    onDelay={};levels[14]=0;ticks=UINT32_MAX-2;
    assert(transport.wait_for_irq(true,5)==ESP_ERR_TIMEOUT && ticks==2);
    transport.deinit();transport.deinit();
    assert(removes==1 && bus.devices==1 && configured==0);
    assert(transport.init()==ESP_OK);transport.deinit();assert(bus.devices==1);
    addError=ESP_ERR_NO_MEM;
    assert(transport.init()==ESP_ERR_NO_MEM && configured==0 && bus.devices==1);
    addError=ESP_OK;gpioError=ESP_FAIL;
    assert(transport.init()==ESP_FAIL && configured==0 && bus.devices==1);
    gpioError=ESP_OK;
    for(auto invalid:{-1,40}) {
        auto bad=config;bad.irq=invalid;PN7160_I2C other(bad);
        assert(other.init()==ESP_ERR_INVALID_ARG);
    }
    auto bad=config;bad.bus=nullptr;PN7160_I2C nullBus(bad);
    assert(nullBus.init()==ESP_ERR_INVALID_ARG);
    bad=config;bad.dwl_req=bad.ven;PN7160_I2C overlap(bad);
    assert(overlap.init()==ESP_ERR_INVALID_ARG);
    // Failed removal retains the handle for a later cleanup, never double-adds.
    assert(transport.init()==ESP_OK);removeError=ESP_FAIL;transport.deinit();
    assert(transport.init()==ESP_ERR_INVALID_STATE && bus.devices==2);
    removeError=ESP_OK;transport.deinit();assert(bus.devices==1);
    std::cout<<"Native PN7160 I2C lifecycle, framing, IRQ and error tests passed\n";
}
