#pragma once
#include "freertos/FreeRTOS.h"
#include <functional>
inline TickType_t ticks=0;
inline std::function<void()> onDelay;
inline TickType_t xTaskGetTickCount() { return ticks; }
inline void vTaskDelay(TickType_t amount) { ticks+=amount; if(onDelay)onDelay(); }
