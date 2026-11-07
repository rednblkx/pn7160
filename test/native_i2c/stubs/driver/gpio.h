#pragma once
#include "esp_err.h"
#include <cstdint>
using gpio_num_t=int;
constexpr int GPIO_NUM_NC=-1, GPIO_NUM_MAX=40, GPIO_MODE_INPUT=1,
    GPIO_MODE_OUTPUT=2, GPIO_INTR_DISABLE=0;
#define GPIO_IS_VALID_GPIO(pin) ((pin)>=0 && (pin)<40)
#define GPIO_IS_VALID_OUTPUT_GPIO(pin) ((pin)>=0 && (pin)<34)
struct gpio_config_t { uint64_t pin_bit_mask=0; int mode=0, intr_type=0; };
inline int levels[40]={};
inline uint64_t configured=0;
inline esp_err_t gpioError=ESP_OK;
inline esp_err_t gpio_set_level(gpio_num_t pin,int level) { levels[pin]=level; return ESP_OK; }
inline int gpio_get_level(gpio_num_t pin) { return levels[pin]; }
inline esp_err_t gpio_config(const gpio_config_t* config) {
    if(gpioError!=ESP_OK)return gpioError;
    configured|=config->pin_bit_mask; return ESP_OK;
}
inline esp_err_t gpio_reset_pin(gpio_num_t pin) { configured&=~(uint64_t{1}<<pin);levels[pin]=0;return ESP_OK; }
