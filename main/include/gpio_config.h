#pragma once

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define MODE_BUTTON_PIN     GPIO_NUM_32
#define VOLUME_POT_PIN      ADC_CHANNEL_6     // GPIO34
#define FREQUENCY_POT_PIN   ADC_CHANNEL_7     // GPIO35

class GPIOConfig {
public:
    static GPIOConfig& getInstance() {
        static GPIOConfig instance;
        return instance;
    }

    void init();
    static void IRAM_ATTR buttonIsrHandler(void* arg);
    float readVolume();
    float readFrequency();

private:
    GPIOConfig() = default;
    adc_oneshot_unit_handle_t adc1Handle;
    adc_cali_handle_t adc1Cali;
}; 