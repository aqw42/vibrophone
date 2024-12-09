#include "gpio_config.h"
#include "audio_output.h"
#include "esp_log.h"
#include <math.h>

static const char* TAG = "GPIO";
static bool s_last_mode = false;

static void IRAM_ATTR button_isr_handler(void* arg) {
    bool newMode = !AudioOutput::getInstance().getBluetoothMode();
    AudioOutput::getInstance().setBluetoothMode(newMode);
    s_last_mode = newMode;  // Will be logged in next readVolume call
}

void GPIOConfig::init() {
    ESP_LOGI(TAG, "Initializing GPIO configuration");

    // Configure button with interrupt
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MODE_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure mode button: %s", esp_err_to_name(ret));
        return;
    }

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
        return;
    }

    ret = gpio_isr_handler_add(MODE_BUTTON_PIN, button_isr_handler, nullptr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler: %s", esp_err_to_name(ret));
        return;
    }

    // Configure ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ret = adc_oneshot_new_unit(&init_config, &adc1Handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return;
    }

    // Configure ADC channels without calibration first
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    
    ret = adc_oneshot_config_channel(adc1Handle, VOLUME_POT_PIN, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure volume ADC channel: %s", esp_err_to_name(ret));
        return;
    }

    ret = adc_oneshot_config_channel(adc1Handle, FREQUENCY_POT_PIN, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure frequency ADC channel: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "GPIO initialization complete");
}

float GPIOConfig::readVolume() {
    static uint32_t last_print = 0;
    static int last_raw = -1;
    
    int raw_value;
    esp_err_t ret = adc_oneshot_read(adc1Handle, VOLUME_POT_PIN, &raw_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read volume ADC: %s", esp_err_to_name(ret));
        return 0.5f;
    }

    // Print values every second if they changed
    uint32_t now = esp_log_timestamp();
    if (now - last_print > 1000 || raw_value != last_raw || s_last_mode != AudioOutput::getInstance().getBluetoothMode()) {
        ESP_LOGI(TAG, "Volume ADC: %d, Mode: %s", 
                raw_value, 
                AudioOutput::getInstance().getBluetoothMode() ? "BT" : "Sine");
        last_print = now;
        last_raw = raw_value;
        s_last_mode = AudioOutput::getInstance().getBluetoothMode();
    }

    return raw_value / 4095.0f;  // Direct conversion without calibration
}

float GPIOConfig::readFrequency() {
    static int last_raw = -1;
    
    int raw_value;
    esp_err_t ret = adc_oneshot_read(adc1Handle, FREQUENCY_POT_PIN, &raw_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read frequency ADC: %s", esp_err_to_name(ret));
        return 20.0f;
    }

    if (raw_value != last_raw) {
        ESP_LOGI(TAG, "Frequency ADC: %d", raw_value);
        last_raw = raw_value;
    }

    float normalized = raw_value / 4095.0f;
    if (AudioOutput::getInstance().getBluetoothMode()) {
        return 10.0f + normalized * 90.0f;
    } else {
        return 1.5f + powf(48.5f, normalized);
    }
} 