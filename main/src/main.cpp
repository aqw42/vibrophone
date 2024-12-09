#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "bluetooth_handler.h"
#include "audio_output.h"
#include "gpio_config.h"

static const char* TAG = "MAIN";

extern "C" void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Initializing ESP32 Bass Speaker...");

    // Initialize components
    GPIOConfig::getInstance().init();
    AudioOutput::getInstance().init();
    BluetoothHandler::getInstance().init();

    // Start audio processing task
    AudioOutput::getInstance().startAudioTask();

    ESP_LOGI(TAG, "Initialization complete");
} 