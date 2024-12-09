#pragma once

#include <stdint.h>
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

class BluetoothHandler {
public:
    static BluetoothHandler& getInstance() {
        static BluetoothHandler instance;
        return instance;
    }

    void init();
    static void audioDataCallback(const uint8_t *data, uint32_t len);
    static void gapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

private:
    BluetoothHandler() = default;
    static void a2dpCallback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
}; 