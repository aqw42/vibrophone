#include "bluetooth_handler.h"
#include "audio_output.h"
#include "esp_log.h"
#include "esp_check.h"

static const char* TAG = "BT_A2DP";
static uint8_t s_reconnect_attempts = 0;
static const uint8_t MAX_RECONNECT_ATTEMPTS = 3;

void BluetoothHandler::init() {
    ESP_LOGI(TAG, "Initializing Bluetooth A2DP sink...");

    // Initialize NVS
    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to release BLE memory: %s", esp_err_to_name(ret));
        return;
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize controller: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable controller: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Bluetooth controller initialized");

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bluedroid: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable bluedroid: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Bluedroid initialized and enabled");

    // Set device name
    ret = esp_bt_dev_set_device_name("ESP32 Bass Speaker");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set device name: %s", esp_err_to_name(ret));
        return;
    }

    // Register callbacks
    ret = esp_bt_gap_register_callback(gapCallback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GAP callback: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_a2d_register_callback(a2dpCallback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register A2DP callback: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_a2d_sink_register_data_callback(audioDataCallback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register A2DP data callback: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_a2d_sink_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize A2DP sink: %s", esp_err_to_name(ret));
        return;
    }

    // Set discoverable and connectable mode
    ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set scan mode: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "A2DP sink initialized successfully, ready for connections");
}

void BluetoothHandler::audioDataCallback(const uint8_t *data, uint32_t len) {
    static uint32_t total_bytes = 0;
    total_bytes += len;
    
    if (!data || len == 0) {
        ESP_LOGW(TAG, "Received invalid audio data");
        return;
    }

    // Log audio data stats periodically (every ~1 second)
    static uint32_t last_log = 0;
    uint32_t now = esp_log_timestamp();
    if (now - last_log > 1000) {
        ESP_LOGD(TAG, "Audio throughput: %lu bytes/sec", total_bytes);
        total_bytes = 0;
        last_log = now;
    }

    if (AudioOutput::getInstance().getBluetoothMode()) {
        BaseType_t result = xQueueSend(AudioOutput::getInstance().getAudioQueue(), data, pdMS_TO_TICKS(10));
        if (result != pdTRUE) {
            ESP_LOGW(TAG, "Audio queue full, dropping data");
        }
    }
}

void BluetoothHandler::gapCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT: {
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Authentication success with device: %s", param->auth_cmpl.device_name);
                s_reconnect_attempts = 0;
            } else {
                ESP_LOGE(TAG, "Authentication failed, status: %d", param->auth_cmpl.stat);
                if (++s_reconnect_attempts < MAX_RECONNECT_ATTEMPTS) {
                    ESP_LOGI(TAG, "Attempting to reconnect (%d/%d)", s_reconnect_attempts, MAX_RECONNECT_ATTEMPTS);
                    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                } else {
                    ESP_LOGE(TAG, "Max reconnection attempts reached");
                }
            }
            break;
        }
        case ESP_BT_GAP_CFM_REQ_EVT: {
            ESP_LOGI(TAG, "Please confirm the numeric value: %lu", param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;
        }
        case ESP_BT_GAP_MODE_CHG_EVT: {
            ESP_LOGD(TAG, "Power mode changed to: %d", param->mode_chg.mode);
            break;
        }
        default: {
            ESP_LOGV(TAG, "Unhandled GAP event: %d", event);
            break;
        }
    }
}

void BluetoothHandler::a2dpCallback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    switch (event) {
        case ESP_A2D_CONNECTION_STATE_EVT: {
            ESP_LOGI(TAG, "A2DP connection state: %d", param->conn_stat.state);
            if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
                ESP_LOGI(TAG, "A2DP connected to device");
                s_reconnect_attempts = 0;
            } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
                ESP_LOGI(TAG, "A2DP disconnected from device");
                if (++s_reconnect_attempts < MAX_RECONNECT_ATTEMPTS) {
                    ESP_LOGI(TAG, "Ready for new connections (%d/%d attempts)", s_reconnect_attempts, MAX_RECONNECT_ATTEMPTS);
                    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                }
            }
            break;
        }
        case ESP_A2D_AUDIO_STATE_EVT: {
            const char* state_str;
            switch (param->audio_stat.state) {
                case ESP_A2D_AUDIO_STATE_STARTED: state_str = "STARTED"; break;
                case ESP_A2D_AUDIO_STATE_STOPPED: state_str = "STOPPED"; break;
                default: state_str = "UNKNOWN"; break;
            }
            ESP_LOGI(TAG, "Audio streaming state: %s", state_str);
            break;
        }
        case ESP_A2D_AUDIO_CFG_EVT: {
            ESP_LOGI(TAG, "Audio configuration received - Sample rate: %d Hz", param->audio_cfg.mcc.cie.sbc[0] & 0xF0);
            break;
        }
        default: {
            ESP_LOGV(TAG, "Unhandled A2DP event: %d", event);
            break;
        }
    }
} 