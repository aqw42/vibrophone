#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"  // Add this for NVS
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/dac_oneshot.h"
#include <math.h>
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"  // For esp_rom_delay_us

static const char *TAG = "BT_SPEAKER";
static char bda_str[18];
static const char *SPP_TAG = "SPP_TAG";

// Define ADC channels for potentiometers
#define VOLUME_POT_CHANNEL     ADC_CHANNEL_6  // GPIO34
#define CUTOFF_POT_CHANNEL     ADC_CHANNEL_7  // GPIO35

// Define DAC channels
#define AUDIO_OUTPUT_CHANNEL   DAC_CHAN_0   // GPIO25
#define AUDIO_OUTPUT_CHANNEL_RAW   DAC_CHAN_1   // GPIO26 for unfiltered audio
#define SAMPLE_RATE           44100
#define BUF_SIZE             (2048 * 32)  // Increase to 64KB
#define SAMPLES_PER_YIELD    512          // Process more samples per batch
#define DAC_BUFFER_SIZE     1024          // Separate DAC buffer size

// Add these near the top with other defines
#define BUTTON_PIN          GPIO_NUM_4    // Changed from GPIO0 to GPIO4
#define PI                 3.14159265359f
#define SINE_TABLE_SIZE    256

static RingbufHandle_t rb = NULL;
static float volume = 0.8;
static float cutoff_frequency = 100.0;
static float filtered_value = 0.0;
static float filter_alpha = 0.0;

static adc_oneshot_unit_handle_t adc1_handle;
static dac_oneshot_handle_t dac_handle;
static dac_oneshot_handle_t dac_handle_raw;

// Add these with other static variables
static bool mode_switch = false;           // For button debouncing
static volatile bool is_bluetooth_mode = true;  // Mode flag
static float sine_phase = 0.0f;
static float sine_table[SINE_TABLE_SIZE];  // Lookup table for sine wave

// Forward declarations
void audio_processing_task(void *pvParameters);

// Helper function to convert Bluetooth address to string
static char *bda2str(const uint8_t *bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    return str;
}

// Bluetooth callback function for audio data
void bt_audio_data_callback(const uint8_t *data, uint32_t len) {
    if (rb && is_bluetooth_mode) {
        BaseType_t ret = xRingbufferSend(rb, (void*)data, len, pdMS_TO_TICKS(10));
        if (ret != pdTRUE) {
            // Clear some old data if buffer is full
            size_t item_size;
            void *item = xRingbufferReceive(rb, &item_size, 0);
            if (item) {
                vRingbufferReturnItem(rb, item);
            }
            // Try sending again
            ret = xRingbufferSend(rb, (void*)data, len, pdMS_TO_TICKS(10));
            if (ret != pdTRUE) {
                ESP_LOGW(TAG, "Ring buffer full, dropping %" PRIu32 " bytes", len);
            }
        }
    }
}

// GAP callback
static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT: {
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Authentication success: %s", param->auth_cmpl.device_name);
            } else {
                ESP_LOGE(TAG, "Authentication failed, status: %d", param->auth_cmpl.stat);
            }
            break;
        }
        case ESP_BT_GAP_PIN_REQ_EVT: {
            ESP_LOGI(TAG, "PIN request, using default PIN: 1234");
            esp_bt_pin_code_t pin_code = {'1', '2', '3', '4'};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            break;
        }
        case ESP_BT_GAP_CFM_REQ_EVT: {
            ESP_LOGI(TAG, "Confirmation request, auto-accepting");
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;
        }
        case ESP_BT_GAP_KEY_NOTIF_EVT: {
            ESP_LOGI(TAG, "Pairing key notification: %" PRIu32, param->key_notif.passkey);
            break;
        }
        case ESP_BT_GAP_KEY_REQ_EVT: {
            ESP_LOGI(TAG, "Pairing key request");
            break;
        }
        case ESP_BT_GAP_MODE_CHG_EVT: {
            ESP_LOGI(TAG, "Power mode change: %d", param->mode_chg.mode);
            break;
        }
        default: {
            ESP_LOGI(TAG, "GAP event: %d", event);
            break;
        }
    }
}

// Add these function declarations
static void button_isr_handler(void* arg);
static void init_gpio(void);
static void init_sine_table(void);
static float generate_sine_sample(float frequency);

// Add these function implementations
static void init_gpio(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL));
}

static void button_isr_handler(void* arg) {
    if (!mode_switch) {  // Simple debouncing
        is_bluetooth_mode = !is_bluetooth_mode;
        mode_switch = true;
    }
}

static void init_sine_table(void) {
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        sine_table[i] = sinf(2.0f * PI * i / SINE_TABLE_SIZE);
    }
}

static float generate_sine_sample(float frequency) {
    sine_phase += 2.0f * PI * frequency / SAMPLE_RATE;
    if (sine_phase >= 2.0f * PI) {
        sine_phase -= 2.0f * PI;
    }
    
    // Use lookup table for better performance
    float index = (sine_phase * SINE_TABLE_SIZE) / (2.0f * PI);
    int idx1 = (int)index;
    int idx2 = (idx1 + 1) % SINE_TABLE_SIZE;
    float frac = index - idx1;
    
    // Linear interpolation between table values
    return sine_table[idx1] * (1.0f - frac) + sine_table[idx2] * frac;
}

// Add these callback function declarations near the top with other declarations
static void bt_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);
static void bt_avrc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param);

// Add these callback function implementations before init_bluetooth()
static void bt_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param) {
    ESP_LOGI(TAG, "AVRC controller event: %d", event);
}

static void bt_avrc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param) {
    ESP_LOGI(TAG, "AVRC target event: %d", event);
}

// Modify the audio processing task
void audio_processing_task(void *pvParameters) {
    size_t item_size;
    uint8_t *data;
    float input_sample;
    uint8_t dac_buffer[DAC_BUFFER_SIZE];  // For filtered audio
    uint8_t dac_buffer_raw[DAC_BUFFER_SIZE];  // For raw audio
    int buffer_index = 0;
    TickType_t last_yield = xTaskGetTickCount();
    
    // Set highest priority for audio task
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
    
    while (1) {
        if (is_bluetooth_mode) {
            // Bluetooth audio processing
            data = (uint8_t *)xRingbufferReceive(rb, &item_size, pdMS_TO_TICKS(1));
            if (data) {
                // Process data in larger chunks
                for (int i = 0; i < item_size; i += 2) {
                    // Get input sample
                    input_sample = (float)((int16_t)(data[i] | (data[i+1] << 8))) / 32768.0;
                    
                    // Store raw audio
                    dac_buffer_raw[buffer_index] = (uint8_t)((input_sample * volume * 127.0) + 128.0);
                    
                    // Store filtered audio
                    filtered_value = filtered_value + filter_alpha * (input_sample - filtered_value);
                    dac_buffer[buffer_index] = (uint8_t)((filtered_value * volume * 127.0) + 128.0);
                    
                    buffer_index++;
                    
                    // Write to DACs when buffer is full or at end of data
                    if (buffer_index >= DAC_BUFFER_SIZE || i >= item_size - 2) {
                        // Write entire buffer to both DACs
                        for (int j = 0; j < buffer_index; j++) {
                            ESP_ERROR_CHECK(dac_oneshot_output_voltage(dac_handle, dac_buffer[j]));
                            ESP_ERROR_CHECK(dac_oneshot_output_voltage(dac_handle_raw, dac_buffer_raw[j]));
                        }
                        buffer_index = 0;
                        
                        // Yield only if enough time has passed
                        TickType_t now = xTaskGetTickCount();
                        if ((now - last_yield) >= pdMS_TO_TICKS(5)) {
                            vTaskDelay(1);
                            last_yield = now;
                        }
                    }
                }
                vRingbufferReturnItem(rb, (void *)data);
            } else {
                // No data, minimal yield
                vTaskDelay(1);
            }
        } else {
            // Frequency generator mode - only output to filtered DAC
            for (int i = 0; i < DAC_BUFFER_SIZE; i++) {
                float sine_sample = generate_sine_sample(cutoff_frequency);
                dac_buffer[i] = (uint8_t)((sine_sample * volume * 127.0) + 128.0);
            }
            
            // Write entire buffer to filtered DAC only
            for (int i = 0; i < DAC_BUFFER_SIZE; i++) {
                ESP_ERROR_CHECK(dac_oneshot_output_voltage(dac_handle, dac_buffer[i]));
            }
            
            // More precise timing for sine wave generation
            TickType_t delay_ticks = pdMS_TO_TICKS(1000 * DAC_BUFFER_SIZE / SAMPLE_RATE);
            if (delay_ticks == 0) delay_ticks = 1;
            vTaskDelay(delay_ticks);
        }
        
        // Handle mode switch with minimal delay
        if (mode_switch) {
            vTaskDelay(pdMS_TO_TICKS(20));  // Reduced debounce time
            mode_switch = false;
        }
    }
}

// Modify update_filter_parameters()
void update_filter_parameters() {
    int volume_raw, cutoff_raw;
    
    // Read potentiometers
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, VOLUME_POT_CHANNEL, &volume_raw));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, CUTOFF_POT_CHANNEL, &cutoff_raw));
    
    // Convert to usable values
    volume = volume_raw / 4095.0;
    
    if (is_bluetooth_mode) {
        cutoff_frequency = 1.0 + (cutoff_raw / 4095.0) * 99.0; // 1-100 Hz for filter
    } else {
        cutoff_frequency = 1.0 + (cutoff_raw / 4095.0) * 99.0; // 1-100 Hz for sine wave
    }
    
    // Update filter coefficient (only needed in bluetooth mode)
    if (is_bluetooth_mode) {
        float dt = 1.0 / SAMPLE_RATE;
        filter_alpha = dt / (1.0 / (2.0 * M_PI * cutoff_frequency) + dt);
    }
}

// A2DP callback
static void bt_av_hdl_stack_evt(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    switch (event) {
        case ESP_A2D_CONNECTION_STATE_EVT: {
            ESP_LOGI(TAG, "A2DP connection state: %d", param->conn_stat.state);
            break;
        }
        case ESP_A2D_AUDIO_STATE_EVT: {
            ESP_LOGI(TAG, "A2DP audio state: %d", param->audio_stat.state);
            break;
        }
        default:
            ESP_LOGI(TAG, "A2DP event: %d", event);
            break;
    }
}

void init_bluetooth() {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Release the memory of BLE if it is not needed
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    // Initialize controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    // Initialize Bluedroid
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Register GAP callback
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(bt_app_gap_cb));

    // Initialize AVRC first
    ESP_ERROR_CHECK(esp_avrc_ct_init());
    ESP_ERROR_CHECK(esp_avrc_ct_register_callback(bt_avrc_ct_cb));
    ESP_ERROR_CHECK(esp_avrc_tg_init());
    ESP_ERROR_CHECK(esp_avrc_tg_register_callback(bt_avrc_tg_cb));

    // Then initialize A2DP
    ESP_ERROR_CHECK(esp_a2d_register_callback(bt_av_hdl_stack_evt));
    ESP_ERROR_CHECK(esp_a2d_sink_init());
    ESP_ERROR_CHECK(esp_a2d_sink_register_data_callback(bt_audio_data_callback));

    // Set device name using the non-deprecated function
    ESP_ERROR_CHECK(esp_bt_gap_set_device_name("ESP32 Bass Speaker"));

    // SPP initialization
    esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
    bool esp_spp_enable_l2cap_ertm = true;
    esp_spp_cfg_t bt_spp_cfg = {
        .mode = esp_spp_mode,
        .enable_l2cap_ertm = esp_spp_enable_l2cap_ertm,
        .tx_buffer_size = 0, /* Only used for ESP_SPP_MODE_VFS mode */
    };
    
    ret = esp_spp_enhanced_init(&bt_spp_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    // Set default parameters for Secure Simple Pairing
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    // Set default parameters for Legacy Pairing
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

    // Set discoverable and connectable mode
    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE));

    // Disable sniff mode
    esp_bt_sleep_disable();

    ESP_LOGI(TAG, "Bluetooth initialized, device name is ESP32 Bass Speaker");
    ESP_LOGI(TAG, "Waiting for device to be discovered...");
}

void init_adc() {
    // ADC init configuration
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // ADC channel configurations
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, VOLUME_POT_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, CUTOFF_POT_CHANNEL, &config));
}

void init_dac() {
    // Initialize filtered output DAC
    dac_oneshot_config_t dac_config = {
        .chan_id = AUDIO_OUTPUT_CHANNEL,
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&dac_config, &dac_handle));

    // Initialize raw output DAC
    dac_oneshot_config_t dac_config_raw = {
        .chan_id = AUDIO_OUTPUT_CHANNEL_RAW,
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&dac_config_raw, &dac_handle_raw));
}

extern "C" {
    void app_main(void);
}

// Modify app_main()
void app_main(void) {
    // Initialize components
    init_bluetooth();
    init_adc();
    init_dac();
    init_gpio();
    init_sine_table();
    
    // Create ring buffer
    rb = xRingbufferCreate(BUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!rb) {
        ESP_LOGE(TAG, "Failed to create ring buffer");
        return;
    }
    
    // Create audio processing task with highest priority
    xTaskCreatePinnedToCore(
        audio_processing_task,
        "audio_task",
        8192,
        NULL,
        configMAX_PRIORITIES - 1,  // Highest priority
        NULL,
        1  // Run on core 1
    );
    
    // Parameter update task with very low priority
    xTaskCreatePinnedToCore(
        [](void* param) {
            while(1) {
                update_filter_parameters();
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        },
        "param_task",
        2048,
        NULL,
        tskIDLE_PRIORITY + 1,  // Just above idle priority
        NULL,
        0  // Run on core 0
    );
    
    // Delete the original task
    vTaskDelete(NULL);
}

