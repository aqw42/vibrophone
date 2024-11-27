#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
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
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"
#include "driver/dac_continuous.h"
#include "soc/dac_channel.h"

static const char *TAG = "BT_SPEAKER";
static char bda_str[18];
static const char *SPP_TAG = "SPP_TAG";

// Define ADC channels for potentiometers
#define VOLUME_POT_CHANNEL     ADC_CHANNEL_6  // GPIO34
#define CUTOFF_POT_CHANNEL     ADC_CHANNEL_7  // GPIO35

#define MIN_FREQ 1.5f
#define MAX_FREQ 50.0f

// Define DAC channels
#define AUDIO_OUTPUT_CHANNEL   DAC_CHAN_0   // GPIO25
#define AUDIO_OUTPUT_CHANNEL_RAW   DAC_CHAN_1   // GPIO26 for unfiltered audio
#define SAMPLE_RATE           44100
#define BUF_SIZE             (2048 * 8)   // Reduce buffer size to 16KB - large buffers can cause latency
#define DMA_BUFFER_SIZE      256
#define DMA_BUFFER_COUNT     3    // Number of DMA buffers to cycle through

// Add these near the top with other defines
#define DEBOUNCE_TIME_MS    1000   // Debounce time in milliseconds
#define BUTTON_PIN          GPIO_NUM_32
#define PI                 3.14159265359f
#define SINE_TABLE_SIZE    1024    // Size of the sine lookup table

static RingbufHandle_t rb = NULL;
static float volume = 0.8;
static float cutoff_frequency = 100.0;
static float filtered_value = 0.0;
static float filter_alpha = 0.0;

static adc_oneshot_unit_handle_t adc1_handle;
static dac_oneshot_handle_t dac_handle;
static dac_oneshot_handle_t dac_handle_raw;
static dac_continuous_handle_t dac_handle = NULL;
static uint8_t *dma_buffer = NULL;
static size_t dma_buffer_size = 0;
static QueueHandle_t dac_event_queue;

// Add these with other static variables
static volatile uint32_t last_button_press = 0;
static volatile bool is_bluetooth_mode = false;
static float sine_table[SINE_TABLE_SIZE];
static float phase_accumulator = 0.0f;
static float phase_increment = 0.0f;
static uint32_t mode_change_count = 0;  // To track number of mode changes

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
        BaseType_t ret = xRingbufferSend(rb, (void*)data, len, 0); // No waiting
        if (ret != pdTRUE) {
            // If buffer is full, clear half of it
            size_t item_size;
            void *item = xRingbufferReceive(rb, &item_size, 0);
            if (item) {
                vRingbufferReturnItem(rb, item);
            }
            // Try sending again
            xRingbufferSend(rb, (void*)data, len, 0);
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
        .intr_type = GPIO_INTR_ANYEDGE  // Changed to trigger on both edges
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL));
}

static void IRAM_ATTR button_isr_handler(void* arg) {
    uint32_t current_time = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
    
    // Only toggle mode on button press (when GPIO is low), not on release
    if (gpio_get_level(BUTTON_PIN) == 0) {  // Check if button is pressed (LOW)
        // Check if enough time has passed since last press
        if ((current_time - last_button_press) > DEBOUNCE_TIME_MS) {
            is_bluetooth_mode = !is_bluetooth_mode;
            last_button_press = current_time;
            mode_change_count++;
        }
    }
}

static void init_sine_table(void) {
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        sine_table[i] = sinf(2.0f * PI * i / SINE_TABLE_SIZE);
    }
}

static float generate_sine_sample(float frequency) {
    // Calculate how much the phase should advance per sample for desired frequency
    // For example, for 1Hz we need 2*PI radians per second
    // At 44100Hz sample rate, that means (2*PI*freq)/SAMPLE_RATE radians per sample
    phase_increment = 2.0f * PI * frequency / SAMPLE_RATE;
    
    // Update phase accumulator
    phase_accumulator += phase_increment;
    
    // Keep phase_accumulator in range [0, 2*PI]
    if (phase_accumulator >= 2.0f * PI) {
        phase_accumulator -= 2.0f * PI;
    }
    
    // Generate sine directly (more accurate than table lookup for this application)
    return sinf(phase_accumulator);
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

// Add this function to handle DAC events
static bool IRAM_ATTR dac_on_convert_done_callback(dac_continuous_handle_t handle, const dac_event_data_t *event, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;
    xQueueSendFromISR(queue, event, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

// Add this function to initialize DAC with DMA
void init_dac_dma() {
    // DAC continuous mode configuration
    dac_continuous_config_t cont_cfg = {
        .chan_mask = DAC_CHANNEL_MASK_ALL,
        .desc_num = DMA_BUFFER_COUNT,
        .buf_size = DMA_BUFFER_SIZE,
        .freq_hz = SAMPLE_RATE,
        .offset = 0,
        .clk_src = DAC_DIGI_CLK_SRC_DEFAULT,
        .chan_mode = DAC_CHANNEL_MODE_SIMUL,
    };
    
    ESP_ERROR_CHECK(dac_continuous_new_channels(&cont_cfg, &dac_handle));
    
    // Create event queue
    dac_event_queue = xQueueCreate(DMA_BUFFER_COUNT, sizeof(dac_event_data_t));
    assert(dac_event_queue);
    
    // Register callback
    dac_event_callbacks_t cbs = {
        .on_convert_done = dac_on_convert_done_callback,
        .on_stop = NULL,
    };
    ESP_ERROR_CHECK(dac_continuous_register_event_callback(dac_handle, &cbs, dac_event_queue));
    
    // Allocate DMA buffer
    dma_buffer_size = DMA_BUFFER_SIZE * DMA_BUFFER_COUNT;
    dma_buffer = (uint8_t *)heap_caps_malloc(dma_buffer_size, MALLOC_CAP_DMA);
    assert(dma_buffer);
    
    // Start DAC output
    ESP_ERROR_CHECK(dac_continuous_start(dac_handle));
}

// Modify the audio processing task
void audio_processing_task(void *pvParameters) {
    size_t item_size;
    uint8_t *data;
    float input_sample;
    dac_event_data_t dac_event;
    uint32_t buf_index = 0;
    
    while (1) {
        if (is_bluetooth_mode) {
            data = (uint8_t *)xRingbufferReceive(rb, &item_size, pdMS_TO_TICKS(1));
            if (data) {
                // Process data in chunks
                for (int i = 0; i < item_size; i += 2) {
                    // Convert to proper 16-bit sample (little-endian)
                    int16_t sample = (int16_t)(data[i] | (data[i+1] << 8));
                    input_sample = (float)sample / 32768.0f;
                    
                    // Apply volume and filtering
                    filtered_value = filtered_value + filter_alpha * (input_sample - filtered_value);
                    
                    // Fill DMA buffer
                    uint8_t dac_value = (uint8_t)((filtered_value * volume * 127.0f) + 128.0f);
                    dma_buffer[buf_index++] = dac_value;
                    
                    // When buffer is full, send it to DAC
                    if (buf_index >= DMA_BUFFER_SIZE) {
                        ESP_ERROR_CHECK(dac_continuous_write_cyclically(dac_handle, dma_buffer, DMA_BUFFER_SIZE, NULL));
                        buf_index = 0;
                        
                        // Wait for DMA transfer to complete
                        if (xQueueReceive(dac_event_queue, &dac_event, portMAX_DELAY) != pdTRUE) {
                            ESP_LOGE(TAG, "Failed to receive DAC event");
                        }
                    }
                }
                vRingbufferReturnItem(rb, (void *)data);
            }
        } else {
            // Sine wave generation mode
            float sine_sample = generate_sine_sample(cutoff_frequency);
            uint8_t dac_value = (uint8_t)((sine_sample + 1.0f) * 127.0f * volume + 0.5f);
            dma_buffer[buf_index++] = dac_value;
            
            if (buf_index >= DMA_BUFFER_SIZE) {
                ESP_ERROR_CHECK(dac_continuous_write_cyclically(dac_handle, dma_buffer, DMA_BUFFER_SIZE, NULL));
                buf_index = 0;
                
                if (xQueueReceive(dac_event_queue, &dac_event, portMAX_DELAY) != pdTRUE) {
                    ESP_LOGE(TAG, "Failed to receive DAC event");
                }
            }
        }
    }
}

// Modify update_filter_parameters()
void update_filter_parameters() {
    int volume_raw, cutoff_raw;
    
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, VOLUME_POT_CHANNEL, &volume_raw));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, CUTOFF_POT_CHANNEL, &cutoff_raw));
    
    // Update volume (0.0 to 1.0)
    volume = volume_raw / 4095.0f;
    
    if (is_bluetooth_mode) {
        // Bluetooth mode: cutoff frequency for filter
        cutoff_frequency = 1.0f + (cutoff_raw / 4095.0f) * 99.0f; // 1-100 Hz for filter
        float dt = 1.0f / SAMPLE_RATE;
        filter_alpha = dt / (1.0f / (2.0f * PI * cutoff_frequency) + dt);
    } else {

        float normalized = cutoff_raw / 4095.0f;
        cutoff_frequency = MIN_FREQ + (MAX_FREQ - MIN_FREQ) * (expf(normalized * normalized * normalized * logf(2.0f)) - 1.0f);
        
        // Add debug logging for frequency and ADC value
        ESP_LOGI(TAG, "ADC: %d, Normalized: %.3f, Sine frequency: %.2f Hz", 
                 cutoff_raw, normalized, cutoff_frequency);
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

extern "C" {
    void app_main(void);
}

// Add this function to periodically log mode status
static void mode_debug_task(void* pvParameters) {
    while(1) {
        ESP_LOGI(TAG, "Mode: %s, Changes: %lu, Last press: %lu ms ago", 
                 is_bluetooth_mode ? "BLUETOOTH" : "SINE WAVE",
                 mode_change_count,
                 (xTaskGetTickCount() * portTICK_PERIOD_MS) - last_button_press);
        vTaskDelay(pdMS_TO_TICKS(1000));  // Log every second
    }
}

// Add cleanup function
void cleanup_dac_dma() {
    if (dac_handle) {
        dac_continuous_stop(dac_handle);
        dac_continuous_del_channels(dac_handle);
    }
    if (dma_buffer) {
        heap_caps_free(dma_buffer);
        dma_buffer = NULL;
    }
    if (dac_event_queue) {
        vQueueDelete(dac_event_queue);
        dac_event_queue = NULL;
    }
}

// Update app_main()
void app_main(void) {
    // Initialize components
    init_bluetooth();
    init_adc();
    init_dac_dma();  // Use DMA initialization instead of regular DAC init
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
    
    // Create mode debug task
    xTaskCreatePinnedToCore(
        mode_debug_task,
        "mode_debug",
        2048,
        NULL,
        1,  // Low priority
        NULL,
        0   // Run on core 0
    );
    
    // Delete the original task
    vTaskDelete(NULL);
}

