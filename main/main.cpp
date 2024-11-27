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

#define VOLUME_POT_CHANNEL ADC_CHANNEL_6 // GPIO34
#define CUTOFF_POT_CHANNEL ADC_CHANNEL_7 // GPIO35
#define MIN_FREQ 1.5f
#define MAX_FREQ 50.0f
#define AUDIO_OUTPUT_CHANNEL DAC_CHAN_0     // GPIO25
#define AUDIO_OUTPUT_CHANNEL_RAW DAC_CHAN_1 // GPIO26
#define SAMPLE_RATE 44100
#define BUF_SIZE (2048 * 4)
#define DMA_BUFFER_SIZE (128 * 4)
#define DMA_BUFFER_COUNT 4
#define DEBOUNCE_TIME_MS 1000
#define BUTTON_PIN GPIO_NUM_32
#define PI 3.14159265359f

static RingbufHandle_t rb = NULL;
static uint32_t sample_rate = SAMPLE_RATE;
static float volume = 0.5;
static float cutoff_frequency = MIN_FREQ;
static float filtered_value = 0.0;
static float filter_alpha = 0.0;
static adc_oneshot_unit_handle_t adc1_handle;
static dac_continuous_handle_t dac_handle_continuous = NULL;
static uint8_t *dma_buffer = NULL;
static size_t dma_buffer_size = 0;
static QueueHandle_t dac_event_queue;
static volatile uint32_t last_button_press = 0;
static volatile bool is_bluetooth_mode = false;
static float phase_accumulator = 0.0f;
static float phase_increment = 0.0f;

/**
 * @brief Converts a Bluetooth device address to a string representation
 * @param bda Pointer to the Bluetooth device address
 * @param str Buffer to store the resulting string
 * @param size Size of the buffer
 * @return Pointer to the string buffer, or NULL if parameters are invalid
 */
static char *bda2str(const uint8_t *bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18)
    {
        return NULL;
    }
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    return str;
}

/**
 * @brief Generates a sine wave sample at the specified frequency
 * @param frequency Target frequency in Hz
 * @return Sample value in the range [-1.0, 1.0]
 */
static float generate_sine_sample(float frequency)
{
    phase_increment = 2.0f * PI * frequency / sample_rate;
    phase_accumulator += phase_increment;
    if (phase_accumulator >= 2.0f * PI)
    {
        phase_accumulator -= 2.0f * PI;
    }
    return sinf(phase_accumulator);
}

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
    {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(TAG, "Authentication success: %s", param->auth_cmpl.device_name);
        }
        else
        {
            ESP_LOGE(TAG, "Authentication failed, status: %d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:
    {
        ESP_LOGI(TAG, "PIN request, using default PIN: 1234");
        esp_bt_pin_code_t pin_code = {'1', '2', '3', '4'};
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        break;
    }
    case ESP_BT_GAP_CFM_REQ_EVT:
    {
        ESP_LOGI(TAG, "Confirmation request, auto-accepting");
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    }
    case ESP_BT_GAP_KEY_NOTIF_EVT:
    {
        ESP_LOGI(TAG, "Pairing key notification: %" PRIu32, param->key_notif.passkey);
        break;
    }
    case ESP_BT_GAP_KEY_REQ_EVT:
    {
        ESP_LOGI(TAG, "Pairing key request");
        break;
    }
    case ESP_BT_GAP_MODE_CHG_EVT:
    {
        ESP_LOGI(TAG, "Power mode change: %d", param->mode_chg.mode);
        break;
    }
    default:
    {
        ESP_LOGI(TAG, "GAP event: %d", event);
        break;
    }
    }
}

static void bt_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    ESP_LOGI(TAG, "AVRC controller event: %d", event);
}

static void bt_avrc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param)
{
    ESP_LOGI(TAG, "AVRC target event: %d", event);
}

static void bt_av_hdl_stack_evt(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event)
    {
    case ESP_A2D_CONNECTION_STATE_EVT:
        ESP_LOGI(TAG, "A2DP connection state: %d", param->conn_stat.state);
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "A2DP audio state: %d", param->audio_stat.state);
        break;
    case ESP_A2D_AUDIO_CFG_EVT:
        {
            // Get the sample rate from A2DP configuration
            uint32_t new_sample_rate;
            switch (param->audio_cfg.mcc.cie.sbc[0] & 0xF0) {
                case 0x80:    // 16000
                    new_sample_rate = 16000;
                    break;
                case 0x40:    // 32000
                    new_sample_rate = 32000;
                    break;
                case 0x20:    // 44100
                    new_sample_rate = 44100;
                    break;
                case 0x10:    // 48000
                    new_sample_rate = 48000;
                    break;
                default:
                    ESP_LOGE(TAG, "Unknown sample rate!");
                    return;
            }
            
            if (new_sample_rate != sample_rate) {
                sample_rate = new_sample_rate;
                ESP_LOGI(TAG, "Sample rate changed to %lud Hz", sample_rate);
            }
        }
        break;
    default:
        ESP_LOGI(TAG, "A2DP event: %d", event);
        break;
    }
}

/**
 * @brief Bluetooth audio data callback
 * @param data Pointer to received audio data
 * @param len Length of received data in bytes
 * @details Handles incoming A2DP audio data and writes to ring buffer
 */
void bt_audio_data_callback(const uint8_t *data, uint32_t len)
{
    if (rb && is_bluetooth_mode)
    {
        BaseType_t ret = xRingbufferSend(rb, (void *)data, len, 0);
        if (ret != pdTRUE)
        {
            size_t item_size;
            void *item = xRingbufferReceive(rb, &item_size, 0);
            if (item)
            {
                vRingbufferReturnItem(rb, item);
            }
            xRingbufferSend(rb, (void *)data, len, 0);
        }
    }
}

/**
 * @brief DAC conversion complete callback
 * @param handle DAC handle
 * @param event Event data
 * @param user_data User data pointer (queue handle)
 * @return True if a high priority task was woken
 */
static bool IRAM_ATTR dac_on_convert_done_callback(dac_continuous_handle_t handle,
                                                   const dac_event_data_t *event,
                                                   void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;
    xQueueSendFromISR(queue, event, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

/**
 * @brief Button interrupt service routine
 * @param arg ISR argument (unused)
 * @details Handles mode switching between Bluetooth and sine wave generation
 *          with debouncing
 */
static void IRAM_ATTR button_isr_handler(void *arg)
{
    uint32_t current_time = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;

    if (gpio_get_level(BUTTON_PIN) == 0)
    {
        return;
    }

    if ((current_time - last_button_press) > DEBOUNCE_TIME_MS)
    {
        is_bluetooth_mode = !is_bluetooth_mode;
        last_button_press = current_time;
    }
}

/**
 * @brief Initializes GPIO for button input
 * @details Configures button pin with pull-down and interrupt on high level
 */
static void init_gpio(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE};
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL));
}

/**
 * @brief Initializes ADC for reading potentiometers
 * @details Configures ADC1 for reading volume and cutoff frequency potentiometers
 *          with 12-bit resolution and 11dB attenuation
 */
void init_adc(void)
{
    // Initialize ADC1
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // Configure ADC channels
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, VOLUME_POT_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, CUTOFF_POT_CHANNEL, &config));
}

/**
 * @brief Initializes DAC with DMA for continuous audio output
 * @details Sets up dual DAC channels with DMA for simultaneous output
 *          of filtered and unfiltered audio
 */
void init_dac_dma(void)
{
    dac_continuous_config_t cont_cfg = {
        .chan_mask = DAC_CHANNEL_MASK_ALL,
        .desc_num = DMA_BUFFER_COUNT,
        .buf_size = DMA_BUFFER_SIZE * 2,
        .freq_hz = SAMPLE_RATE,
        .offset = 0,
        .clk_src = DAC_DIGI_CLK_SRC_DEFAULT,
        .chan_mode = DAC_CHANNEL_MODE_ALTER};

    ESP_ERROR_CHECK(dac_continuous_new_channels(&cont_cfg, &dac_handle_continuous));

    dac_event_queue = xQueueCreate(DMA_BUFFER_COUNT, sizeof(dac_event_data_t));
    assert(dac_event_queue);

    dac_event_callbacks_t cbs = {
        .on_convert_done = dac_on_convert_done_callback,
        .on_stop = NULL,
    };
    ESP_ERROR_CHECK(dac_continuous_register_event_callback(dac_handle_continuous, &cbs, dac_event_queue));

    dma_buffer_size = DMA_BUFFER_SIZE * 2;
    dma_buffer = (uint8_t *)heap_caps_malloc(dma_buffer_size, MALLOC_CAP_DMA);
    assert(dma_buffer);

    memset(dma_buffer, 128, dma_buffer_size);

    ESP_ERROR_CHECK(dac_continuous_enable(dac_handle_continuous));

    size_t bytes_written;
    ESP_ERROR_CHECK(dac_continuous_write(dac_handle_continuous, dma_buffer,
                                         DMA_BUFFER_SIZE * 2, &bytes_written, portMAX_DELAY));
}

/**
 * @brief Initializes Bluetooth A2DP sink and SPP
 * @details Sets up Bluetooth classic mode, configures A2DP sink profile,
 *          and initializes SPP with security settings
 */
void init_bluetooth(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    //ESP_ERROR_CHECK(esp_bt_gap_register_callback(bt_app_gap_cb));

    //ESP_ERROR_CHECK(esp_avrc_ct_init());
    //ESP_ERROR_CHECK(esp_avrc_ct_register_callback(bt_avrc_ct_cb));
    //ESP_ERROR_CHECK(esp_avrc_tg_init());
    //ESP_ERROR_CHECK(esp_avrc_tg_register_callback(bt_avrc_tg_cb));

    ESP_ERROR_CHECK(esp_a2d_register_callback(bt_av_hdl_stack_evt));
    ESP_ERROR_CHECK(esp_a2d_sink_init());
    ESP_ERROR_CHECK(esp_a2d_sink_register_data_callback(bt_audio_data_callback));

    ESP_ERROR_CHECK(esp_bt_gap_set_device_name("ESP32 Bass Speaker"));

    /*esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
    bool esp_spp_enable_l2cap_ertm = true;
    esp_spp_cfg_t bt_spp_cfg = {
        .mode = esp_spp_mode,
        .enable_l2cap_ertm = esp_spp_enable_l2cap_ertm,
        .tx_buffer_size = 0,
    };

    ret = esp_spp_enhanced_init(&bt_spp_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s", __func__, esp_err_to_name(ret));
        return;
    }*/

    /*esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));*/

    /*esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);*/

    ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE));

    esp_bt_sleep_disable();

    ESP_LOGI(TAG, "Bluetooth initialized, device name is ESP32 Bass Speaker");
    ESP_LOGI(TAG, "Waiting for device to be discovered...");
}

/**
 * @brief Main audio processing task
 * @param pvParameters Task parameters (unused)
 * @details Handles audio data processing for both Bluetooth and sine wave modes,
 *          applies filtering, and outputs to DAC via DMA
 */
void audio_processing_task(void *pvParameters)
{
    size_t item_size;
    uint8_t *data;
    dac_event_data_t dac_event;
    uint32_t buf_index = 0;
    uint32_t current_sample_rate = sample_rate; // Track current sample rate

    while (1)
    {
        // Check if sample rate has changed
        if (current_sample_rate != sample_rate)
        {
            ESP_LOGI(TAG, "Reconfiguring DAC for sample rate: %lud Hz", sample_rate);
            dac_continuous_disable(dac_handle_continuous);

            dac_continuous_config_t cont_cfg = {
                .chan_mask = DAC_CHANNEL_MASK_ALL,
                .desc_num = DMA_BUFFER_COUNT,
                .buf_size = DMA_BUFFER_SIZE * 2,
                .freq_hz = sample_rate,
                .offset = 0,
                .clk_src = DAC_DIGI_CLK_SRC_DEFAULT,
                .chan_mode = DAC_CHANNEL_MODE_ALTER
            };
            ESP_ERROR_CHECK(dac_continuous_new_channels(&cont_cfg, &dac_handle_continuous));
            ESP_ERROR_CHECK(dac_continuous_enable(dac_handle_continuous));

            current_sample_rate = sample_rate; // Update current sample rate
        }

        if (is_bluetooth_mode)
        {
            data = (uint8_t *)xRingbufferReceive(rb, &item_size, pdMS_TO_TICKS(10));
            if (data)
            {
                for (int i = 0; i < item_size; i += 2)
                {
                    int16_t sample = (int16_t)(data[i] | (data[i + 1] << 8));
                    float input_sample = (float)sample / 32768.0f;

                    uint8_t unfiltered_value = (uint8_t)((input_sample * volume * 127.0f) + 128.0f);
                    dma_buffer[buf_index] = unfiltered_value;

                    filtered_value = filtered_value + filter_alpha * (input_sample - filtered_value);
                    dma_buffer[buf_index + 1] = (uint8_t)((filtered_value * volume * 127.0f) + 128.0f);

                    buf_index += 2;

                    if (buf_index >= DMA_BUFFER_SIZE * 2)
                    {
                        size_t bytes_written;
                        ESP_ERROR_CHECK(dac_continuous_write(dac_handle_continuous, dma_buffer,
                                                             DMA_BUFFER_SIZE * 2, &bytes_written, portMAX_DELAY));

                        if (xQueueReceive(dac_event_queue, &dac_event, portMAX_DELAY) != pdTRUE)
                        {
                            ESP_LOGE(TAG, "Failed to receive DAC event");
                        }
                        buf_index = 0;
                    }
                }
                vRingbufferReturnItem(rb, (void *)data);
            }
        }
        else
        {
            float sine_sample = generate_sine_sample(cutoff_frequency);
            uint8_t sine_value = (uint8_t)((sine_sample + 1.0f) * 127.0f * volume + 0.5f);

            dma_buffer[buf_index] = sine_value;
            dma_buffer[buf_index + 1] = sine_value;
            buf_index += 2;

            if (buf_index >= DMA_BUFFER_SIZE * 2)
            {
                size_t bytes_written;
                ESP_ERROR_CHECK(dac_continuous_write(dac_handle_continuous, dma_buffer,
                                                     DMA_BUFFER_SIZE * 2, &bytes_written, portMAX_DELAY));

                if (xQueueReceive(dac_event_queue, &dac_event, portMAX_DELAY) != pdTRUE)
                {
                    ESP_LOGE(TAG, "Failed to receive DAC event");
                }
                buf_index = 0;
            }
        }
    }
}

/**
 * @brief Updates filter and volume parameters based on potentiometer readings
 * @details Reads ADC values and updates:
 *          - Volume level (0.0 to 1.0)
 *          - Cutoff frequency (1-100 Hz in BT mode, 1.5-50 Hz in sine mode)
 *          - Filter alpha coefficient for low-pass filter
 */
void update_filter_parameters(void)
{
    int volume_raw, cutoff_raw;

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, VOLUME_POT_CHANNEL, &volume_raw));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, CUTOFF_POT_CHANNEL, &cutoff_raw));

    static int last_cutoff_raw = cutoff_raw;
    static int last_volume_raw = volume_raw;

    if (abs(cutoff_raw - last_cutoff_raw) > 20)
    {
        last_cutoff_raw = cutoff_raw;
        if (is_bluetooth_mode)
        {
            cutoff_frequency = 1.0f + (cutoff_raw / 4095.0f) * 99.0f;
            float dt = 1.0f / SAMPLE_RATE;
            filter_alpha = dt / (1.0f / (2.0f * PI * cutoff_frequency) + dt);
        }
        else
        {
            float normalized = cutoff_raw / 4095.0f;
            cutoff_frequency = MIN_FREQ + (MAX_FREQ - MIN_FREQ) *
                                              (expf(normalized * normalized * normalized * logf(2.0f)) - 1.0f);
        }
    }
    else if (abs(volume_raw - last_volume_raw) > 20)
    {
        last_volume_raw = volume_raw;
        volume = volume_raw / 4095.0f;
    }
    else
    {
        return;
    }

    ESP_LOGI(TAG, "Mode: %s, Volume : %d%% , Sine frequency: %.2f Hz\n",
             is_bluetooth_mode ? "BLUETOOTH" : "SINE WAVE", (int)(volume * 100), cutoff_frequency);
}

/**
 * @brief Main application entry point
 * @details Initializes all components and creates audio processing tasks
 */
extern "C"
{
    void app_main(void);
}

void app_main(void)
{
    init_bluetooth();
    init_adc();
    init_dac_dma();
    init_gpio();

    rb = xRingbufferCreate(BUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!rb)
    {
        ESP_LOGE(TAG, "Failed to create ring buffer");
        return;
    }

    xTaskCreatePinnedToCore(
        audio_processing_task,
        "audio_task",
        8192 * 2,
        NULL,
        configMAX_PRIORITIES - 1,
        NULL,
        1);

    xTaskCreatePinnedToCore(
        [](void *param)
        {
            while (1)
            {
                update_filter_parameters();
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        },
        "param_task",
        8192,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL,
        0);

    vTaskDelete(NULL);
}
