#include "audio_output.h"
#include "gpio_config.h"
#include "esp_log.h"
#include <math.h>

static const char* TAG = "AUDIO";
static const int SAMPLE_RATE = 44100;
static const float PI = 3.14159265359f;

// Frequency range constants
static const float MIN_FREQ = 1.5f;
static const float MAX_FREQ = 50.0f;
static const float MIN_CUTOFF = 10.0f;
static const float MAX_CUTOFF = 100.0f;

// Volume curve parameters
static const float VOLUME_CURVE_EXPONENT = 2.0f;

AudioOutput::AudioOutput()
    : isBluetoothMode(true)
    , volume(0.5f)
    , frequency(20.0f)
    , filterAlpha(0.1f)
    , lastFilteredSample{0.0f, 0.0f}  // One for each channel
{
    ESP_LOGI(TAG, "Initializing audio output");
    audioQueue = xQueueCreate(32, BUFFER_SIZE);
    if (audioQueue == nullptr) {
        ESP_LOGE(TAG, "Failed to create audio queue");
    }
}

void AudioOutput::init() {
    ESP_LOGI(TAG, "Configuring DAC in continuous mode");

    dac_continuous_config_t dac_config = {
        .chan_mask = DAC_CHANNEL_MASK_ALL,
        .desc_num = 8,
        .buf_size = BUFFER_SIZE * 2,
        .freq_hz = SAMPLE_RATE,
        .offset = 0,
        .clk_src = DAC_DIGI_CLK_SRC_DEFAULT,
        .chan_mode = DAC_CHANNEL_MODE_ALTER,
        .write_mode = DAC_WRITE_MODE_ASYNC
    };

    esp_err_t ret = dac_continuous_new_channels(&dac_config, &dacHandle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create DAC channels: %s", esp_err_to_name(ret));
        return;
    }

    ret = dac_continuous_enable(dacHandle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable DAC: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "DAC initialized at %d Hz", SAMPLE_RATE);
}

void AudioOutput::startAudioTask() {
    ESP_LOGI(TAG, "Starting audio processing task");
    BaseType_t ret = xTaskCreatePinnedToCore(
        audioProcessingTask,
        "audio_task",
        4096,
        this,
        5,
        nullptr,
        1
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio processing task");
    }
}

float AudioOutput::generateSineSample(float freq) {
    static float phase = 0.0f;
    float sample = sinf(2.0f * PI * phase);
    phase += freq / SAMPLE_RATE;
    if (phase >= 1.0f) phase -= 1.0f;
    return sample;
}

float AudioOutput::mapExponentialFrequency(float input) {
    // Map input (0-1) to exponential frequency range
    if (isBluetoothMode) {
        return MIN_CUTOFF * powf(MAX_CUTOFF / MIN_CUTOFF, input);
    } else {
        return MIN_FREQ * powf(MAX_FREQ / MIN_FREQ, input);
    }
}

float AudioOutput::applyVolumeCurve(float rawVolume) {
    // Apply power curve for more natural volume control
    return powf(rawVolume, VOLUME_CURVE_EXPONENT);
}

void AudioOutput::updateFilterParameters() {
    float rawVolume = GPIOConfig::getInstance().readVolume();
    volume = applyVolumeCurve(rawVolume);
    
    float rawFreq = GPIOConfig::getInstance().readFrequency();
    if (isBluetoothMode) {
        float cutoff = mapExponentialFrequency(rawFreq);
        filterAlpha = 1.0f / (1.0f + SAMPLE_RATE / (2.0f * PI * cutoff));
        ESP_LOGD(TAG, "Updated filter - Cutoff: %.1f Hz, Alpha: %.3f, Volume: %.2f", cutoff, filterAlpha, volume);
    } else {
        frequency = mapExponentialFrequency(rawFreq);
        ESP_LOGD(TAG, "Updated sine wave - Freq: %.1f Hz, Volume: %.2f", frequency, volume);
    }
}

void AudioOutput::audioProcessingTask(void* params) {
    AudioOutput* audio = &AudioOutput::getInstance();
    const float dc_offset = 128.0f;
    const float scale = 128.0f;
    uint32_t underruns = 0;
    uint32_t total_writes = 0;

    ESP_LOGI(TAG, "Audio processing task started in %s mode", 
             audio->isBluetoothMode ? "Bluetooth" : "Sine Wave");

    while (true) {
        audio->updateFilterParameters();

        if (audio->isBluetoothMode) {
            if (xQueueReceive(audio->audioQueue, audio->rawBuffer, portMAX_DELAY)) {
                // Process received audio data
                for (int i = 0; i < BUFFER_SIZE; i++) {
                    float sample = (audio->rawBuffer[i] - dc_offset) / scale;
                    audio->stereoBuffer[i*2] = audio->rawBuffer[i];
                    audio->lastFilteredSample[1] = audio->lastFilteredSample[1] + 
                        audio->filterAlpha * (sample - audio->lastFilteredSample[1]);
                    float processed = audio->lastFilteredSample[1] * audio->volume;
                    processed = fmaxf(-1.0f, fminf(1.0f, processed));
                    audio->stereoBuffer[i*2 + 1] = (uint8_t)((processed * scale) + dc_offset);
                }
            }
        } else {
            // Generate sine wave
            for (int i = 0; i < BUFFER_SIZE; i++) {
                float sample = audio->generateSineSample(audio->frequency);
                float processed = sample * audio->volume;
                processed = fmaxf(-1.0f, fminf(1.0f, processed));
                uint8_t value = (uint8_t)((processed * scale) + dc_offset);
                audio->stereoBuffer[i*2] = value;
                audio->stereoBuffer[i*2 + 1] = value;
            }
        }
        
        size_t written;
        esp_err_t ret = dac_continuous_write_asynchronously(audio->dacHandle, 
            audio->stereoBuffer, 
            BUFFER_SIZE * 2, 
            &written, 
            DAC_CONTINUOUS_TIMEOUT_NEVER);
        total_writes++;

        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "DAC write failed: %s", esp_err_to_name(ret));
        } else if (written != BUFFER_SIZE * 2) {
            underruns++;
            if (underruns % 100 == 0) {  // Log every 100th underrun to avoid spam
                ESP_LOGW(TAG, "DAC underrun rate: %.1f%% (%lu/%lu)", 
                        (float)underruns / total_writes * 100, underruns, total_writes);
            }
        }
    }
} 