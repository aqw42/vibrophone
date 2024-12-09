#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/dac_continuous.h"

class AudioOutput {
public:
    static constexpr size_t BUFFER_SIZE = 1024;

    static AudioOutput& getInstance() {
        static AudioOutput instance;
        return instance;
    }

    void init();
    void startAudioTask();
    
    // Getters and setters
    bool getBluetoothMode() const { return isBluetoothMode; }
    void setBluetoothMode(bool mode) { isBluetoothMode = mode; }
    QueueHandle_t getAudioQueue() const { return audioQueue; }

private:
    AudioOutput();
    ~AudioOutput() = default;
    AudioOutput(const AudioOutput&) = delete;
    AudioOutput& operator=(const AudioOutput&) = delete;

    // Audio processing methods
    static void audioProcessingTask(void* params);
    float generateSineSample(float freq);
    void updateFilterParameters();
    float mapExponentialFrequency(float input);
    float applyVolumeCurve(float rawVolume);

    // State variables
    bool isBluetoothMode;
    float volume;
    float frequency;
    float filterAlpha;
    float lastFilteredSample[2];  // One for each channel

    // Hardware handles and buffers
    QueueHandle_t audioQueue;
    dac_continuous_handle_t dacHandle;
    uint8_t rawBuffer[BUFFER_SIZE];
    uint8_t stereoBuffer[BUFFER_SIZE * 2];  // Double size for stereo output
}; 