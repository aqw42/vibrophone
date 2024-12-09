# ESP32 Bass Speaker Project Requirements

## Overview
This project implements a dual-mode audio system on an ESP32 microcontroller, capable of functioning as both a Bluetooth audio receiver and a sine wave generator with real-time parameter control.

## Functional Requirements

### 1. Operating Modes
- **Bluetooth A2DP Sink Mode**
  - Receives audio streams from Bluetooth audio sources
  - Applies low-pass filtering with adjustable cutoff frequency (10-100 Hz)
  - Controls volume via potentiometer
  - Device name: "ESP32 Bass Speaker"

- **Sine Wave Generator Mode**
  - Generates pure sine waves
  - Frequency range: 1.5 Hz to 50 Hz
  - Volume control via potentiometer
  - Exponential frequency mapping for fine control

### 2. User Interface
- Mode switching via physical button (GPIO 32)
- Real-time parameter control through two potentiometers:
  - Volume control (GPIO 34)
  - Frequency/cutoff control (GPIO 35)

### 3. Audio Output
- Dual DAC output channels:
  - Raw audio output (GPIO 26)
  - Filtered audio output (GPIO 25)
- 44.1 kHz default sample rate (adjustable based on Bluetooth source)
- DMA-based continuous audio output

## Technical Specifications

### Hardware Requirements
- ESP32 microcontroller
- 2x potentiometers
- 1x push button
- Audio output circuitry for DAC channels

### Software Framework
- ESP-IDF (Espressif IoT Development Framework)
  - Version: v5.x
  - Components used:
    - esp_bt (Bluetooth Classic)
    - esp_adc
    - esp_dac
    - driver
    - freertos
- FreeRTOS for task management
  - Version: 10.5.x (included in ESP-IDF)

### Build Requirements
- ESP-IDF toolchain
  - Minimum CMake version: 3.16
  - Supported compilers: 
    - gcc 12.2.0 (xtensa-esp32-elf)
    - clang 15.0.0

### ESP-IDF Configuration (menuconfig)
```
# Bluetooth Configuration
CONFIG_BT_ENABLED=y
CONFIG_BT_CLASSIC_ENABLED=y
CONFIG_BT_A2DP_ENABLE=y
CONFIG_BT_A2DP_SINK_ENABLE=y
CONFIG_BT_BLE_DISABLED=y
CONFIG_BT_DEVICE_NAME="ESP32 Bass Speaker"

# FreeRTOS Configuration
CONFIG_FREERTOS_HZ=1000
CONFIG_FREERTOS_UNICORE=n
CONFIG_FREERTOS_TIMER_TASK_STACK_DEPTH=3072

# Audio Configuration
CONFIG_DAC_DMA_AUTO_16BIT_ALIGN=y
CONFIG_DAC_DMA_DATA_WIDTH_16BIT=y
CONFIG_DAC_CTRL_DIGI_FORCE_ENABLE=y

# Memory Configuration
CONFIG_SPIRAM_SUPPORT=y
CONFIG_SPIRAM_USE_MALLOC=y
CONFIG_SPIRAM_TYPE_AUTO=y
CONFIG_SPIRAM_SIZE=-1
CONFIG_SPIRAM_SPEED_80M=y

# ADC Configuration
CONFIG_ADC_FORCE_XPD_FSM=y
CONFIG_ADC_CAL_EFUSE_TP_ENABLE=y
CONFIG_ADC_CAL_LUT_ENABLE=y
```

### Partition Table
```csv
# Name,   Type, SubType, Offset,  Size, Flags
nvs,      data, nvs,     0x9000,  0x4000,
otadata,  data, ota,     0xd000,  0x2000,
phy_init, data, phy,     0xf000,  0x1000,
factory,  app,  factory, 0x10000, 1M,
storage,  data, spiffs,  ,        0x100000,
```

Partition explanation:
- nvs: Non-volatile storage for system parameters
- otadata: OTA (Over The Air) update data
- phy_init: PHY initialization data
- factory: Main application code (1MB)
- storage: Optional SPIFFS partition for future use (1MB)

### Key Components
- **Bluetooth Stack**
  - Classic Bluetooth (A2DP Sink profile)
  - AVRC (Audio/Video Remote Control) support
  - GAP (Generic Access Profile)

- **Audio Processing**
  - Sample rate: 44.1 kHz (default)
  - Buffer size: 1024 bytes
  - DMA buffer count: 32
  - 8-bit audio resolution

- **ADC Configuration**
  - 12-bit resolution
  - 12dB attenuation
  - One-shot mode

- **DAC Configuration**
  - Continuous mode with DMA
  - Dual channel output
  - Alternating channel mode

### Task Architecture
- Audio processing task (Core 1)
- Parameter update task (Core 0)
- Main task for system initialization

### Memory Requirements
- DMA capable memory for audio buffers
- Task stack sizes:
  - Audio task: 16 KB
  - Parameter task: 8 KB

### Dependencies
- FreeRTOS
- ESP32 Bluetooth APIs
- ESP32 ADC/DAC drivers
- ESP32 GPIO drivers

### Project Structure
```
esp32_bass_speaker/
├── CMakeLists.txt 
├── sdkconfig 
├── README.md 
├── requirements.md
└── main/
    ├── CMakeLists.txt
    ├── include/
    │   ├── bluetooth_handler.h
    │   ├── audio_output.h
    │   └── gpio_config.h
    └── src/
        ├── main.cpp       
        ├── bluetooth_handler.cpp
        ├── audio_output.cpp
        └── gpio_config.cpp
```

The project follows the standard ESP-IDF project structure with:
- Separate header and implementation files for better organization
- Component-based architecture for reusable modules
- Clear separation of concerns between different functionalities
- Standard build system configuration files

### API Description

#### bluetooth_handler.h
```cpp
// Handles Bluetooth A2DP sink functionality
void init_bluetooth(void);                    // Initialize Bluetooth stack and A2DP sink
void bt_audio_data_callback(                  // Callback for received A2DP audio data
    const uint8_t *data, 
    uint32_t len
);
void bt_app_gap_cb(                          // GAP event callback
    esp_bt_gap_cb_event_t event, 
    esp_bt_gap_cb_param_t *param
);
```

#### audio_output.h
```cpp
// Handles audio processing and DAC output
void init_dac_dma(void);                     // Initialize DAC with DMA
void audio_processing_task(void *params);     // Main audio processing task
float generate_sine_sample(float frequency);  // Generate sine wave sample
void update_filter_parameters(void);          // Update filter based on potentiometer readings
```

#### gpio_config.h
```cpp
// Handles GPIO configuration and ADC
void init_gpio(void);                        // Initialize GPIO for button input
void init_adc(void);                         // Initialize ADC for potentiometers
void IRAM_ATTR button_isr_handler(void *arg);// Button interrupt handler
```

#### main.cpp
```cpp
// Main application entry point
extern "C" void app_main(void);              // Main application entry point
```

### Key Data Structures
```cpp
// Audio configuration
static QueueHandle_t audio_queue;            // Queue for audio data
static dac_continuous_handle_t dac_handle;   // DAC handle
static adc_oneshot_unit_handle_t adc1_handle;// ADC handle

// Audio parameters
static float volume;                         // Current volume level (0.0-1.0)
static float cutoff_frequency;               // Current cutoff/sine frequency
static float filter_alpha;                   // Low-pass filter coefficient
static bool is_bluetooth_mode;               // Current operating mode
```
