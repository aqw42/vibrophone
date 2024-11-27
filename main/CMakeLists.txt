idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES bt driver esp_adc esp_hw_support freertos nvs_flash esp_system esp_common hal
)
