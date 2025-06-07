file(GLOB component_sources "*.cpp") # Ищем .cpp файлы

idf_component_register(SRCS                "${component_sources}"
                       INCLUDE_DIRS        "."
                       PRIV_INCLUDE_DIRS   "."
                                           # --- Пути к ESP-Matter ---
                                           "${ESP_MATTER_PATH}/components/esp_matter"
                                           "${ESP_MATTER_PATH}/device_hal/device/include"
                                           "${ESP_MATTER_PATH}/examples/common/utils"
                                           # --- Пути к Matter SDK (connectedhomeip) ---
                                           "${MATTER_SDK_PATH}/src"
                                           # Явно добавляем путь к сгенерированным заголовкам ZAP
                                           # Это критично для компиляции кода, использующего Matter кластеры
                                           "${MATTER_SDK_PATH}/zzz_generated/app-common/app-common/zap-generated"
                       REQUIRES
                           nvs_flash
                           log
                           driver
                           esp_matter
                           adc_oneshot
                           # Удаляем зависимости от драйверов света и кнопки
                           # led_driver
                           # espressif__button
) 