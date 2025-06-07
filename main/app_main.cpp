#include <cstdint>
#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>
#include <adc_oneshot.h>
#include <common_macros.h>
#include <app_priv.h>
#include <app_reset.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

#include "pid_controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>

// Явное объявление функции pid_reset для уверенности в видимости
void pid_reset(pid_controller_t* controller);

static float g_target_temperature = 40.0f;
static pid_controller_t g_pid;
#define TEMP_CONTROL_TASK_PERIOD_MS 1000

static const char *TAG = "app_main";
uint16_t temp_endpoint_id = 0;

static void temp_control_task(void *arg)
{
    // Получаем атрибуты кластера Thermostat
    attribute_t *target_temp_attribute = attribute::get(temp_endpoint_id, Thermostat::Id, chip::app::Clusters::Thermostat::Attributes::TargetHeatingSetpoint::Id);
    attribute_t *hvac_mode_attribute = attribute::get(temp_endpoint_id, Thermostat::Id, chip::app::Clusters::Thermostat::Attributes::HVACMode::Id);

    // В рамках этого эндпоинта (Термостат) нам также может понадобиться атрибут MeasuredValue
    attribute_t *measured_temp_attribute = attribute::get(temp_endpoint_id, Thermostat::Id, chip::app::Clusters::Thermostat::Attributes::LocalTemperature::Id); // Используем LocalTemperature если MeasuredValue не доступен напрямую в Thermostat

    esp_matter_attr_val_t measured_val = esp_matter_invalid(NULL);
    esp_matter_attr_val_t target_val = esp_matter_invalid(NULL);
    esp_matter_attr_val_t hvac_mode_val = esp_matter_invalid(NULL);

    while (true) {
        float current_temp_celsius = -273.15f;
        esp_err_t err = app_temp_sensor_read(&current_temp_celsius);
        if (err == ESP_OK) {
            // Matter Temperature Measurement uses 100ths of a degree Celsius
            int16_t measured_value_matter = (int16_t)(current_temp_celsius * 100);
            if (measured_temp_attribute) {
                measured_val.type = ESP_MATTER_VAL_TYPE_INT16;
                measured_val.val.i16 = measured_value_matter;
                // Обновляем MeasuredValue (или LocalTemperature) кластера Thermostat
                attribute::update(temp_endpoint_id, Thermostat::Id, chip::app::Clusters::Thermostat::Attributes::LocalTemperature::Id, &measured_val);
            }

            // Также считываем текущий режим HVAC
            chip::app::Clusters::Thermostat::SystemModeEnum hvac_mode = chip::app::Clusters::Thermostat::SystemModeEnum::kOff;
            if (hvac_mode_attribute && attribute::get_val(hvac_mode_attribute, &hvac_mode_val) == ESP_OK && hvac_mode_val.type == ESP_MATTER_VAL_TYPE_ENUM8) {
                hvac_mode = (chip::app::Clusters::Thermostat::SystemModeEnum)hvac_mode_val.val.u8;
            }

            // Обновляем g_target_temperature только если режим работы Нагрев и атрибут TargetHeatingSetpoint доступен
            if (hvac_mode == chip::app::Clusters::Thermostat::SystemModeEnum::kHeat && target_temp_attribute && attribute::get_val(target_temp_attribute, &target_val) == ESP_OK && target_val.type == ESP_MATTER_VAL_TYPE_INT16) {
                 g_target_temperature = (float)target_val.val.i16 / 100.0f;
            } else {
                 // Use default target temperature if attribute not available or invalid
                 g_target_temperature = 40.0f; // Default target if Matter attribute is not set/readable
            }

            // Логика PID-контроля и управления нагревателем должна быть активна только в режиме нагрева
            if (hvac_mode == chip::app::Clusters::Thermostat::SystemModeEnum::kHeat) {
                 float power = pid_compute(&g_pid, g_target_temperature, current_temp_celsius);
                 app_heater_set_power(power);
                 ESP_LOGI("temp_ctrl", "T=%.2f°C → power=%.1f%% (target=%.1f Matter=%.2f Mode=%u)", current_temp_celsius, power, g_target_temperature, (float)target_val.val.i16/100.0f, hvac_mode);
            } else {
                 // Если не в режиме нагрева, выключаем нагреватель и сбрасываем PID
                 app_heater_set_power(0.0f);
                 pid_reset(&g_pid);
                 ESP_LOGI("temp_ctrl", "T=%.2f°C → Heater OFF (Mode=%u)", (uint8_t)hvac_mode); // Удален current_temp_celsius из лога, так как он не используется в этой ветке
            }
        } else {
            ESP_LOGE("temp_ctrl", "Failed to read temperature");
        }
        vTaskDelay(pdMS_TO_TICKS(TEMP_CONTROL_TASK_PERIOD_MS));
    }
}

void app_main() {
    esp_err_t err;

    /* Initialize the ESP NVS layer */
    err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Start OpenThread task */
    ESP_ERROR_CHECK(esp_matter::openthread::task::init());
    ESP_ERROR_CHECK(openthread_launch_init());
#endif

    // Initialize the temperature sensor and heater drivers
    ESP_ERROR_CHECK(app_temp_sensor_init());
    ESP_ERROR_CHECK(app_heater_init());

    // Initialize PID controller
    // Примерные коэффициенты PID. Возможно, потребуется их подстроить.
    // params: kp, ki, kd, min_output, max_output
    pid_init(&g_pid, 5.0f, 0.1f, 2.0f, 0.0f, 100.0f);

    // Create the temperature control task
    xTaskCreate(temp_control_task, "temp_ctrl", 4096, NULL, configMAX_PRIORITIES - 5, NULL);

    // Удаляем инициализацию драйверов света и кнопки
    // app_driver_handle_t light_handle = app_driver_light_init();
    // app_driver_handle_t button_handle = app_driver_button_init();
    // app_reset_button_register(button_handle); // Удаляем регистрацию кнопки сброса

    node::config_t node_config;
    // Оригинальный способ создания узла и установки колбэков
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    // Удаляем создание эндпоинта extended_color_light
    /*
    extended_color_light::config_t light_config;
    light_config.on_off.on_off = DEFAULT_POWER;
    light_config.on_off.lighting.start_up_on_off = nullptr;
    light_config.level_control.current_level = DEFAULT_BRIGHTNESS;
    light_config.level_control.on_level = DEFAULT_BRIGHTNESS;
    light_config.level_control.lighting.start_up_current_level = DEFAULT_BRIGHTNESS;
    light_config.color_control.color_mode = (uint8_t)ColorControl::ColorMode::kColorTemperature;
    light_config.color_control.enhanced_color_mode = (uint8_t)ColorControl::ColorMode::kColorTemperature;
    light_config.color_control.color_temperature.startup_color_temperature_mireds = nullptr;

    // Оригинальный способ создания эндпоинта
    // priv_data здесь это light_handle
    endpoint_t *endpoint = extended_color_light::create(node, &light_config, ENDPOINT_FLAG_NONE, light_handle);
    ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "Failed to create extended color light endpoint"));

    light_endpoint_id = endpoint::get_id(endpoint);
    ESP_LOGI(TAG, "Light created with endpoint_id %d", light_endpoint_id);
    */

    // Удаляем отложенное сохранение для атрибутов света
    /*
    attribute_t *current_level_attribute = attribute::get(light_endpoint_id, LevelControl::Id, LevelControl::Attributes::CurrentLevel::Id);
    attribute::set_deferred_persistence(current_level_attribute);

    attribute_t *current_x_attribute = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentX::Id);
    attribute::set_deferred_deferred_persistence(current_x_attribute);
    attribute_t *current_y_attribute = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentY::Id);
    attribute::set_deferred_persistence(current_y_attribute);
    attribute_t *color_temp_attribute = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::ColorTemperatureMireds::Id);
    attribute::set_deferred_persistence(color_temp_attribute);
    */

    // Create Thermostat endpoint
    // Используем структуру config_t для Thermostat из esp_matter_endpoint.h, которая не включает Temperature Measurement
    thermostat::config_t thermostat_config;
    // Инициализируем только те поля, которые есть в endpoint::thermostat::config_t
    // Согласно esp_matter_endpoint.h, эта структура наследует от app_with_group_config и содержит cluster::thermostat::config_t и cluster::scenes_management::config_t
    // Нам нужно инициализировать thermostat. (Поля scenes_management можно оставить по умолчанию, если не используются сцены)

    // Инициализация полей для cluster::thermostat::config_t
    thermostat_config.thermostat.target_heating_setpoint = 4000; // Default 40.00 C in 100ths
    thermostat_config.thermostat.min_heat_setpoint_limit = 1500; // 15.00 C
    thermostat_config.thermostat.max_heat_setpoint_limit = 4500; // 45.00 C
    thermostat_config.thermostat.hvac_mode = (uint8_t)chip::app::Clusters::Thermostat::SystemModeEnum::kHeat; // Start in Heat mode
    // Другие поля thermostat_config.thermostat могут быть инициализированы здесь при необходимости

    // Передаем nullptr в качестве priv_data, так как специфический драйвер (например, датчик температуры)
    // управляется отдельной задачей (temp_control_task) и не привязан напрямую к этому эндпоинту
    // через app_attribute_update_cb. Если бы нам нужно было реагировать на запись атрибутов Thermostat
    // через app_attribute_update_cb и передавать туда handle драйвера, мы бы передали handle здесь.
    // Сейчас это не требуется, т.к. мы считываем атрибуты изнутри temp_control_task.
    endpoint_t *thermostat_endpoint = thermostat::create(node, &thermostat_config, ENDPOINT_FLAG_NONE, nullptr);
    ABORT_APP_ON_FAILURE(thermostat_endpoint != nullptr, ESP_LOGE(TAG, "Failed to create thermostat endpoint"));

    temp_endpoint_id = endpoint::get_id(thermostat_endpoint);
    ESP_LOGI(TAG, "Thermostat created with endpoint_id %d", temp_endpoint_id);

    // Set deferred persistence for Thermostat attributes (например, TargetHeatingSetpoint и HVACMode)
    attribute_t *target_heat_attribute = attribute::get(temp_endpoint_id, Thermostat::Id, chip::app::Clusters::Thermostat::Attributes::TargetHeatingSetpoint::Id);
    attribute::set_deferred_persistence(target_heat_attribute);

    attribute_t *hvac_mode_attribute = attribute::get(temp_endpoint_id, Thermostat::Id, chip::app::Clusters::Thermostat::Attributes::HVACMode::Id);
    attribute::set_deferred_persistence(hvac_mode_attribute);

    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

    // Matter console
    esp_matter_console_init();
    esp_matter_console_start();

#if CONFIG_ENABLE_OTA_REQUESTOR
    esp_matter_ota_requestor_init();
#endif

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter_repl_shell_init();
#endif
}