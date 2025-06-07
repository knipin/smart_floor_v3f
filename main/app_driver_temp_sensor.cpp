#include "app_driver_temp_sensor.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define TAG "temp_sensor"

// Убедитесь, что этот канал АЦП (и соответствующий GPIO) 
// совпадает с тем, куда подключен ваш датчик LM335Z.
// ADC_CHANNEL_0 обычно соответствует GPIO0 на многих платах ESP32-C6.
// Проверьте документацию на вашу плату или схему.
// Возможные каналы для ADC1 на ESP32-C6:
// ADC_CHANNEL_0 (GPIO0)
// ADC_CHANNEL_1 (GPIO1)
// ADC_CHANNEL_2 (GPIO2)
// ADC_CHANNEL_3 (GPIO3)
// ADC_CHANNEL_4 (GPIO4)
// ADC_CHANNEL_5 (GPIO5) - Этот пин может быть задействован для SPI flash на некоторых модулях, будьте осторожны.
#define TEMP_ADC_UNIT        ADC_UNIT_1
#define TEMP_ADC_CHANNEL     ADC_CHANNEL_0   // Пример: GPIO0
#define TEMP_ADC_ATTEN       ADC_ATTEN_DB_12 // Аттенюация 12 дБ (старое название ADC_ATTEN_DB_11). Позволяет измерять напряжение до VDD_ADC (около 3.1В-3.3В в зависимости от Vref). Это подходит для LM335Z, который выдает ~2.98В.
#define TEMP_ADC_BITWIDTH    ADC_BITWIDTH_DEFAULT // Для ESP32-C6 это 12 бит (макс. значение 4095)

// Константы для нашей ручной калибровки (используется, если встроенная калибровка ESP-IDF недоступна/неудачна)
#define MANUAL_CALIB_REF_VOLTAGE 4.079f // Наше откалиброванное опорное напряжение
#define ADC_MAX_VALUE_12_BIT     4095.0f // Максимальное значение для 12-битного АЦП ((1 << 12) - 1)

adc_oneshot_unit_handle_t adc_handle; // Объявлена как extern в .h
static adc_cali_handle_t adc_cali_handle = NULL; // Хэндл для калибровки, инициализируем NULL
static bool adc_calibrated = false; // Флаг, указывающий, удалось ли инициализировать калибровку

esp_err_t app_temp_sensor_init(void) {
    // Инициализация ADC Oneshot модуля
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = TEMP_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE, // ESP32-C6 не поддерживает режим ADC ULP
    };
    esp_err_t ret = adc_oneshot_new_unit(&unit_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s (%d)", esp_err_to_name(ret), ret);
        return ret;
    }

    // Конфигурация канала ADC Oneshot
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = TEMP_ADC_ATTEN,
        .bitwidth = TEMP_ADC_BITWIDTH,
    };
    ret = adc_oneshot_config_channel(adc_handle, TEMP_ADC_CHANNEL, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s (%d)", esp_err_to_name(ret), ret);
        // Если конфигурация канала не удалась, нужно освободить ранее созданный модуль АЦП
        adc_oneshot_del_unit(adc_handle); 
        adc_handle = NULL; // Обнуляем хэндл, чтобы избежать его использования
        return ret;
    }

    // Попытка калибровки АЦП (с использованием метода Curve Fitting)
    ESP_LOGI(TAG, "Attempting ADC calibration with curve fitting...");
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = TEMP_ADC_UNIT,
        .chan = TEMP_ADC_CHANNEL, // Для ESP32-C6 схема curve_fitting требует указания канала
        .atten = TEMP_ADC_ATTEN,
        .bitwidth = TEMP_ADC_BITWIDTH,
    };

    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
    if (ret == ESP_OK) {
        adc_calibrated = true;
        ESP_LOGI(TAG, "ADC calibration scheme 'curve_fitting' created successfully.");
    } else if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "ADC calibration scheme 'curve_fitting' is not supported (e.g., eFuse bits for Vref not burned). ESP-IDF ADC calibration will not be used.");
        // adc_cali_handle останется NULL, adc_calibrated останется false
    } else {
        ESP_LOGE(TAG, "Failed to create ADC calibration scheme 'curve_fitting': %s (%d)", esp_err_to_name(ret), ret);
        // adc_cali_handle останется NULL, adc_calibrated останется false
    }
    // Примечание: для ESP32-C6 также доступна калибровка 'line_fitting', но 'curve_fitting' обычно предпочтительнее, если поддерживается.

    return ESP_OK;
}

esp_err_t app_temp_sensor_read(float *temperature_celsius) {
    if (!adc_handle) {
        ESP_LOGE(TAG, "ADC unit not initialized. Call app_temp_sensor_init() first.");
        if (temperature_celsius) *temperature_celsius = -273.15f; // Возвращаем невозможное значение
        return ESP_FAIL; // Или более специфичную ошибку, например ESP_ERR_INVALID_STATE
    }

    int raw_adc_value = 0;
    esp_err_t ret = adc_oneshot_read(adc_handle, TEMP_ADC_CHANNEL, &raw_adc_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC oneshot read failed: %s (%d)", esp_err_to_name(ret), ret);
        if (temperature_celsius) *temperature_celsius = -273.15f;
        return ret;
    }

    float voltage_volts = 0.0f;
    bool voltage_obtained_from_idf_cali = false;

    // Шаг 1: Попытаться получить напряжение с помощью встроенной калибровки ESP-IDF
    if (adc_calibrated && adc_cali_handle != NULL) {
        int voltage_mv = 0;
        ret = adc_cali_raw_to_voltage(adc_cali_handle, raw_adc_value, &voltage_mv);
        if (ret == ESP_OK) {
            voltage_volts = (float)voltage_mv / 1000.0f;
            voltage_obtained_from_idf_cali = true;
            // ESP_LOGD(TAG, "Calibrated ADC: Raw=%d, Voltage_mV=%d -> %.4f V", raw_adc_value, voltage_mv, voltage_volts); // Для отладки
        } else {
            ESP_LOGW(TAG, "adc_cali_raw_to_voltage failed: %s (%d). Falling back to manual calculation.", esp_err_to_name(ret), ret);
            // Продолжаем, чтобы использовать ручной расчет
        }
    }

    // Шаг 2: Если встроенная калибровка не использовалась или не удалась, используем нашу ручную калибровку
    if (!voltage_obtained_from_idf_cali) {
        // ESP_LOGD(TAG, "Using manual voltage calculation: Raw=%d", raw_adc_value); // Для отладки
        voltage_volts = ((float)raw_adc_value / ADC_MAX_VALUE_12_BIT) * MANUAL_CALIB_REF_VOLTAGE;
    }

    // Шаг 3: Преобразование напряжения в температуру для LM335Z (10 мВ/К)
    float temperatureK = voltage_volts * 100.0f;
    if (temperature_celsius) { // Проверка, что указатель не NULL
        *temperature_celsius = temperatureK - 273.15f;
    }


    // Логирование результата (можно использовать ESP_LOGD для менее частого вывода)
    ESP_LOGI(TAG, "Sensor Read: Raw ADC=%d, Voltage=%.4f V, Temp=%.2f °C (IDF Cali Used: %s)", 
             raw_adc_value, voltage_volts, (temperature_celsius ? *temperature_celsius : -273.15f), voltage_obtained_from_idf_cali ? "Yes" : "No");

    return ESP_OK;
}

// Опционально: функция деинициализации, если нужно освобождать ресурсы
// esp_err_t app_temp_sensor_deinit(void) {
//     if (adc_cali_handle) {
//         ESP_LOGI(TAG, "Deleting ADC calibration scheme.");
//         // Для ESP32-C6 и curve fitting нет явной функции delete для adc_cali_handle,
//         // он освобождается при удалении ADC unit, если схема была с ним связана.
//         // Но если бы использовалась adc_cali_check_scheme(), то для нее есть adc_cali_delete_scheme_line_fitting()
//         // или adc_cali_delete_scheme_curve_fitting().
//         // Уточните в документации для вашей версии ESP-IDF и типа калибровки.
//         // Обычно для adc_oneshot с curve_fitting достаточно удалить adc_oneshot_unit.
//         // adc_cali_delete_scheme_curve_fitting(adc_cali_handle); // Если такая функция есть и нужна
//         adc_cali_handle = NULL;
//         adc_calibrated = false;
//     }
//     if (adc_handle) {
//         ESP_LOGI(TAG, "Deleting ADC oneshot unit.");
//         esp_err_t ret = adc_oneshot_del_unit(adc_handle);
//         if (ret != ESP_OK) {
//             ESP_LOGE(TAG, "Failed to delete adc_oneshot_unit: %s", esp_err_to_name(ret));
//         }
//         adc_handle = NULL;
//     }
//     return ESP_OK;
// }