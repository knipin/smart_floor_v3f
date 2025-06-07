/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include <esp_matter.h>
#include <esp_matter_attribute_utils.h>
#include <app_priv.h>

static const char *TAG = "app_driver";

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    // Этот колбэк остается, но его логика теперь пустая, т.к. основные обновления
    // атрибутов Thermostat происходят через polling в temp_control_task.
    // Если потребуется обработка команд Thermostat (например, SetHeatingSetpoint),
    // она может быть добавлена здесь в будущем.
    ESP_LOGI(TAG, "Attribute update callback (driver): type: PRE_UPDATE, endpoint: %u, cluster: %lu, attribute: %lu",
             endpoint_id, cluster_id, attribute_id);
    return ESP_OK;
} 