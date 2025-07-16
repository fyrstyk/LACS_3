#include "storage.h"
#include "hardware_config.h"
#include "sun_tracking.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "STORAGE";

nvs_handle_t nvs_handle_storage = 0;

void storage_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle_storage));
}

void save_calibration(void) {
    for (int i = 0; i < 7; i++) {
        if (motors[i].type == MOTOR_TYPE_CLOSED_LOOP) {
            char key[16];
            snprintf(key, sizeof(key), "cal_%d", i);
            nvs_set_blob(nvs_handle_storage, key, &motor_states[i].pulses_per_unit, sizeof(float));
        }
    }
    nvs_commit(nvs_handle_storage);
    ESP_LOGI(TAG, "Calibration saved");
}

void load_calibration(void) {
    for (int i = 0; i < 7; i++) {
        if (motors[i].type == MOTOR_TYPE_CLOSED_LOOP) {
            char key[16];
            snprintf(key, sizeof(key), "cal_%d", i);
            size_t length = sizeof(float);
            nvs_get_blob(nvs_handle_storage, key, &motor_states[i].pulses_per_unit, &length);
        }
    }
    ESP_LOGI(TAG, "Calibration loaded");
}

void save_home_bearing(void) {
    nvs_set_blob(nvs_handle_storage, "home_bearing", &home_bearing, sizeof(float));
    nvs_commit(nvs_handle_storage);
    ESP_LOGI(TAG, "Home bearing saved: %.1f°", home_bearing);
}

void load_home_bearing(void) {
    size_t length = sizeof(float);
    if (nvs_get_blob(nvs_handle_storage, "home_bearing", &home_bearing, &length) != ESP_OK) {
        home_bearing = 0.0f;
    }
    ESP_LOGI(TAG, "Home bearing loaded: %.1f°", home_bearing);
}

void save_location(void) {
    nvs_set_blob(nvs_handle_storage, "latitude", &latitude, sizeof(float));
    nvs_set_blob(nvs_handle_storage, "longitude", &longitude, sizeof(float));
    nvs_commit(nvs_handle_storage);
    ESP_LOGI(TAG, "Location saved: %.2f°N, %.2f°E", latitude, longitude);
}

void load_location(void) {
    size_t length = sizeof(float);
    if (nvs_get_blob(nvs_handle_storage, "latitude", &latitude, &length) != ESP_OK) {
        latitude = 52.77f;
    }
    length = sizeof(float);
    if (nvs_get_blob(nvs_handle_storage, "longitude", &longitude, &length) != ESP_OK) {
        longitude = -0.38f;
    }
    ESP_LOGI(TAG, "Location loaded: %.2f°N, %.2f°E", latitude, longitude);
}