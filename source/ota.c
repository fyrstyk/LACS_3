#include "ota.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "cJSON.h"
#include <string.h>

static const char *TAG = "OTA";

ota_upload_t ota_upload = { 0 };

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

esp_err_t ota_upload_handler(httpd_req_t *req) {
    char buf[1024];
    int remaining = req->content_len;
    bool header_checked = false;

    ESP_LOGI(TAG, "OTA: Upload started, size = %d bytes", remaining);

    ota_upload.update_partition = esp_ota_get_next_update_partition(NULL);
    if (ota_upload.update_partition == NULL) {
        ESP_LOGE(TAG, "OTA: No available update partition");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    esp_err_t err = esp_ota_begin(ota_upload.update_partition, OTA_SIZE_UNKNOWN, &ota_upload.update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: esp_ota_begin failed (%s)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    ota_upload.in_progress = true;
    ota_upload.bytes_written = 0;
    ota_upload.total_size = remaining;
    strcpy(ota_upload.status_msg, "Uploading...");

    while (remaining > 0) {
        int read_len = httpd_req_recv(req, buf, MIN(sizeof(buf), remaining));
        if (read_len <= 0) {
            ESP_LOGE(TAG, "OTA: Error receiving data");
            esp_ota_abort(ota_upload.update_handle);
            ota_upload.in_progress = false;
            return ESP_FAIL;
        }

        if (!header_checked) {
            if (buf[0] != 0xE9) {
                ESP_LOGE(TAG, "OTA: Invalid header byte: 0x%02X", buf[0]);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid firmware");
                esp_ota_abort(ota_upload.update_handle);
                ota_upload.in_progress = false;
                return ESP_FAIL;
            }
            header_checked = true;
        }

        err = esp_ota_write(ota_upload.update_handle, (const void *)buf, read_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OTA: Write failed (%s)", esp_err_to_name(err));
            esp_ota_abort(ota_upload.update_handle);
            ota_upload.in_progress = false;
            return ESP_FAIL;
        }

        remaining -= read_len;
        ota_upload.bytes_written += read_len;
        snprintf(ota_upload.status_msg, sizeof(ota_upload.status_msg), "Uploading: %d%%", 
                 (ota_upload.bytes_written * 100) / ota_upload.total_size);
    }

    err = esp_ota_end(ota_upload.update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: esp_ota_end failed (%s)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        ota_upload.in_progress = false;
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(ota_upload.update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: Failed to set boot partition (%s)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot partition failed");
        ota_upload.in_progress = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA: Firmware update complete. Rebooting...");
    strcpy(ota_upload.status_msg, "OTA Success! Rebooting...");
    ota_upload.in_progress = false;

    httpd_resp_sendstr(req, "{\"success\":true,\"msg\":\"OTA completed. Restarting...\"}");

    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return ESP_OK;
}

esp_err_t ota_upload_status_handler(httpd_req_t *req) {
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "in_progress", ota_upload.in_progress);
    cJSON_AddNumberToObject(json, "progress", ota_upload.total_size > 0 ?
        (ota_upload.bytes_written * 100) / ota_upload.total_size : 0);
    cJSON_AddStringToObject(json, "status", ota_upload.status_msg);

    char *resp_str = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp_str, strlen(resp_str));

    free(resp_str);
    cJSON_Delete(json); 
    return ESP_OK;
}

esp_err_t ota_page_handler(httpd_req_t *req) {
    const char *html_response =
        "<!DOCTYPE html><html><body>"
        "<h3>OTA Firmware Upload</h3>"
        "<input type='file' id='fileInput'><br><br>"
        "<button onclick='upload()'>Upload Firmware</button><br><br>"
        "<div id='status'></div>"
        "<script>"
        "function upload() {"
        "  const file = document.getElementById('fileInput').files[0];"
        "  if (!file) return alert('Please select a file');"
        "  document.getElementById('status').innerText = 'Uploading...';"
        "  fetch('/api/ota/upload', {"
        "    method: 'POST',"
        "    headers: { 'X-Filename': file.name },"
        "    body: file"
        "  }).then(r => r.text()).then(t => {"
        "    document.getElementById('status').innerText = 'Upload complete. Rebooting...';"
        "  }).catch(e => {"
        "    document.getElementById('status').innerText = 'Error: ' + e;"
        "  });"
        "}"
        "</script></body></html>";

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html_response, HTTPD_RESP_USE_STRLEN);
}