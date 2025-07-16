#ifndef OTA_H
#define OTA_H

#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"

// OTA upload structure
typedef struct {
    bool in_progress;
    size_t bytes_written;
    size_t total_size;
    char status_msg[64];
    esp_ota_handle_t update_handle;
    const esp_partition_t *update_partition;
} ota_upload_t;

// Function prototypes
esp_err_t ota_page_handler(httpd_req_t *req);
esp_err_t ota_upload_handler(httpd_req_t *req);
esp_err_t ota_upload_status_handler(httpd_req_t *req);

// Global OTA state
extern ota_upload_t ota_upload;

#endif // OTA_H